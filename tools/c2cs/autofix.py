#!/usr/bin/env python3
"""Compiler driven fixups for converted C# files.

Builds the library, reads the errors, and applies mechanical fixes for the error codes that have
an unambiguous fix (for example a missing or extra ref on an argument). Repeats until the build
makes no more progress. Run cleanup first for the one-time text rewrites.

usage: autofix.py [--cleanup]
"""

import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
LIB = os.path.join(REPO, "src", "Box2DNet")
PROJECT = os.path.join(LIB, "Box2DNet.csproj")

ERROR = re.compile(r"^(.*?\.cs)\((\d+),(\d+)\): error (CS\d+): (.*?) \[")


def find_call(text, start):
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return i
    return -1


def find_block_end(text, start):
    """text[start] is '{' - return index after the matching '}'."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return i + 1
    return -1


def remove_statements(text, name):
    """Remove statements that are calls to name( ... );"""
    pattern = re.compile(r"^[ \t]*%s\s*\(" % re.escape(name), re.M)
    while True:
        m = pattern.search(text)
        if not m:
            return text
        close = find_call(text, m.end() - 1)
        end = text.index(";", close) + 1
        if end < len(text) and text[end] == "\n":
            end += 1
        text = text[: m.start()] + text[end:]


def remove_recording_blocks(text):
    """Remove if blocks that only run while recording."""
    pattern = re.compile(r"^[ \t]*if \( ([^\n]*recording[^\n]*) \)\n[ \t]*\{", re.M)
    pos = 0
    while True:
        m = pattern.search(text, pos)
        if not m:
            return text
        if "!= null" not in m.group(1) and "rec" not in m.group(1):
            pos = m.end()
            continue
        end = find_block_end(text, m.end() - 1)
        if end < len(text) and text[end] == "\n":
            end += 1
        text = text[: m.start()] + text[end:]


def cleanup(path):
    text = open(path, encoding="utf-8").read()
    orig = text
    for name in ("B2_REC", "B2_REC_CREATE", "_Static_assert", "TracyCFrame", "Prefetch", "b2Prefetch"):
        text = remove_statements(text, name)
    text = re.sub(r"^[ \t]*TracyCFrame;?\s*\n", "", text, flags=re.M)
    text = remove_recording_blocks(text)
    text = re.sub(r"\bUINT8_MAX\b", "byte.MaxValue", text)
    text = re.sub(r"\bUINT_MAX\b", "uint.MaxValue", text)
    text = re.sub(r"\bINT_MAX\b", "int.MaxValue", text)
    if text != orig:
        open(path, "w", encoding="utf-8", newline="\n").write(text)
        print("cleaned", os.path.relpath(path, REPO))


def build():
    r = subprocess.run(["dotnet", "build", PROJECT, "-v", "q", "-clp:NoSummary"], capture_output=True,
                       text=True, encoding="utf-8", errors="replace")
    errors = {}
    for line in r.stdout.splitlines():
        m = ERROR.match(line.strip())
        if m:
            key = (m.group(1), int(m.group(2)), int(m.group(3)), m.group(4))
            errors[key] = m.group(5)
    return errors


def apply(errors):
    by_file = {}
    for (path, line, col, code), msg in errors.items():
        by_file.setdefault(path, []).append((line, col, code, msg))
    changed = 0
    for path, items in by_file.items():
        lines = open(path, encoding="utf-8").read().split("\n")
        # apply right to left so columns stay valid
        for line, col, code, msg in sorted(items, key=lambda x: (x[0], x[1]), reverse=True):
            i, c = line - 1, col - 1
            text = lines[i]
            if code == "CS1620":
                kw = "out" if "'out'" in msg else "ref"
                if not text[c:].startswith(kw + " "):
                    lines[i] = text[:c] + kw + " " + text[c:]
                    changed += 1
            elif code in ("CS1615", "CS8329"):
                # argument may not be passed with ref
                m = re.match(r"(ref|out|in) ", text[c:])
                if m:
                    lines[i] = text[:c] + text[c + len(m.group(0)):]
                    changed += 1
                else:
                    j = text.rfind("ref ", 0, c + 1)
                    if 0 <= j and c - j < 6:
                        lines[i] = text[:j] + text[j + 4:]
                        changed += 1
            elif code == "CS0165":
                # use of unassigned local: initialize the declaration
                nm = re.search(r"'(\w+)'", msg)
                if nm:
                    name = nm.group(1)
                    for k in range(i, max(-1, i - 300), -1):
                        dm = re.match(r"^(\s*)([\w<>]+) ((?:\w+(?: = [^,;]+)?, )*)%s((?:, \w+)*);\s*$" % name, lines[k])
                        if dm and dm.group(2) not in ("return", "else"):
                            lines[k] = lines[k].replace(" %s" % name, " %s = default" % name, 1) if (
                                " %s," % name in lines[k] or " %s;" % name in lines[k]) else lines[k]
                            changed += 1
                            break
            elif code == "CS0136":
                # C lets an inner scope reuse a name declared later in the outer scope: rename the
                # inner variable until the end of its block
                nm = re.search(r"'(\w+)'", msg)
                if nm:
                    name = nm.group(1)
                    depth, end = 0, None
                    for k in range(i, len(lines)):
                        segment = lines[k][c:] if k == i else lines[k]
                        for ch in segment:
                            if ch == "{":
                                depth += 1
                            elif ch == "}":
                                depth -= 1
                                if depth < 0:
                                    end = k
                                    break
                        if end is not None:
                            break
                    if end is not None:
                        for k in range(i, end + 1):
                            # object initializer member names keep their name
                            lines[k] = re.sub(r"(?<![\w.])%s\b(?! = )|(?<!\{ )(?<!, )(?<![\w.])%s\b(?= = )" % (name, name),
                                              name + "Inner", lines[k])
                        changed += 1
            elif code == "CS1510" and text[c:].startswith("ref "):
                # ref to a non-variable, drop the ref
                lines[i] = text[:c] + text[c + 4:]
                changed += 1
        open(path, "w", encoding="utf-8", newline="\n").write("\n".join(lines))
    return changed


def main():
    global PROJECT
    if "--project" in sys.argv:
        PROJECT = os.path.join(REPO, sys.argv[sys.argv.index("--project") + 1])
    if "--cleanup" in sys.argv:
        for root, _, files in os.walk(LIB):
            for f in files:
                if f.endswith(".cs"):
                    cleanup(os.path.join(root, f))
    for iteration in range(20):
        errors = build()
        changed = apply(errors)
        print("iteration %d: %d errors, %d fixes" % (iteration, len(errors), changed))
        if changed == 0:
            break


if __name__ == "__main__":
    main()
