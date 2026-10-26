#!/usr/bin/env python3
"""Build the library and print a compact, de-duplicated error list.

usage: errors.py [--summary] [--file Name] [--code CSxxxx] [--limit N]
"""

import collections
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
PROJECT = os.path.join(REPO, "src", "Box2DNet", "Box2DNet.csproj")
ERROR = re.compile(r"^(.*?)([A-Za-z0-9_]+\.cs)\((\d+),(\d+)\): error (CS\d+): (.*?) \[")


def main():
    args = sys.argv[1:]
    sys.stdout.reconfigure(encoding="utf-8")
    proj = PROJECT
    if "--project" in args:
        proj = os.path.join(REPO, args[args.index("--project") + 1])
    r = subprocess.run(["dotnet", "build", proj, "--no-restore", "-v", "q", "-clp:NoSummary"], capture_output=True,
                       text=True, encoding="utf-8", errors="replace")
    errors = []
    seen = set()
    for line in r.stdout.splitlines():
        m = ERROR.match(line.strip())
        if m:
            key = m.groups()[1:5]
            if key in seen:
                continue
            seen.add(key)
            errors.append((m.group(2), int(m.group(3)), int(m.group(4)), m.group(5), m.group(6)))
    if "--file" in args:
        f = args[args.index("--file") + 1]
        errors = [e for e in errors if e[0].startswith(f)]
    if "--code" in args:
        c = args[args.index("--code") + 1]
        errors = [e for e in errors if e[3] == c]
    print("%d errors" % len(errors))
    if "--summary" in args:
        print(collections.Counter(e[3] for e in errors).most_common(15))
        print(collections.Counter(e[0] for e in errors).most_common(30))
        names = collections.Counter()
        for e in errors:
            if e[3] == "CS0103":
                m = re.search(r"'(\w+)'", e[4])
                if m:
                    names[m.group(1)] += 1
        print(names.most_common(40))
        return
    limit = int(args[args.index("--limit") + 1]) if "--limit" in args else 60
    for e in sorted(errors)[:limit]:
        print("%s(%d,%d) %s %s" % e)


if __name__ == "__main__":
    main()
