#!/usr/bin/env python3
"""Mechanical first-pass converter for the Box2D samples (C++) to the Box2DNet testbed (C#).

The samples are C with classes, so this handles the class layer (members, constructors, overrides,
registration) and hands the function bodies to the C converter in c2cs.py. ImGui calls keep their
shape through Hexa.NET.ImGui, printf style text goes through Printf.Fmt. Every converted file must be
reviewed by hand and the output is not regenerated once it has been fixed.

usage: cpp2cs.py <box2d root> <out dir> <sample.cpp | helper.cpp> ...
"""

import os
import re
import sys

import c2cs
from c2cs import convert_type, find_call, rewrite_macro_calls, split_args

# C++ names that are C# keywords. 'this', 'override' and friends mean the same thing in both.
CS_KEYWORDS = ["params", "base", "lock", "event", "fixed", "checked", "string", "object", "out", "ref", "internal",
               "readonly", "foreach", "implicit", "decimal", "in", "is", "as", "lock", "unchecked", "sbyte", "ushort",
               "ulong", "uint", "finally"]

IMGUI_ENUMS = ["ImGuiWindowFlags", "ImGuiCol", "ImGuiCond", "ImGuiTreeNodeFlags", "ImGuiTabItemFlags", "ImGuiSliderFlags",
               "ImGuiTableFlags", "ImGuiTableColumnFlags", "ImGuiSelectableFlags", "ImGuiInputTextFlags", "ImGuiStyleVar",
               "ImGuiKey", "ImGuiMouseButton", "ImGuiChildFlags", "ImGuiComboFlags", "ImGuiTabBarFlags", "ImGuiFocusedFlags",
               "ImGuiHoveredFlags", "ImGuiPopupFlags", "ImGuiDir", "ImGuiButtonFlags", "ImGuiColorEditFlags", "ImDrawFlags"]

# ImGui functions whose first string argument (after the leading args) is a printf format
IMGUI_FORMAT = {"Text": 0, "TextDisabled": 0, "TextWrapped": 0, "BulletText": 0, "SetItemTooltip": 0, "SetTooltip": 0,
                "TextColored": 1, "LabelText": 1}

# Sample classes whose name is also a Box2DNet type get a suffix
LIBRARY_TYPES = set()

REGISTRATIONS = []


def load_library_types(repo):
    for root, _, files in os.walk(os.path.join(repo, "src", "Box2DNet")):
        if "obj" in root or "bin" in root:
            continue
        for f in files:
            if f.endswith(".cs"):
                text = open(os.path.join(root, f), encoding="utf-8").read()
                for m in re.finditer(r"^public (?:readonly )?(?:struct|enum|class|delegate \w+|sealed class) (\w+)", text, re.M):
                    LIBRARY_TYPES.add(m.group(1))


def escape_keywords(src):
    out = []
    for line in src.split("\n"):
        idx = line.find("//")
        code, comment = (line[:idx], line[idx:]) if idx >= 0 else (line, "")
        # leave string literals alone
        parts = re.split(r'("(?:[^"\\]|\\.)*")', code)
        for i in range(0, len(parts), 2):
            for kw in CS_KEYWORDS:
                parts[i] = re.sub(r"(?<![\w@])%s\b(?!\s*\()" % kw, "@" + kw, parts[i])
        out.append("".join(parts) + comment)
    return "\n".join(out)


def block_end(text, open_idx):
    """text[open_idx] is '{' - return the index of the matching '}'."""
    depth = 0
    i = open_idx
    in_str = False
    while i < len(text):
        c = text[i]
        if c == '"' and text[i - 1] != "\\":
            in_str = not in_str
        elif not in_str:
            if text.startswith("//", i):
                i = text.find("\n", i)
                continue
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    return i
        i += 1
    return -1


def cs_type(t):
    t = t.strip()
    t = re.sub(r"\bconst\b", "", t).strip()
    t = re.sub(r"\bstruct\b", "", t).strip()
    m = re.match(r"^std::vector<\s*(.+?)\s*>$", t)
    if m:
        return "List<%s>" % cs_type(m.group(1))
    if t in ("ImVec2",):
        return "Vector2"
    if t in ("ImVec4",):
        return "Vector4"
    if t in ("ImU32",):
        return "uint"
    if t == "SampleContext*":
        return "SampleContext"
    if t in ENUM_TYPES:
        return ENUM_TYPES[t]
    return convert_type(t)


def convert_param(p):
    p = p.strip()
    if p in ("void", ""):
        return None
    p = re.sub(r"\s*=\s*(.+)$", lambda m: " = " + m.group(1).replace("nullptr", "null"), p)
    default = ""
    dm = re.match(r"^(.*?)( = .+)$", p)
    if dm:
        p, default = dm.group(1), dm.group(2)
    m = re.match(r"^(const\s+)?(struct\s+)?([\w:<>]+)\s*([*&]{0,2})\s*(@?\w+)?(\[.*\])?$", p)
    if not m:
        return "/*TODO param*/ " + p
    const, _, typ, ptr, name, arr = m.groups()
    if not name:
        UNNAMED[0] += 1
        name = "unused%d" % UNNAMED[0]
    if typ in ("SampleContext", "Sample", "Camera", "Draw") or typ in REF_CLASSES:
        return "%s %s%s" % (cs_class(typ) if typ in CLASSES else cs_type(typ), name, default)
    if typ in CLASSES and not ptr:
        return "%s %s%s" % (typ, name, default)
    if typ == "void" and ptr:
        return "object? %s%s" % (name, default)
    if typ == "char" and ptr:
        return "string %s%s" % (name, default)
    cs = cs_type(typ)
    if arr:
        return ("ReadOnlySpan<%s> " if const else "Span<%s> ") % cs + name
    if ptr == "&":
        return ("in " if const else "ref ") + cs + " " + name
    if ptr == "*":
        if c2cs.CLASS_TYPES and cs in c2cs.CLASS_TYPES:
            return cs + " " + name
        return ("in " if const else "ref ") + cs + " " + name
    return cs + " " + name + default


UNNAMED = [0]

# class names of the samples and helpers, filled per run
CLASSES = set()


def convert_signature(indent, sig, class_name, base_class):
    """Convert a member or free function signature. Returns None if it is not one."""
    sig = " ".join(sig.split())
    m = re.match(r"^(explicit\s+)?(\w+)\s*\((.*?)\)\s*(?::\s*(\w+)\s*\((.*?)\)(?:\s*,.*)?)?$", sig)
    if m and class_name and m.group(2) == class_name:
        params = [convert_param(p) for p in split_args(m.group(3))]
        params = [p for p in params if p]
        init = ""
        if m.group(4):
            init = " : base( %s )" % m.group(5).strip() if m.group(4) in ("Sample", base_class) else ""
        return "%spublic %s( %s )%s" % (indent, cs_class(class_name), ", ".join(params), init)
    m = re.match(r"^(virtual\s+)?~(\w+)\s*\(\s*\)\s*(override)?$", sig)
    if m:
        return "%sDESTRUCTOR" % indent
    m = re.match(r"^((?:static|virtual|inline|constexpr|explicit)\s+)*(const\s+)?([\w:<>]+(?:\s*<[^>]*>)?)\s*([*&]?)\s*(\w+)\s*\((.*)\)\s*(const)?\s*(override)?\s*(final)?$", sig)
    if not m:
        return None
    mods, _, rtype, rptr, name, params, _, override, _ = m.groups()
    if rtype in ("return", "else", "if", "while", "for", "switch", "case", "new", "delete"):
        return None
    mods = mods or ""
    plist = [convert_param(p) for p in split_args(params)]
    plist = [p for p in plist if p]
    if rtype == "void" and rptr == "*":
        cs_r = "object?"
    elif rtype == "char" and rptr == "*":
        cs_r = "string"
    elif rptr == "*" and (rtype in CLASSES or rtype == "Sample"):
        cs_r = cs_class(rtype)
    elif rptr in ("*", "&"):
        cs_r = "ref " + cs_type(rtype)
    else:
        cs_r = cs_type(rtype)
    kw = []
    kw.append("public")
    if "static" in mods or not class_name:
        kw.append("static")
    if override:
        kw.append("override")
    elif "virtual" in mods:
        kw.append("virtual")
    if "::" in name:
        name = name.split("::")[-1]
    return "%s%s %s %s( %s )" % (indent, " ".join(kw), cs_r, name, ", ".join(plist))


def cs_class(name):
    return name + "Sample" if name in LIBRARY_TYPES else name


def convert_member_field(indent, line):
    """Convert a field declaration inside a class or struct."""
    stripped = re.sub(r"\binline\s+", "", line.strip())
    m = re.match(r"^static\s+constexpr\s+(\w+)\s+(\w+)\s*=\s*(.+);(.*)$", stripped) or re.match(
        r"^static\s+const\s+(\w+)\s+(\w+)\s*=\s*(.+);(.*)$", stripped)
    if m:
        t = cs_type(m.group(1))
        kind = "const" if t in ("int", "float", "bool", "uint", "ulong", "double", "string", "byte", "long") else "static readonly"
        return "%spublic %s %s %s = %s;%s" % (indent, kind, t, m.group(2), m.group(3), m.group(4))
    m = re.match(r"^(static\s+)?(const\s+)?([\w:<>, ]+?)\s*([*&]?)\s*(@?\w+)\s*((?:\[[^\]]+\])*)\s*(?:=\s*(.+?))?\s*;(.*)$", stripped)
    if not m or m.group(3) in ("return", "delete", "using", "friend"):
        return None
    static, const, typ, ptr, name, arr, init, rest = m.groups()
    if "(" in typ:
        return None
    vis = "public static " if static else "public "
    if typ == "void" and ptr:
        cs = "object?"
    elif typ == "char" and ptr:
        cs = "string"
    elif ptr == "*" and typ in REF_CLASSES:
        cs = cs_class(typ) + "?"
    elif ptr == "*" and cs_type(typ) in c2cs.CLASS_TYPES:
        cs = cs_type(typ)
    elif ptr == "*":
        cs = cs_type(typ) + "[]"
    else:
        cs = cs_class(typ) if typ in CLASSES else cs_type(typ)
    if arr:
        dims = re.findall(r"\[([^\]]+)\]", arr)
        if typ == "char":
            return "%s%sstring %s = \"\";%s" % (indent, vis, name, rest)
        if init:
            return "%s%s%s%s %s = %s;%s" % (indent, vis, cs, "[]" * len(dims), name, init, rest)
        if len(dims) == 1:
            if typ in REF_STRUCTS:
                return "%s%s%s[] %s = NewArray<%s>( %s );%s" % (indent, vis, cs, name, cs, dims[0], rest)
            return "%s%s%s[] %s = new %s[%s];%s" % (indent, vis, cs, name, cs, dims[0], rest)
        return "%s%s%s[%s] %s = new %s[%s];%s" % (indent, vis, cs, "," * (len(dims) - 1), name, cs, ", ".join(dims), rest)
    if ( cs.startswith("List<") or ( typ in REF_STRUCTS and not ptr ) ) and not init:
        init = "new()"
    if init:
        return "%s%s%s %s = %s;%s" % (indent, vis, cs, name, init, rest)
    return "%s%s%s %s;%s" % (indent, vis, cs, name, rest)


def convert_enum(indent, name, body, as_ints):
    """Anonymous and unscoped enums become int constants, so members keep working as ints."""
    out = []
    value = 0
    expr = None
    for item in split_args(re.sub(r"//.*", "", body)):
        item = item.strip()
        if not item:
            continue
        m = re.match(r"^(\w+)\s*=\s*(.+)$", item, re.S)
        if m:
            member, expr = m.group(1), m.group(2).strip()
            value = 0
        else:
            member = item
            value += 1 if expr is not None or value or out else 0
        if expr is None:
            v = str(value)
        elif value == 0:
            v = expr
        else:
            v = "%s + %d" % (expr if re.match(r"^[\w.]+$", expr) else "( %s )" % expr, value)
        typ = "ulong" if as_ints == "ulong" or "~" in body else "int"
        out.append("%spublic const %s %s = %s;" % (indent, typ, member, v))
        if expr is None:
            pass
    return "\n".join(out)


def convert_class_body(body, class_name, base_class, indent):
    """Convert the members of a class or struct body (text between the braces)."""
    lines = body.split("\n")
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        lead = line[: len(line) - len(line.lstrip())]
        if stripped in ("public:", "private:", "protected:") or stripped.startswith("friend "):
            i += 1
            continue
        if lead != indent or not stripped or stripped.startswith("//") or stripped.startswith("#"):
            out.append(line)
            i += 1
            continue
        # enums
        em = re.match(r"^enum\s*(class\s+)?(\w+)?\s*(?::\s*(\w+))?\s*$", stripped)
        if em and i + 1 < len(lines) and lines[i + 1].strip() == "{":
            j = i + 1
            while lines[j].strip() not in ("};", "}"):
                j += 1
            body_text = "\n".join(lines[i + 2: j])
            if em.group(2):
                ENUM_TYPES[em.group(2)] = "ulong" if em.group(3) == "uint64_t" else "int"
            out.append(convert_enum(indent, em.group(2), body_text, ENUM_TYPES.get(em.group(2), "int")))
            i = j + 1
            continue
        # nested struct
        sm = re.match(r"^(struct|class)\s+(\w+)\s*$", stripped)
        if sm and i + 1 < len(lines) and lines[i + 1].strip() == "{":
            j = i + 1
            depth = 0
            while True:
                depth += lines[j].count("{") - lines[j].count("}")
                if depth == 0:
                    break
                j += 1
            inner = "\n".join(lines[i + 2: j])
            is_struct = not (sm.group(1) == "class" or sm.group(2) in REF_STRUCTS)
            out.append("%spublic %s %s" % (indent, "struct" if is_struct else "class", sm.group(2)))
            out.append(indent + "{")
            members = convert_class_body(inner, sm.group(2), None, indent + "\t")
            if is_struct and re.search(r"^\s*public (?!const|static)[\w<>\[\]?, ]+ \w+ = ", members, re.M):
                out.append("%s\tpublic %s()\n%s\t{\n%s\t}\n" % (indent, sm.group(2), indent, indent))
            out.append(members)
            out.append(indent + "}")
            i = j + 1
            continue
        # methods: signature lines followed by a brace at the same indent
        sig = stripped
        j = i
        while sig.count("(") > sig.count(")") and j + 1 < len(lines):
            j += 1
            sig += " " + lines[j].strip()
        # constructor initializer on the next line
        if j + 1 < len(lines) and lines[j + 1].strip().startswith(":"):
            j += 1
            sig += " " + lines[j].strip()
            while j + 1 < len(lines) and lines[j + 1].strip() != "{":
                j += 1
                sig += " " + lines[j].strip()
        if j + 1 < len(lines) and lines[j + 1].rstrip() == indent + "{" and "(" in sig and not sig.endswith(";"):
            conv = convert_signature(indent, sig, class_name, base_class)
            if conv:
                if conv.strip() == "DESTRUCTOR":
                    # the body runs before the base cleanup, like a C++ destructor
                    k = j + 1
                    depth = 0
                    while True:
                        depth += lines[k].count("{") - lines[k].count("}")
                        if depth == 0:
                            break
                        k += 1
                    out.append(indent + ("public override void Dispose()" if base_class else "public void Dispose()"))
                    out.extend(lines[j + 1: k])
                    if base_class:
                        out.append(indent + "\tbase.Dispose();")
                    out.append(lines[k])
                    i = k + 1
                    continue
                out.append(conv)
                i = j + 1
                continue
        # one line inline method: void Foo() { ... }
        im = re.match(r"^(.*\))\s*(const)?\s*(override)?\s*\{(.*)\}\s*$", stripped)
        if im:
            conv = convert_signature(indent, im.group(1) + (" override" if im.group(3) else ""), class_name, base_class)
            if conv:
                out.append(conv)
                out.append(indent + "{")
                out.append(indent + "\t" + im.group(4).strip())
                out.append(indent + "}")
                i += 1
                continue
        # declarations of methods defined elsewhere
        if re.match(r"^[\w:<>*& ]+\(.*\)\s*(const)?\s*(override)?\s*;$", sig) and not re.match(r"^(return|delete)\b", sig):
            i = j + 1
            continue
        conv = convert_member_field(indent, line)
        if conv is not None:
            out.append(conv)
            i += 1
            continue
        out.append(line)
        i += 1
    return "\n".join(out)


ENUM_TYPES = {}
REF_STRUCTS = set()

# types declared with class (and the structs in REF_STRUCTS): pointers to these are references
REF_CLASSES = set()


def preprocess(text):
    text = re.sub(r"^#include.*$\n?", "", text, flags=re.M)
    text = re.sub(r"^#pragma once.*$\n?", "", text, flags=re.M)
    text = re.sub(r"^// SPDX.*$\n?", "", text, flags=re.M)
    text = re.sub(r"^#if defined\( _MSC_VER \).*?^#endif\s*$\n?", "", text, flags=re.M | re.S)
    text = re.sub(r"\bnullptr\b", "NULL", text)
    text = re.sub(r"\bstatic_cast<\s*([^>]+?)\s*>\s*\(", r"(\1)(", text)
    text = re.sub(r"\breinterpret_cast<\s*([^>]+?)\s*>\s*\(", r"(\1)(", text)
    text = re.sub(r"\bconstexpr\b", "const", text)
    text = re.sub(r"\bauto\b", "var", text)
    text = re.sub(r"\bSample::(\w+)\s*\(", r"__BASE__.\1(", text)
    text = re.sub(r"\bstd::(sqrt|abs|min|max|floor|ceil|sin|cos|atan2|exp|pow)\s*\(", lambda m: "MathF.%s(" % m.group(1).capitalize(), text)
    text = re.sub(r"\b(sinf|cosf|tanf|atan2f|expf|powf|logf|roundf|fmodf|acosf|asinf|atanf|truncf)\s*\(",
                  lambda m: "MathF.%s(" % {"atan2f": "Atan2", "fmodf": "IEEERemainder"}.get(m.group(1), m.group(1)[:-1].capitalize()), text)
    text = re.sub(r"\bFLT_MAX\b", "float.MaxValue", text)
    text = re.sub(r"\bFLT_EPSILON\b", "float.Epsilon", text)
    text = re.sub(r"\bINT_MAX\b", "int.MaxValue", text)
    text = re.sub(r"\bB2_ARRAY_COUNT\b|\bIM_ARRAYSIZE\b|\bARRAY_COUNT\b|\bstd::size\b", "ARRAY_COUNT", text)
    text = re.sub(r"^#ifdef _DEBUG", "#if DEBUG", text, flags=re.M)
    text = re.sub(r"^#ifdef NDEBUG", "#if !DEBUG", text, flags=re.M)
    text = re.sub(r"^#ifndef NDEBUG", "#if DEBUG", text, flags=re.M)
    text = re.sub(r"^#if 0\b", "#if false", text, flags=re.M)
    text = re.sub(r"^#if 1\b", "#if true", text, flags=re.M)
    text = re.sub(r"^#elif 0\b", "#elif false", text, flags=re.M)
    text = re.sub(r"^#elif 1\b", "#elif true", text, flags=re.M)
    text = re.sub(r"^#ifdef (\w+)", r"#if \1", text, flags=re.M)
    text = re.sub(r"^#ifndef (\w+)", r"#if !\1", text, flags=re.M)
    text = re.sub(r"\bif const\s*\(", "if (", text)
    # adjacent string literals
    text = re.sub(r'"(\s*\n\s*)"', r'" +\1"', text)
    text = re.sub(r'" "(?=[^,;)])', '" + "', text)
    # typed brace literals
    prev = None
    while prev != text:
        prev = text
        text = re.sub(r"\b(b2Vec2|b2Pos|b2Rot|b2Transform|b2AABB|ImVec2)\s*\{([^{}]*)\}", lambda m: "%s( %s )" % (
            {"ImVec2": "new Vector2"}.get(m.group(1), "new " + m.group(1)), m.group(2).strip()), text)
    # pointer arrays of callbacks
    text = re.sub(r"\b(\w+Fcn)\s*\*\s*(\w+)\s*\[\s*\]\s*=", r"\1[] \2 =", text)
    # functional casts
    text = re.sub(r"(?<![\w.])(float|int|double|bool|uint32_t|uint64_t)\(\s", lambda m: "(%s)( " % c2cs.PRIMITIVES.get(m.group(1), m.group(1)), text)
    return text


def convert_imgui(text):
    text = re.sub(r"\bImGui::", "ImGui.", text)
    text = re.sub(r"\bImVec2\s*\(", "new Vector2(", text)
    text = re.sub(r"\bImVec4\s*\(", "new Vector4(", text)
    text = re.sub(r"\bImVec2\b", "Vector2", text)
    text = re.sub(r"\bImVec4\b", "Vector4", text)
    text = re.sub(r"\bImU32\b", "uint", text)
    text = re.sub(r"\bImDrawList\s*\*", "ImDrawListPtr ", text)
    for e in IMGUI_ENUMS:
        text = re.sub(r"\b%s_(\w+)" % e, r"%s.\1" % e, text)
    text = re.sub(r"\bImGuiWindowFlags\.None\b", "ImGuiWindowFlags.None", text)

    def fmt_call(name, lead):
        def fn(args):
            if len(args) > lead and args[lead].startswith('"') and ("%" in args[lead] or len(args) > lead + 1):
                fmt = "Fmt( %s )" % ", ".join(args[lead:])
                return "ImGui.%s( %s )" % (name, ", ".join(args[:lead] + [fmt]))
            return "ImGui.%s( %s )" % (name, ", ".join(args))
        return fn

    for name, lead in IMGUI_FORMAT.items():
        text = rewrite_macro_calls(text, "ImGui." + name, fmt_call(name, lead))
    return text


def convert_strings(text):
    # string arrays
    text = re.sub(r"\bconst\s+char\s*\*\s*(const\s+)?(\w+)\s*\[\s*\w*\s*\]\s*=", r"string[] \2 =", text)
    text = re.sub(r"\bconst\s+char\s*\*\s*(\w+)\s*=", r"string \1 =", text)
    text = re.sub(r"\bchar\s*\*\s*(\w+)\s*=", r"string \1 =", text)
    # char buffers and snprintf
    text = re.sub(r"^(\s*)char\s+(\w+)\s*\[\s*\w+\s*\]\s*(=\s*\{\s*0?\s*\}\s*)?;", r'\1string \2 = "";', text, flags=re.M)
    text = rewrite_macro_calls(text, "snprintf", lambda a: "%s = Fmt( %s )" % (a[0], ", ".join(a[2:])))
    text = rewrite_macro_calls(text, "printf", lambda a: "Console.Write( Fmt( %s ) )" % ", ".join(a))
    return text


def convert_vectors(text):
    text = re.sub(r"\bstd::vector<\s*([\w:]+)\s*>", lambda m: "List<%s>" % cs_type(m.group(1)), text)
    text = re.sub(r"\.push_back\(", ".Add(", text)
    text = re.sub(r"\.emplace_back\(", ".Add(", text)
    text = re.sub(r"\.size\(\)", ".Count", text)
    text = re.sub(r"\.clear\(\)", ".Clear()", text)
    text = re.sub(r"\.empty\(\)", ".Count == 0", text)
    text = re.sub(r"\.back\(\)", "[^1]", text)
    text = re.sub(r"\.front\(\)", "[0]", text)
    text = re.sub(r"\.pop_back\(\)", ".RemoveAt( 0 /*TODO last*/ )", text)
    text = re.sub(r"\.data\(\)", "", text)
    return text


def designated(body):
    items = [i.strip() for i in split_args(body) if i.strip()]
    if items and all(i.startswith(".") for i in items):
        return "new() { %s }" % ", ".join(re.sub(r"^\.(\w+)\s*=\s*", r"\1 = ", i) for i in items)
    return None


def assignments(text):
    """name = { ... }; for a field or local of a struct type with a known layout."""
    types = {}
    for m in re.finditer(r"\b([A-Z]\w*)(?:\[\])?\s+(@?\w+)\s*(?=[=;,)])", text):
        types[m.group(2)] = m.group(1)

    def resolve(path):
        parts = path.split(".")
        typ = types.get(parts[0])
        for part in parts[1:]:
            if typ is None:
                return None
            typ = c2cs.field_type(typ, part)
        return typ

    def fn(m):
        indent, target, body = m.group(1), m.group(2), m.group(3)
        d = designated(body)
        if d:
            return "%s%s = %s;" % (indent, target, d)
        typ = resolve(target)
        if typ and typ in c2cs.STRUCTS:
            conv = c2cs.positional_init(typ, body)
            if conv:
                return "%s%s = %s;" % (indent, target, conv)
        return m.group(0)

    pattern = re.compile(r"^(\s*)([\w.\[\]]+)\s*=\s*\{(.*?)\};", re.M | re.S)
    out, pos = [], 0
    while True:
        m = pattern.search(text, pos)
        if not m:
            out.append(text[pos:])
            break
        # the body must be a balanced brace expression ending at the match
        body = m.group(3)
        if body.count("{") != body.count("}") or "\n\n" in body:
            out.append(text[pos:m.end(2)])
            pos = m.end(2)
            continue
        out.append(text[pos:m.start()])
        out.append(fn(m))
        pos = m.end()
    return "".join(out)


def brace_inits(text):
    """Remaining brace initializers in expression position become target-typed new."""
    def init(m):
        before = text_ref[0][max(0, m.start() - 80):m.start()]
        if re.search(r"\[\]\s+@?\w+\s*=\s*$", before) or re.search(r"\bnew \w+\[[^\]]*\]\s*$", before):
            return m.group(0)
        body = m.group(1)
        d = designated(body)
        if d:
            return d
        return "new( %s )" % body if body else "default"

    text_ref = [text]
    prev = None
    while prev != text:
        prev = text
        text_ref[0] = text
        text = re.sub(r"(?<=[=(,?:]\s)\{\s*([^{};]*?)\s*,?\s*\}(?=\s*[;),:])", init, text)
        text = re.sub(r"(?<=return )\{\s*([^{};]*?)\s*\}(?=\s*;)", lambda m: "new( %s )" % m.group(1), text)
        text = re.sub(r"(?<=[=(,]\s)new\(\s*\{\s*([^{};]*?)\s*\}", lambda m: "new( new( %s )" % m.group(1), text)
        text = re.sub(r",\s*\{\s*([^{};]*?)\s*\}(?=\s*[,)])", lambda m: ", new( %s )" % m.group(1), text)
    return text


def local_arrays(text):
    """T name[N]; locals become arrays so they can be passed to the API."""
    def arr(m):
        indent, typ, name, size = m.groups()
        if typ in ("return", "else", "delete", "case"):
            return m.group(0)
        cs = cs_class(typ) if typ in CLASSES else cs_type(typ)
        return "%s%s[] %s = new %s[%s];" % (indent, cs, name, cs, size)
    text = re.sub(r"^(\s+)(\w+)\s+(\w+)\s*\[\s*([^\]]+?)\s*\]\s*;", arr, text, flags=re.M)

    def arr_init(m):
        indent, const, typ, name, size, body = m.groups()
        cs = cs_class(typ) if typ in CLASSES else cs_type(typ)
        if re.match(r"^\{\s*0?\s*\}$", body) and size.strip():
            return "%s%s[] %s = new %s[%s];" % (indent, cs, name, cs, size.strip())
        return "%s%s[] %s = %s;" % (indent, cs, name, body)
    text = re.sub(r"^(\s*)(const\s+|static\s+const\s+|static\s+)?(\w+)\s+(\w+)\s*\[\s*([^\]]*)\]\s*=\s*(\{.*?\})\s*;", arr_init, text, flags=re.M | re.S)
    return text


def convert_file(root, path, helper_header=None):
    src = open(path, encoding="utf-8").read()
    if helper_header:
        src = merge_header(open(helper_header, encoding="utf-8").read(), src)
    src = preprocess(src)
    src = escape_keywords(src)
    src = src.replace("__BASE__.", "base.")

    # registrations
    def reg(m):
        args = split_args(m.group(2))
        cls = args[2].split("::")[0].strip()
        cap = args[3].strip() if len(args) > 3 else None
        REGISTRATIONS.append((args[0].strip(), args[1].strip(), cs_class(cls), cap))
        return ""
    src = re.sub(r"^static int \w+\s*=\s*(RegisterSample\w*)\s*\((.*?)\);\s*$\n?", lambda m: reg(m), src, flags=re.M | re.S)

    for m in re.finditer(r"^(?:class|struct)\s+(\w+)", src, re.M):
        CLASSES.add(m.group(1))

    out = []
    helpers = []
    pos = 0
    pattern = re.compile(r"^(class|struct)\s+(\w+)\s*(?::\s*public\s+(\w+))?\s*$\n^\{", re.M)
    top_enum = re.compile(r"^enum\s+(\w+)?\s*(?::\s*(\w+))?\s*$\n^\{", re.M)
    while True:
        m = pattern.search(src, pos)
        e = top_enum.search(src, pos)
        if e and (not m or e.start() < m.start()):
            helpers.append(src[pos:e.start()])
            close = block_end(src, e.end() - 1)
            if e.group(1):
                ENUM_TYPES[e.group(1)] = "ulong" if e.group(2) == "uint64_t" else "int"
            helpers.append(convert_enum("", e.group(1), src[e.end():close], ENUM_TYPES.get(e.group(1), "int")))
            pos = src.index(";", close) + 1
            continue
        if not m:
            helpers.append(src[pos:])
            break
        helpers.append(src[pos:m.start()])
        close = block_end(src, m.end() - 1)
        body = src[m.end():close]
        kind, name, base = m.group(1), m.group(2), m.group(3)
        is_class = kind == "class" or base is not None or name in REF_STRUCTS
        decl = "public %s %s%s" % ("class" if is_class else "struct", cs_class(name), " : %s" % base if base else "")
        members = convert_class_body(body, name, base, "\t")
        if not is_class and re.search(r"^\s*public (?!const|static)[\w<>\[\]?, ]+ \w+ = ", members, re.M):
            members = "\n\tpublic %s()\n\t{\n\t}\n" % name + members
        out.append(decl + "\n{" + members + "}\n")
        pos = src.index(";", close) + 1

    # free functions and globals go in a static class per file
    free = "\n".join(helpers)
    free = convert_free(free)

    result = "\n".join(out)
    cls = "Helpers_" + re.sub(r"\W", "_", os.path.splitext(os.path.basename(path))[0])
    text = ""
    if free.strip():
        text += "public static partial class %s\n{\n%s\n}\n\n" % (cls, indent_block(free))
    text += result
    text = convert_bodies(text)
    return text, cls if free.strip() else None


def indent_block(text):
    return "\n".join(("\t" + l) if l.strip() else "" for l in text.strip("\n").split("\n"))


def convert_free(text):
    lines = text.split("\n")
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if line and not line[0].isspace() and stripped and not stripped.startswith("//") and not stripped.startswith("#") and not stripped.startswith("public "):
            sig = stripped
            j = i
            while sig.count("(") > sig.count(")") and j + 1 < len(lines):
                j += 1
                sig += " " + lines[j].strip()
            if j + 1 < len(lines) and lines[j + 1].rstrip() == "{" and "(" in sig:
                conv = convert_signature("", sig, None, None)
                if conv:
                    out.append(conv)
                    i = j + 1
                    continue
            if sig.endswith(";") and "(" in sig and "=" not in sig.split("(")[0]:
                # forward declaration
                i = j + 1
                continue
            conv = convert_member_field("", line)
            if conv is not None:
                out.append(conv)
                i += 1
                continue
        out.append(line)
        i += 1
    return "\n".join(out)


def merge_header(header, source):
    """Move the out of line member definitions (Class::Method) of a helper into its class."""
    defs = {}
    pos = 0
    pattern = re.compile(r"^(?:([\w<>*& ]+?)\s+)?(\w+)::(~?\w+)\s*\((.*?)\)\s*(const)?\s*$\n^\{", re.M | re.S)
    rest = []
    while True:
        m = pattern.search(source, pos)
        if not m:
            rest.append(source[pos:])
            break
        rest.append(source[pos:m.start()])
        close = block_end(source, m.end() - 1)
        rtype = (m.group(1) or "").strip()
        sig = "%s%s( %s )" % (rtype + " " if rtype else "", m.group(3), m.group(4).strip())
        body = source[m.end() - 1: close + 1]
        body = "\n".join(("\t" + l) if l else l for l in body.split("\n"))
        defs.setdefault(m.group(2), []).append("\t" + sig + "\n" + body)
        pos = close + 1
    for cls, members in defs.items():
        cm = re.search(r"^(class|struct)\s+%s\b.*?$\n^\{" % cls, header, re.M)
        close = block_end(header, cm.end() - 1)
        header = header[:close] + "\n" + "\n\n".join(members) + "\n" + header[close:]
    return header + "\n" + "".join(rest)


def cpp_statements(text):
    # range for
    def range_for(m):
        typ, ref, name, coll = m.group(2), m.group(3), m.group(4), m.group(5)
        return "foreach ( %s%s %s in %s )" % ("ref " if ref and not m.group(1) else "", "var" if typ == "auto" else typ, name, coll.strip())
    text = re.sub(r"for\s*\(\s*(const\s+)?([\w:<>]+)\s*(&?)\s*(\w+)\s*:\s*([^)]+?)\s*\)", range_for, text)
    # references to locals
    text = re.sub(r"^(\s*)const\s+([\w:<>]+)\s*&\s*(\w+)\s*=\s*", r"\1\2 \3 = ", text, flags=re.M)
    text = re.sub(r"^(\s*)([\w:<>]+)\s*&\s*(\w+)\s*=\s*", r"\1ref \2 \3 = ref ", text, flags=re.M)
    return text


def float_literals(text):
    """C++ double literals used as floats get the f suffix (outside strings and comments)."""
    out = []
    for line in text.split("\n"):
        idx = line.find("//")
        code, comment = (line[:idx], line[idx:]) if idx >= 0 else (line, "")
        parts = re.split(r'("(?:[^"\\]|\\.)*")', code)
        for i in range(0, len(parts), 2):
            parts[i] = re.sub(r"(?<![\w.])(\d+\.\d*(?:e[-+]?\d+)?)(?![\w.])", r"\1f", parts[i])
        out.append("".join(parts) + comment)
    return "\n".join(out)


SAMPLE_BASE_METHODS = ["CreateWorld", "Step", "Keyboard", "MouseDown", "MouseUp", "MouseMove", "DrawControls", "ResetText",
                       "FocusHome", "DrawProfile", "DrawMetrics", "DrawHud", "ResetProfile", "Create", "Destroy"]


def convert_bodies(text):
    """Run the C converter over each member body."""
    text = cpp_statements(text)

    # library calls whose name is hidden by a member of the sample
    methods = set(re.findall(r"public (?:override |virtual |static )*[\w<>\[\]?]+ (\w+)\(", text)) | set(SAMPLE_BASE_METHODS)
    text = re.sub(r"\bb2(\w+)\s*\(", lambda m: "B2.%s(" % m.group(1) if m.group(1) in methods else m.group(0), text)

    # library enum functional casts
    for e in set(v.split(".")[0] for v in TYPED_ENUMS.values()):
        text = re.sub(r"(?<![\w.])b2%s\(" % e, "(%s)(" % e, text)
    text = re.sub(r"(?<![\w.])(?<!new )(?:b2)?(ShapeType|BodyType|JointType)\(\s", r"(\1)( ", text)
    for name in CLASSES:
        if cs_class(name) != name:
            text = re.sub(r"\bnew %s\(" % name, "new %s(" % cs_class(name), text)
    text = re.sub(r"\.materials = &([\w.]+);", r".materials = [\1];", text)

    # malloc and free of member arrays
    text = re.sub(r"\(\s*\w+\s*\*\s*\)\s*\(?\s*(?:malloc|calloc)\(\s*(.+?)\s*\*\s*sizeof\(\s*(\w+)\s*\)\s*\)\s*\)?",
                  lambda m: "new %s[%s]" % (cs_type(m.group(2)), m.group(1)), text)
    text = re.sub(r"(?<![\w.])(?:malloc|calloc)\(\s*(.+?)\s*\*\s*sizeof\(\s*(\w+)\s*\)\s*\)",
                  lambda m: "new %s[%s]" % (cs_type(m.group(2)), m.group(1)), text)
    text = re.sub(r"(?m)^\s*free\(\s*[\w.]+\s*\);\s*\n", "", text)

    # pointer locals to sample structs become ref locals
    def struct_ptr(m):
        indent, const, typ, name, expr = m.groups()
        if typ in REF_CLASSES or typ in ("void", "char", "Sample", "SampleContext", "Camera", "Draw"):
            return m.group(0)
        expr = expr.strip()
        if expr.startswith("&"):
            expr = expr[1:].strip()
        pm = re.match(r"^([\w.\[\]>-]+)\s*\+\s*(.+)$", expr)
        if pm:
            expr = "%s[%s]" % (pm.group(1), pm.group(2))
        if expr in ("NULL", "nullptr"):
            return "%sref %s %s = ref Unsafe.NullRef<%s>();" % (indent, typ, name, typ)
        return "%sref %s%s %s = ref %s;" % (indent, "readonly " if const else "", typ, name, expr)
    text = re.sub(r"(?m)^(\s*)(const\s+)?([A-Z]\w*)\s*\*\s*(@?\w+)\s*=\s*(.+?);\s*$", struct_ptr, text)
    text = re.sub(r"(?m)^(\s*)(const\s+)?(b2\w+)\s*\*\s*(@\w+)\s*=\s*(.+?);\s*$", struct_ptr, text)

    # glfwGetKey used as a condition
    text = re.sub(r"(\bif\s*\(\s*!?\s*)glfwGetKey\(([^()]*)\)\s*\)", lambda m: "%s( glfwGetKey(%s) %s 0 ) )" % (
        m.group(1).replace("!", ""), m.group(2), "==" if "!" in m.group(1) else "!="), text)
    text = re.sub(r"(&&|\|\|)\s*(!?)\s*glfwGetKey\(([^()]*)\)(?!\s*[=!]=)", lambda m: "%s ( glfwGetKey(%s) %s 0 )" % (
        m.group(1), m.group(3), "==" if m.group(2) else "!="), text)
    for name in REF_CLASSES:
        cs = cs_class(name)
        text = re.sub(r"\(\s*(?:const\s+)?%s\s*\*\s*\)" % name, "(%s?)" % cs, text)
        text = re.sub(r"(?<![\w.])(?:const\s+)?%s\s*\*(?=\s*@?\w+\s*[=;,)])" % name, "%s?" % cs, text)
    for name, typ in ENUM_TYPES.items():
        if name:
            text = re.sub(r"(?<![\w.])%s(?=\s+@?\w+\s*[=;,)])" % name, typ, text)
            text = re.sub(r"\(\s*%s\s*\)" % name, "(%s)" % typ, text)
    text = convert_imgui(text)
    text = convert_strings(text)
    text = convert_vectors(text)
    text = local_arrays(text)
    text = rewrite_macro_calls(text, "ARRAY_COUNT", lambda a: "%s.Length" % a[0])
    text = rewrite_macro_calls(text, "IM_COL32", lambda a: "IM_COL32( %s )" % ", ".join(a))
    text = re.sub(r"\bassert\s*\(", "Debug.Assert(", text)
    text = re.sub(r"\b(\w+)::(\w+)\b", r"\1.\2", text)
    # convert per member so pointer locals do not leak between functions
    chunks = re.split(r"(?m)^(?=\t*public )", text)
    text = "".join(c2cs.convert_body(c, TYPED_ENUMS, FLAG_ENUMS) for c in chunks)
    text = assignments(text)
    text = brace_inits(text)
    # array initializer elements on their own lines
    prev = None
    while prev != text:
        prev = text
        text = re.sub(r"(?m)^([ \t]+)\{[ \t]*([^{};\n]+?)[ \t]*\}(?=[ \t]*,|[ \t]*$)", r"\1new( \2 )", text)
        text = re.sub(r"(?<=\{ )\{[ \t]*([^{};\n]+?)[ \t]*\}(?=,)", r"new( \1 )", text)
        text = re.sub(r"(?<=\), )\{[ \t]*([^{};\n]+?)[ \t]*\}(?=[ \t]*[,}])", r"new( \1 )", text)
    text = float_literals(text)
    text = re.sub(r"(?<![\w.])(\d+)\.f\b", r"\1.0f", text)
    text = re.sub(r"\(\s*intptr_t\s*\)\s*", "", text)
    text = re.sub(r"\bintptr_t\(\s*(.+?)\s*\)", r"( \1 )", text)
    arrays = set(re.findall(r"\[\]\s+(m_\w+)\s*=", text))
    text = re.sub(r"(?<=[=(,]\s)(m_\w+)\s\+\s([\w.]+)(?=\s*[;,)])",
                  lambda m: "%s[%s]" % (m.group(1), m.group(2)) if m.group(1) in arrays else m.group(0), text)
    text = re.sub(r"(customColor\s*=\s*)(HexColor\.\w+)", r"\1(uint)\2", text)
    text = re.sub(r"\(\s*void\s*\*\s*\)\s*", "", text)
    text = re.sub(r"\bref this\b", "this", text)
    text = re.sub(r"\bm_context\.camera\.m_", "m_context.camera.", text)
    # draw and camera are classes
    text = re.sub(r"\bref (m_camera|m_context\.camera|m_draw|m_context\.draw|camera|draw)\b", r"\1", text)
    text = re.sub(r"(?<![\w.])Fmt\(", "Fmt(", text)
    return text


TYPED_ENUMS = {}
FLAG_ENUMS = set()


HEADER = """// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using Hexa.NET.ImGui;
using Box2DNet.Shared;
using static Box2DNet.B2;
using static Box2DNet.Shared.SharedLib;
using static Box2DNet.Testbed.DrawFunctions;
using static Box2DNet.Testbed.Glfw;
using static Box2DNet.Testbed.Printf;
using static Box2DNet.Testbed.SampleApp;
{extra}
namespace Box2DNet.Testbed;

"""


def main():
    global TYPED_ENUMS, FLAG_ENUMS
    sys.stdout.reconfigure(encoding="utf-8", newline="\n")
    root, out_dir = sys.argv[1], sys.argv[2]
    files = sys.argv[3:]
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    load_library_types(repo)
    TYPED_ENUMS, FLAG_ENUMS = c2cs.load_enums(root)
    c2cs.load_structs(root)
    REF_STRUCTS.update(["ShapeUserData", "CastContext", "BodyUserData", "QueryContext", "CastResult", "PlaneExtra",
                        "BenchmarkSensor", "DynamicMoverCastResult", "OverlapResult", "MoverShapeUserData"])

    # helper classes used across files
    for h in ("car", "donut", "doohickey", "dynamic_mover", "geometric_mover"):
        text = open(os.path.join(root, "samples", h + ".h"), encoding="utf-8").read()
        for m in re.finditer(r"^(?:class|struct)\s+(\w+)", text, re.M):
            CLASSES.add(m.group(1))

    for f in files + ["car.h", "donut.h", "doohickey.h", "dynamic_mover.h", "geometric_mover.h"]:
        text = open(os.path.join(root, "samples", f), encoding="utf-8").read()
        for m in re.finditer(r"^class\s+(\w+)", text, re.M):
            REF_CLASSES.add(m.group(1))
    REF_CLASSES.update(REF_STRUCTS)

    # field order of the sample structs, for brace initializers
    for f in files + ["car.h", "donut.h", "doohickey.h", "dynamic_mover.h", "geometric_mover.h"]:
        text = open(os.path.join(root, "samples", f), encoding="utf-8").read()
        text = re.sub(r"//.*", "", text)
        for m in re.finditer(r"^\s*struct (\w+)\s*\{(.*?)\n\s*\};", text, re.S | re.M):
            fields = []
            for decl in m.group(2).split(";"):
                decl = re.sub(r"\s*=.*$", "", decl.strip(), flags=re.S)
                decl = re.sub(r"\bconst\b", "", decl).strip()
                dm = re.match(r"^([\w:<>]+)\s*(\**)\s*(\w+)\s*(\[.*\])?$", decl)
                if dm and "(" not in decl:
                    fields.append((dm.group(3), convert_type(dm.group(1)) + ("*" if dm.group(2) else "")))
            if fields and m.group(1) not in c2cs.STRUCTS:
                c2cs.STRUCTS[m.group(1)] = fields

    usings = []
    results = []
    for f in files:
        ENUM_TYPES.clear()
        path = os.path.join(root, "samples", f)
        header = None
        if not f.startswith("sample_"):
            header = os.path.splitext(path)[0] + ".h"
        text, helper = convert_file(root, path, header)
        results.append((f, text))
        if helper:
            usings.append(helper)
    extra = "".join("using static Box2DNet.Testbed.%s;\n" % u for u in usings)
    for f, text in results:
        name = "".join(p.capitalize() for p in os.path.splitext(f)[0].split("_"))
        out = os.path.join(out_dir, name + ".cs")
        open(out, "w", encoding="utf-8", newline="\n").write(HEADER.replace("{extra}", extra) + text)
        print("wrote", out)

    lines = ["// SPDX-FileCopyrightText: 2022 Erin Catto", "// SPDX-License-Identifier: MIT", "",
             "using static Box2DNet.Shared.SharedLib;", "", "namespace Box2DNet.Testbed;", "", "// The sample registrations, in the order of the C++ static initializers",
             "public static partial class SampleApp", "{", "\tprivate static void RegisterSamples()", "\t{"]
    for cat, name, cls, cap in REGISTRATIONS:
        if cap:
            cap = cap.replace("::", ".")
            lines.append("\t\tRegisterSampleWithCapacity( %s, %s, %s.Create, %s );" % (cat, name, cls, cap))
        else:
            lines.append("\t\tRegisterSample( %s, %s, %s.Create );" % (cat, name, cls))
    lines += ["\t}", "}", ""]
    open(os.path.join(out_dir, "SampleList.cs"), "w", encoding="utf-8", newline="\n").write("\n".join(lines))
    print("registered", len(REGISTRATIONS))


if __name__ == "__main__":
    main()
