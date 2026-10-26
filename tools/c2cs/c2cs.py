#!/usr/bin/env python3
"""Mechanical first-pass C -> C# converter for porting Box2D to Box2DNet.

This is not a compiler. It performs the repetitive rewrites (names, pointers to refs, array
macros, compound literals) so that the remaining work is reviewing the output and fixing what
does not compile. Every converted file must be reviewed by hand.

usage: c2cs.py <box2d root> <file.c> [more files] > out.cs
"""

import json
import os
import re
import sys

# Types that are reference types (classes) in Box2DNet. Pointers to these become plain references.
TEST_MODE = False

CLASS_TYPES = {"World", "DynamicTree", "ArenaAllocator", "Scheduler", "Recording", "BroadPhase", "ConstraintGraph"}

# C type names that are spelled differently in Box2DNet
TYPE_RENAMES = {"HashSet": "HashSet64", "Stack": "ArenaAllocator"}

PRIMITIVES = {
    "uint64_t": "ulong",
    "int64_t": "long",
    "uint32_t": "uint",
    "int32_t": "int",
    "uint16_t": "ushort",
    "int16_t": "short",
    "uint8_t": "byte",
    "int8_t": "sbyte",
    "size_t": "int",
    "uintptr_t": "nuint",
    "char": "byte",
    "unsigned": "uint",
}

CS_KEYWORDS = [
    "params", "base", "lock", "event", "fixed", "checked", "operator", "delegate", "string",
    "object", "out", "ref", "internal", "override", "readonly", "foreach", "using", "this",
    "throw", "try", "catch", "finally", "implicit", "explicit", "decimal", "namespace",
]


def escape_keywords(src):
    """Escape C identifiers that are C# keywords. Comments are left alone."""
    out = []
    in_block = False
    for line in src.split("\n"):
        if in_block:
            if "*/" in line:
                in_block = False
            out.append(line)
            continue
        if line.lstrip().startswith("/*") and "*/" not in line:
            in_block = True
            out.append(line)
            continue
        code, comment = line, ""
        idx = line.find("//")
        if idx >= 0:
            code, comment = line[:idx], line[idx:]
        for kw in CS_KEYWORDS:
            code = re.sub(r"(?<![\w@])%s\b(?!\s*\()" % kw, "@" + kw, code)
        out.append(code + comment)
    return "\n".join(out)

MACROS = {
    "B2_NULL_INDEX": "NullIndex",
    "B2_LINEAR_SLOP": "LinearSlop",
    "B2_SPECULATIVE_DISTANCE": "SpeculativeDistance",
    "B2_HUGE": "Huge",
    "B2_PI": "Pi",
    "B2_MAX_WORKERS": "MaxWorkers",
    "B2_MAX_TASKS": "MaxTasks",
    "B2_GRAPH_COLOR_COUNT": "GraphColorCount",
    "B2_MAX_WORLDS": "MaxWorlds",
    "B2_NAME_LENGTH": "NameLength",
    "B2_MAX_ROTATION": "MaxRotation",
    "B2_CONTACT_RECYCLE_DISTANCE": "ContactRecycleDistance",
    "B2_CONTACT_RECYCLE_COS_ANGLE": "ContactRecycleCosAngle",
    "B2_MAX_AABB_MARGIN": "MaxAABBMargin",
    "B2_AABB_MARGIN_FRACTION": "AABBMarginFraction",
    "B2_TIME_TO_SLEEP": "TimeToSleep",
    "B2_MAX_RESTITUTION_ITERATIONS": "MaxRestitutionIterations",
    "B2_MAX_POLYGON_VERTICES": "MaxPolygonVertices",
    "B2_SECRET_COOKIE": "SecretCookie",
    "B2_HASH_INIT": "HashInit",
    "B2_DEFAULT_CATEGORY_BITS": "DefaultCategoryBits",
    "B2_DEFAULT_MASK_BITS": "DefaultMaskBits",
    "B2_OVERFLOW_INDEX": "OverflowIndex",
    "B2_DYNAMIC_COLOR_COUNT": "DynamicColorCount",
    "B2_SIMD_WIDTH": "SimdWidth",
    "B2_ALIGNMENT": "Alignment",
    "UINT64_MAX": "ulong.MaxValue",
    "UINT32_MAX": "uint.MaxValue",
    "UINT16_MAX": "ushort.MaxValue",
    "INT32_MAX": "int.MaxValue",
    "INT_MAX": "int.MaxValue",
    "NULL": "null",
    "sqrtf": "MathF.Sqrt",
    "fabsf": "MathF.Abs",
    "floorf": "MathF.Floor",
    "ceilf": "MathF.Ceiling",
}


def load_enums(root):
    """Return (typed member map, flag member set). Typed: b2_member -> EnumName.member."""
    typed = {}
    flags = set()
    files = []
    for d in ("src", "include/box2d"):
        for f in os.listdir(os.path.join(root, d)):
            if f.endswith((".h", ".c")):
                files.append(os.path.join(root, d, f))
    for path in files:
        text = open(path, encoding="utf-8").read()
        for m in re.finditer(r"typedef enum (b2\w+)\s*\{(.*?)\}\s*\1\s*;", text, re.S):
            name = m.group(1)[2:]
            for mm in re.finditer(r"^\s*(b2_\w+)", m.group(2), re.M):
                typed[mm.group(1)] = name + "." + mm.group(1)[3:]
        for m in re.finditer(r"^enum (b2\w+)\s*\{(.*?)\};", text, re.S | re.M):
            for mm in re.finditer(r"^\s*(b2_\w+)", m.group(2), re.M):
                flags.add(mm.group(1))
    return typed, flags


STRUCTS = {}


def load_structs(root):
    """Map struct name (without b2) -> list of (field, type) in declaration order."""
    for d in ("src", "include/box2d", "test", "shared"):
        folder = os.path.join(root, d)
        if not os.path.isdir(folder):
            continue
        for f in os.listdir(folder):
            if not f.endswith((".h", ".c")):
                continue
            text = open(os.path.join(folder, f), encoding="utf-8").read()
            text = re.sub(r"//.*", "", text)
            text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
            for m in list(re.finditer(r"typedef struct (\w+)\s*\{(.*?)\}\s*\1\s*;", text, re.S)) + list(
                re.finditer(r"^struct (\w+)\s*\{(.*?)\};", text, re.S | re.M)):
                name = m.group(1)[2:] if m.group(1).startswith("b2") else m.group(1)
                fields = []
                body = re.sub(r"union\s*\{(.*?)\}\s*;", lambda mm: mm.group(1).split(";")[0] + ";", m.group(2), flags=re.S)
                for decl in body.split(";"):
                    decl = re.sub(r"\bconst\b|\bstruct\b|B2_ALIGN_AS\(\s*\d+\s*\)", "", decl).strip()
                    if not decl:
                        continue
                    dm = re.match(r"^(\w+)\s*(\**)\s*(.+)$", decl, re.S)
                    if not dm:
                        continue
                    typ, ptr, names = dm.groups()
                    for n in names.split(","):
                        n = n.strip().lstrip("*").strip()
                        n = re.sub(r"\[.*\]", "", n).strip()
                        if re.match(r"^\w+$", n):
                            fields.append((n, convert_type(typ) + ("*" if ptr else "")))
                STRUCTS[name] = fields


def field_type(typ, name):
    for fname, ftype in STRUCTS.get(typ, []):
        if fname == name:
            return ftype
    return None


def positional_init(typ, body, explicit_type=True):
    """Convert a C brace initializer body for struct typ into a C# expression.

    Handles positional items (using the field order from the struct registry) and designated
    items (.field = value). Nested braces recurse with the field type.
    """
    items = [i for i in split_args(body) if i.strip()]
    fields = STRUCTS.get(typ)
    head = "new %s" % typ if explicit_type and typ else "new()"
    if typ in ("Vec2", "Rot") and items and not any(i.strip().startswith(".") for i in items):
        conv = [positional_init(None, i.strip()[1:-1], False) if i.strip().startswith("{") else i.strip() for i in items]
        return "%s( %s )" % ("new %s" % typ if explicit_type else "new", ", ".join(conv))
    parts = []
    for index, item in enumerate(items):
        item = item.strip()
        dm = re.match(r"^\.(@?\w+)\s*=\s*(.*)$", item, re.S)
        if dm:
            fname, value = dm.group(1), dm.group(2).strip()
        elif fields and index < len(fields):
            fname, value = fields[index][0], item
        else:
            return None
        ftype = field_type(typ, fname) if typ else None
        if ftype and ftype.endswith("*") and ftype[:-1] not in CLASS_TYPES and not value.startswith("{"):
            value = "ref " + (value[1:].strip() if value.startswith("&") else value)
        if value.startswith("{") and value.endswith("}"):
            inner = positional_init(ftype, value[1:-1], False)
            if inner is None:
                return None
            value = inner
        parts.append("%s = %s" % (fname, value))
    return "%s { %s }" % (head, ", ".join(parts))


def load_public_functions(root):
    path = os.path.join(root, "api", "box2d_api.json")
    data = json.load(open(path, encoding="utf-8"))
    return {f["name"] for f in data["functions"]}


def split_args(s):
    """Split a C argument list at top-level commas."""
    args, depth, cur = [], 0, ""
    for ch in s:
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        if ch == "," and depth == 0:
            args.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        args.append(cur.strip())
    return args


def find_call(text, start):
    """text[start] is '(' - return index of matching ')'."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return i
    return -1


def rewrite_macro_calls(text, name, fn):
    out, pos = "", 0
    pattern = re.compile(r"\b" + re.escape(name) + r"\s*\(")
    while True:
        m = pattern.search(text, pos)
        if not m:
            out += text[pos:]
            return out
        open_idx = m.end() - 1
        close_idx = find_call(text, open_idx)
        if close_idx < 0:
            out += text[pos:]
            return out
        args = split_args(text[open_idx + 1 : close_idx])
        out += text[pos : m.start()] + fn(args)
        pos = close_idx + 1


def strip_addr(a):
    a = a.strip()
    if a.startswith("&"):
        a = a[1:].strip()
    return a


def convert_type(t):
    t = t.strip()
    t = re.sub(r"\bconst\b", "", t).strip()
    t = re.sub(r"\bstruct\b", "", t).strip()
    if t in PRIMITIVES:
        return PRIMITIVES[t]
    if t.startswith("b2"):
        t = t[2:]
        return TYPE_RENAMES.get(t, t)
    return t


def convert_param(p, public):
    p = p.strip()
    if p in ("void", ""):
        return None
    m = re.match(r"(const\s+)?(struct\s+)?([\w]+)\s*(\*\*?)?\s*(B2_RESTRICT\s+)?(@?\w+)(\[.*\])?$", p)
    if not m:
        return "/*TODO param*/ " + p
    const, _, typ, ptr, _, name, arr = m.groups()
    if typ == "void" and ptr:
        return "object? " + name
    if typ == "char" and ptr:
        return "string " + name
    cs = convert_type(typ)
    if arr:
        return ("ReadOnlySpan<%s> " if const else "Span<%s> ") % cs + name
    if ptr == "*":
        if cs in CLASS_TYPES:
            return cs + " " + name
        return ("in " if const else "ref ") + cs + " " + name
    if ptr == "**":
        return "/*TODO ptrptr*/ " + cs + "[] " + name
    return cs + " " + name


def convert_signature(line, public_fns, is_header):
    m = re.match(
        r"^(static\s+)?(inline\s+)?(B2_INLINE\s+|B2_FORCE_INLINE\s+)?(const\s+)?(struct\s+)?(\w+)\s*(\*?)\s*(%s)\s*\((.*)\)\s*$" % (r"\w+"),
        line,
        re.S,
    )
    if not m:
        return None
    static, inline, b2inline, _, _, rtype, rptr, name, params = m.groups()
    if rtype in ("return", "else", "if", "while", "for", "switch"):
        return None
    if name in public_fns:
        vis = "public"
    elif static and not is_header and not inline and not b2inline:
        vis = "private"
    else:
        vis = "internal"
    cs_rtype = convert_type(rtype)
    if rtype == "void" and rptr:
        cs_rtype = "object?"
    elif rptr:
        if cs_rtype not in CLASS_TYPES:
            cs_rtype = "ref " + cs_rtype
    plist = [convert_param(p, vis == "public") for p in split_args(params)]
    plist = [p for p in plist if p]
    attr = ""
    if b2inline and "FORCE" in b2inline:
        attr = "[MethodImpl( MethodImplOptions.AggressiveInlining )]\n\t"
    sig = "%s%s static %s %s( %s )" % (attr, vis, cs_rtype, name[2:] if name.startswith("b2") else name, ", ".join(plist))
    sig = sig.replace("(  )", "()")
    return sig, cs_rtype


ARRAY_FIELDS = [
    "nodes", "parents", "proxies", "data", "points", "vertices", "normals", "swapNodes", "leafIndices",
    "leafNodes", "leafBoxes", "leafCenters", "binIndices", "bits", "items", "trees", "colors",
    "solverSets", "bodies", "shapes", "contacts", "joints", "islands", "blocks", "stages",
]


def convert_pointer_locals(text):
    ref_locals = set()
    span_locals = set()

    # pointer parameters used as arrays become spans
    lines = text.split("\n", 1)
    if len(lines) == 2 and re.match(r"^\t(public|internal|private) static ", lines[0]):
        sig, body = lines

        def span_param(m):
            mod, typ, name = m.groups()
            if re.search(r"(?<![\w.])%s\s*\[" % re.escape(name), body) or re.search(
                r"(?<![\w.])%s\s*\+\s*\w" % re.escape(name), body
            ):
                span_locals.add(name)
                return "%s<%s> %s" % ("ReadOnlySpan" if mod == "in" else "Span", typ, name)
            return m.group(0)

        sig = re.sub(r"\b(in|ref) (\w+) (\w+)(?=\s*[,)])", span_param, sig)
        rm = re.match(r"^\t\w+ static ref (\w+) ", sig)
        if rm:
            rtype = rm.group(1)
            body = re.sub(r"\breturn NULL;", "return ref Unsafe.NullRef<%s>();" % rtype, body)
            body = re.sub(r"\breturn (?!ref )(.+?);", lambda mm: "return ref %s;" % (mm.group(1)[1:].strip() if mm.group(1).startswith("&") else mm.group(1)), body)
        text = sig + "\n" + body

    def ptr_local(m):
        indent, const, typ, name, expr = m.groups()
        cs = convert_type(typ)
        if typ == "void":
            return "%sobject? %s = %s;" % (indent, name, expr)
        if cs in CLASS_TYPES:
            return "%s%s %s = %s;" % (indent, cs, name, expr)
        expr = (expr or "").strip()
        # pointer used as an array -> span
        if re.search(r"\b%s\s*\[" % re.escape(name), text) or re.search(
            r"(?<![\w.])%s\s*\+\s*\w" % re.escape(name), text
        ):
            span_locals.add(name)
            pm = re.match(r"^((?:[\w.\[\]]|->)+)\s*\+\s*(.+)$", expr)
            if pm and "(" not in pm.group(1):
                expr = "%s.AsSpan( %s )" % (pm.group(1), pm.group(2).strip())
            if expr.startswith("&"):
                expr = "/*TODO span*/ " + expr
            return "%s%s<%s> %s = %s;" % (indent, "ReadOnlySpan" if const else "Span", cs, name, expr or "default")
        ref_locals.add(name)
        if expr in ("NULL", ""):
            return "%sscoped ref %s%s %s = ref Unsafe.NullRef<%s>();" % (indent, "readonly " if const else "", cs, name, cs)
        if expr.startswith("&"):
            expr = expr[1:].strip()
        pm = re.match(r"^((?:[\w.\[\]]|->)+)\s*\+\s*(.+)$", expr)
        if pm and "(" not in pm.group(1):
            expr = "%s[%s]" % (pm.group(1), pm.group(2).strip())
        return "%sref %s%s %s = ref %s;" % (indent, "readonly " if const else "", cs, name, expr)

    text = re.sub(
        r"^(\s*)(const\s+)?(b2\w+|void|int|float|bool|u?int\d+_t)\s*\*\s*(?:B2_RESTRICT\s+)?(\w+)\s*(?:=\s*(.+?))?;\s*$", ptr_local, text, flags=re.M
    )

    # uninitialized struct locals get default so C# definite assignment is satisfied
    text = re.sub(r"^(\s*)(b2[A-Z]\w*)\s+(\w+)\s*;\s*$", r"\1\2 \3 = default;", text, flags=re.M)

    for name in ref_locals:
        text = re.sub(r"\b%s\s*==\s*NULL\b" % name, "Unsafe.IsNullRef( ref %s )" % name, text)
        text = re.sub(r"\b%s\s*!=\s*NULL\b" % name, "Unsafe.IsNullRef( ref %s ) == false" % name, text)

        def reassign(m):
            expr = m.group(2).strip()
            if expr.startswith("&"):
                expr = expr[1:].strip()
            pm = re.match(r"^((?:[\w.\[\]]|->)+)\s*\+\s*(.+)$", expr)
            if pm and "(" not in pm.group(1):
                expr = "%s[%s]" % (pm.group(1), pm.group(2).strip())
            return "%s%s = ref %s;" % (m.group(1), name, expr)

        text = re.sub(r"^(\s*)%s\s*=\s*(?!ref )(.+);" % name, reassign, text, flags=re.M)

    # pointer arithmetic on arrays: base + index -> base[index]
    bases = [re.escape(n) for n in span_locals] + [r"(?:[\w.]|->)+(?:\.|->)(?:%s)" % "|".join(ARRAY_FIELDS)]
    pattern = r"(?<![\w.])(%s)\s*\+\s*([^,;(){}?:]+?)(?=\s*[,;)])" % "|".join(bases)
    text = re.sub(pattern, lambda m: "%s[%s]" % (m.group(1), m.group(2).strip()), text)
    return text


def size_count(expr):
    """Split 'n * sizeof( T )' into (n, T)."""
    m = re.match(r"^\s*(.+?)\s*\*\s*sizeof\(\s*(.+?)\s*\)\s*$", expr)
    if m:
        return m.group(1), convert_type(m.group(2))
    m = re.match(r"^\s*sizeof\(\s*(.+?)\s*\)\s*\*\s*(.+?)\s*$", expr)
    if m:
        return m.group(2), convert_type(m.group(1))
    m = re.match(r"^\s*sizeof\(\s*(.+?)\s*\)\s*$", expr)
    if m:
        return "1", convert_type(m.group(1))
    return None


def convert_memory(text):
    def alloc(args):
        sc = size_count(args[0])
        if sc:
            return "Alloc<%s>( %s )" % (sc[1], sc[0])
        return "Alloc/*TODO*/( %s )" % ", ".join(args)

    def free(args):
        sc = size_count(args[1]) if len(args) > 1 else None
        if sc:
            return "Free( %s, %s )" % (args[0], sc[0])
        return "Free/*TODO*/( %s )" % ", ".join(args)

    def stack_alloc(args):
        sc = size_count(args[1])
        if sc:
            return "%s.Alloc<%s>( %s, %s )" % (strip_addr(args[0]), sc[1], sc[0], args[2])
        return "%s.Alloc/*TODO*/( %s )" % (strip_addr(args[0]), ", ".join(args[1:]))

    text = re.sub(r"\(\s*(?:const\s+)?\w+\s*\*\s*\)\s*(?=b2Alloc|b2AllocZero|b2StackAlloc|b2GrowAlloc)", "", text)
    text = rewrite_macro_calls(text, "b2AllocZero", alloc)
    text = rewrite_macro_calls(text, "b2Alloc", alloc)
    text = rewrite_macro_calls(text, "b2Free", free)
    text = rewrite_macro_calls(text, "b2StackAlloc", stack_alloc)
    text = rewrite_macro_calls(text, "b2StackFree", lambda a: "%s.Free( %s )" % (strip_addr(a[0]), a[1]))
    text = rewrite_macro_calls(text, "B2_GROW_ZERO", lambda a: "GrowAlloc( %s )" % ", ".join(a))
    text = rewrite_macro_calls(text, "B2_GROW", lambda a: "GrowAlloc( %s )" % ", ".join(a))
    text = rewrite_macro_calls(text, "B2_ALLOC_ARRAY", lambda a: "Alloc<%s>( %s )" % (convert_type(a[1]), a[0]))
    text = rewrite_macro_calls(text, "B2_FREE_ARRAY", lambda a: "Free( %s, %s )" % (a[0], a[1]))
    text = rewrite_macro_calls(text, "B2_ALLOC_STRUCT", lambda a: "new %s()" % convert_type(a[0]))
    text = rewrite_macro_calls(text, "B2_FREE_STRUCT", lambda a: "")
    text = rewrite_macro_calls(
        text,
        "memset",
        lambda a: "%s = default" % strip_addr(a[0]) if a[1] == "0" and a[2].startswith("sizeof") else "memset( %s )" % ", ".join(a),
    )
    return text


class _M:
    def __init__(self, g1, g2):
        self.g = (None, g1, g2)

    def group(self, i):
        return self.g[i]


def replace_compound_literals(text, fn):
    """Replace (b2Type){ ... } with balanced braces."""
    pattern = re.compile(r"(?:\(\s*(b2\w+)\s*\)|B2_LITERAL\(\s*(b2\w+)\s*\))\s*\{")
    out, pos = "", 0
    while True:
        m = pattern.search(text, pos)
        if not m:
            return out + text[pos:]
        depth, i = 0, m.end() - 1
        while i < len(text):
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        body = text[m.end():i]
        out += text[pos:m.start()] + fn(_M(m.group(1) or m.group(2), body))
        pos = i + 1


def convert_body(text, typed_enums, flag_enums):
    text = re.sub(r"(?<!internal )(?<!public )(?<!private )(?<!ref )\b(?:struct|enum) (\w+)\b(?=[ \t]+\*?[ \t]*@?\w|[ \t]*\*[ \t]*@?\w)", r"\1", text)
    text = convert_memory(text)
    # --- array macros
    text = rewrite_macro_calls(text, "b2Array_Get", lambda a: "%s[%s]" % (strip_addr(a[0]), a[1]))
    text = rewrite_macro_calls(text, "b2Array_Push", lambda a: "%s.Push( %s )" % (strip_addr(a[0]), a[1]))
    text = rewrite_macro_calls(text, "b2Array_Emplace", lambda a: "%s.Emplace()" % strip_addr(a[0]))
    text = rewrite_macro_calls(text, "b2Array_Pop", lambda a: "%s.Pop()" % strip_addr(a[0]))
    text = rewrite_macro_calls(text, "b2Array_RemoveSwap", lambda a: "%s.RemoveSwap( %s )" % (strip_addr(a[0]), a[1]))
    text = rewrite_macro_calls(text, "b2Array_Clear", lambda a: "%s.Clear()" % strip_addr(a[0]))
    text = rewrite_macro_calls(text, "b2Array_Destroy", lambda a: "%s.Destroy()" % strip_addr(a[0]))
    text = rewrite_macro_calls(text, "b2Array_Create", lambda a: "%s = default" % strip_addr(a[0]))
    text = rewrite_macro_calls(text, "b2Array_CreateN", lambda a: "%s.CreateN( %s )" % (strip_addr(a[0]), a[1]))
    text = rewrite_macro_calls(text, "b2Array_Reserve", lambda a: "%s.Reserve( %s )" % (strip_addr(a[0]), a[1]))
    text = rewrite_macro_calls(text, "b2Array_Resize", lambda a: "%s.Resize( %s )" % (strip_addr(a[0]), a[1]))
    text = rewrite_macro_calls(
        text, "b2Array_ResizeAndSetZero", lambda a: "%s.ResizeAndSetZero( %s )" % (strip_addr(a[0]), a[1])
    )
    text = rewrite_macro_calls(text, "b2Array_Grow", lambda a: "%s.Grow( %s )" % (strip_addr(a[0]), a[1]))
    text = rewrite_macro_calls(text, "b2Array_ByteCount", lambda a: "%s.ByteCount()" % strip_addr(a[0]))
    text = rewrite_macro_calls(
        text,
        "B2_CHECK_INPUT_RETURN",
        lambda a: "if ( CheckInput( %s ) == false )\n\t\t{\n\t\t\treturn %s;\n\t\t}" % (a[0], a[1]),
    )
    text = rewrite_macro_calls(
        text, "B2_CHECK_INPUT", lambda a: "if ( CheckInput( %s ) == false )\n\t\t{\n\t\t\treturn;\n\t\t}" % a[0]
    )
    text = rewrite_macro_calls(text, "B2_CHECK_DEF", lambda a: "B2_ASSERT( %s.internalValue == SecretCookie )" % a[0])
    text = rewrite_macro_calls(text, "B2_UNUSED", lambda a: "")
    text = rewrite_macro_calls(text, "B2_ARRAY_COUNT", lambda a: "%s.Length" % a[0])
    text = rewrite_macro_calls(text, "ARRAY_COUNT", lambda a: "%s.Length" % a[0])
    text = rewrite_macro_calls(text, "B2_SHAPE_PAIR_KEY", lambda a: "HashSet64.ShapePairKey( %s, %s )" % (a[0], a[1]))
    text = rewrite_macro_calls(text, "B2_IS_NULL", lambda a: "%s.IsNull" % a[0])
    text = rewrite_macro_calls(text, "B2_IS_NON_NULL", lambda a: "%s.IsNonNull" % a[0])
    text = rewrite_macro_calls(text, "B2_ID_EQUALS", lambda a: "( %s ).Equals( %s )" % (a[0], a[1]))
    text = rewrite_macro_calls(text, "B2_SWAP", lambda a: "Swap( ref %s, ref %s )" % (a[0], a[1]))
    text = rewrite_macro_calls(text, "sizeof", lambda a: "Unsafe.SizeOf<%s>()" % convert_type(a[0]))
    text = rewrite_macro_calls(text, "b2TracyCZoneNC", lambda a: "")
    text = rewrite_macro_calls(text, "b2TracyCZoneEnd", lambda a: "")
    text = rewrite_macro_calls(text, "b2TracyCZoneC", lambda a: "")
    text = re.sub(r"^\s*;\s*$\n", "", text, flags=re.M)

    # --- compound literals
    def compound(m):
        typ = convert_type(m.group(1))
        body = m.group(2).strip()
        if body in ("0", ""):
            return "default(%s)" % typ
        conv = positional_init(typ, body)
        return conv if conv else "new %s( %s )" % (typ, body)

    text = replace_compound_literals(text, compound)

    # --- struct initializers with known field order
    def struct_init(m):
        typ = convert_type(m.group(1).replace("const", "").strip())
        body = m.group(3).strip()[1:-1]
        if body.strip() in ("0", ""):
            return "%s%s = default;" % (m.group(1), m.group(2))
        conv = positional_init(typ, body)
        if not conv:
            return m.group(0)
        return "%s%s = %s;" % (m.group(1), m.group(2), conv)

    text = re.sub(r"(\b(?:const\s+)?(?:b2\w+|%s))(\s+@?\w+)\s*=\s*(\{[^;]*\})\s*;" % "|".join(
        sorted((k for k in STRUCTS if not k.startswith("b2") and k[0].isupper()), key=len, reverse=True) or ["b2"]), struct_init, text)

    # arrays with initializers: T name[] = { ... };
    def array_init(m):
        indent, const, typ, name, body = m.groups()
        cs = convert_type(typ)
        body = body.strip()[1:-1]
        prev = None
        while prev != body:
            prev = body
            body = re.sub(r"\{([^{}]*)\}", lambda mm: "new( %s )" % mm.group(1).strip(), body)
        return "%s%s[] %s = { %s };" % (indent, cs, name, body.strip())

    def static_array(m):
        typ, name, body = m.groups()
        cs = convert_type(typ)
        body = body.strip()[1:-1]
        prev = None
        while prev != body:
            prev = body
            body = re.sub(r"\{([^{}]*)\}", lambda mm: "new( %s )" % mm.group(1).strip(), body)
        return "private static readonly %s[] %s = { %s };" % (cs, name, body.strip())

    text = re.sub(
        r"^static\s+(?:const\s+)?(b2\w+|float|int|bool|u?int\d+_t)\s+(\w+)\s*\[[^\]]*\]\s*=\s*(\{.*?\})\s*;",
        static_array,
        text,
        flags=re.M | re.S,
    )

    text = re.sub(
        r"^(\s*)(const\s+)?(b2\w+|float|int|bool|u?int\d+_t)\s+(\w+)\s*\[[^\]]*\]\s*=\s*(\{.*?\})\s*;",
        array_init,
        text,
        flags=re.M | re.S,
    )

    # --- declarations initialized with { 0 }
    text = re.sub(r"\b(b2\w+)(\s+\w+)\s*=\s*\{\s*0\s*\}\s*;", r"\1\2 = default;", text)
    text = re.sub(r"\b(b2\w+)(\s+\w+)\s*=\s*B2_ZERO_INIT\s*;", r"\1\2 = default;", text)
    text = re.sub(r"\b(b2\w+)(\s+\w+)\s*=\s*\{([^{};]*)\}\s*;", r"\1\2 = new( \3 );", text)

    # --- out parameters
    text = re.sub(r"b2GetLengthAndNormalize\(\s*&", "b2GetLengthAndNormalize( out ", text)
    text = re.sub(r"(b2ComputeShapeDistance\([^;]*?),\s*NULL\s*,\s*0\s*\)", r"\1 )", text)

    # --- pointer locals, processed per function so names do not leak between functions
    chunks = re.split(r"(?m)^(?=	(?:public|internal|private) static )", text)
    text = "".join(convert_pointer_locals(c) for c in chunks)

    # local const with a non-literal value is not a C# constant
    text = re.sub(r"(?m)^(\s+)const (\w+ \w+ = (?![-\d.]+f?;))", r"\1\2", text)

    # --- bit tests used as conditions
    def bit_test(m):
        inner = m.group(2)
        if "&&" in inner or "||" in inner or re.search(r"[=<>!]", inner.replace("->", ".")):
            return m.group(0)
        return "%s( ( %s ) != 0 )" % (m.group(1), inner.strip())

    text = re.sub(r"(\b(?:if|while)\s*)\(\s*([^()]*?\w\s&\s[^()]*?)\s*\)(?=\s*$)", bit_test, text, flags=re.M)

    text = re.sub(r"\(\s*void\s*\)\s*", "", text)
    text = re.sub(r"\bref (\w+(?:\.|->)(?:broadPhase|constraintGraph|stack))\b", r"\1", text)
    text = re.sub(r"&(\w+(?:\.|->)(?:broadPhase|constraintGraph|stack))\b", r"\1", text)

    # --- operators
    text = text.replace("->", ".")
    # address-of in argument position
    text = re.sub(r"([(,]\s*)&\s*(?=[\w(])", r"\1ref ", text)
    # unary dereference
    text = re.sub(r"(^|[\s(,=!])\*(?=[A-Za-z_])", r"\1", text, flags=re.M)

    # --- casts of primitive types
    for c, cs in PRIMITIVES.items():
        text = re.sub(r"\(\s*%s\s*\)" % c, "(%s)" % cs, text)
        text = re.sub(r"\b%s\b(?=\s+\w+\s*[=;,)\[])" % c, cs, text)

    # --- enums
    def enum_name(m):
        n = m.group(0)
        if n in typed_enums:
            return typed_enums[n]
        if n in flag_enums:
            return n
        return n[3:]

    text = re.sub(r"\bb2_\w+", enum_name, text)

    # --- macros
    for k, v in MACROS.items():
        text = re.sub(r"\b%s\b" % k, v, text)

    # --- generic b2 prefix
    text = re.sub(r"\bb2([A-Z]\w*)", r"\1", text)

    # --- misc
    text = re.sub(r"\bstatic const (b2\w+|[A-Z]\w+)\b", r"static readonly \1", text)
    text = re.sub(r"\bstatic const\b", "const", text)
    text = re.sub(r"(\d+\.\d*)f\b", r"\1f", text)
    text = text.replace("#ifndef NDEBUG", "#if DEBUG")
    text = text.replace("#if B2_ENABLE_VALIDATION", "#if BOX2D_VALIDATE")
    return text


FIXED_SIZES = {"2": "2", "3": "3", "4": "4", "8": "8", "B2_MAX_POLYGON_VERTICES": "8", "b2_bodyTypeCount": "3",
               "BodyType.bodyTypeCount": "3", "B2_GRAPH_COLOR_COUNT": "24", "GraphColorCount": "24", "24": "24",
               "B2_MAX_WORKERS": "32", "MaxWorkers": "32"}


def convert_field(line, in_struct):
    mm = re.match(r"^(\s*)((?:const\s+)?[\w<>]+)\s+(@?\w+(?:\s*,\s*@?\w+)+)\s*;(.*)$", line)
    if mm and in_struct:
        indent, typ, names, rest = mm.groups()
        return "\n".join(convert_field("%s%s %s;%s" % (indent, typ, n.strip(), rest if i == 0 else ""), True)
                         for i, n in enumerate(names.split(",")))
    m = re.match(r"^(\s*)(const\s+)?(struct\s+)?([\w<>]+)\s*(\*?)\s*(@?\w+)\s*(\[(.+?)\])?\s*;(.*)$", line)
    if not m:
        return line
    indent, _, _, typ, ptr, name, arr, size, rest = m.groups()
    if typ in ("return", "goto", "break", "continue"):
        return line
    cs = convert_type(typ)
    vis = "public " if in_struct else ""
    if ptr:
        if typ == "void":
            return "%s%sobject? %s;%s" % (indent, vis, name, rest)
        if typ == "char":
            return "%s%sstring? %s;%s" % (indent, vis, name, rest)
        if cs in CLASS_TYPES:
            return "%s%s%s %s;%s" % (indent, vis, cs, name, rest)
        if re.search(r"(Fcn|Callback)$", cs):
            return "%s%s%s? %s;%s" % (indent, vis, cs, name, rest)
        return "%s%s/*TODO pointer*/ %s[] %s;%s" % (indent, vis, cs, name, rest)
    if arr:
        if in_struct and name.startswith("padding"):
            return "%s// %s %s[%s] (cache line padding, not needed in C#)" % (indent, cs, name, size)
        if in_struct and typ == "char":
            return "%s%sstring? %s;%s" % (indent, vis, name, rest)
        if size in FIXED_SIZES:
            return "%s%sFixedArray%s<%s> %s%s;%s" % (indent, vis, FIXED_SIZES[size], cs, name, "" if in_struct else " = default", rest)
        if in_struct:
            return "%s%s/*TODO fixed %s*/ %s[] %s;%s" % (indent, vis, size, cs, name, rest)
        return "%sSpan<%s> %s = stackalloc %s[%s];%s" % (indent, cs, name, cs, size, rest)
    if in_struct:
        return "%s%s%s %s;%s" % (indent, vis, cs, name, rest)
    return line


def delegate_param(p_):
    p_ = p_.strip()
    if p_ in ("void", ""):
        return None
    return convert_param(p_, False)


def convert_types(src):
    """Convert struct and enum definitions."""

    def fn_typedef(m):
        ret, rptr, name, params = m.group(1), m.group(2), m.group(3), m.group(4)
        cs_ret = "object?" if ret == "void" and rptr else convert_type(ret)
        plist = [delegate_param(x) for x in split_args(params.replace("\n", " "))]
        plist = [x for x in plist if x]
        return "internal delegate %s %s( %s );" % (cs_ret, convert_type(name), ", ".join(plist))

    src = re.sub(r"^typedef (?:const\s+)?(\w+)\s*(\*?)\s*(b2\w+)\(\s*(.*?)\s*\);", fn_typedef, src, flags=re.M | re.S)

    def typed_enum(m):
        name = m.group(1)[2:]
        body = re.sub(r"\bb2_(\w+)", r"\1", m.group(2))
        return "internal enum %s\n{%s}" % (name, body)

    src = re.sub(r"typedef enum (b2\w+)\s*\{(.*?)\}\s*\1\s*;", typed_enum, src, flags=re.S)

    def flag_enum(m):
        body = m.group(2)
        out = []
        for line in body.split("\n"):
            mm = re.match(r"^(\s*)(b2_\w+)\s*=\s*(.+?),?\s*$", line)
            if mm:
                out.append("%sinternal const int %s = %s;" % (mm.group(1), mm.group(2), mm.group(3)))
            else:
                mm = re.match(r"^(\s*)(b2_\w+),?\s*$", line)
                out.append(line if not mm else "%sinternal const int %s = /*TODO*/;" % (mm.group(1), mm.group(2)))
        return "// enum %s\n%s" % (m.group(1), "\n".join(out).strip("\n"))

    src = re.sub(r"^enum (b2\w+)\s*\{(.*?)\};", flag_enum, src, flags=re.S | re.M)

    def struct(m, name=None, body=None):
        name = name or m.group(1)[2:]
        body = body if body is not None else m.group(2)
        unions = []

        def union(um):
            index = len(unions)
            uname = "%sUnion%s" % (name, index if index else "")
            members = []
            for line in um.group(1).split("\n"):
                fm = re.match(r"^\s*(const\s+)?(\w+)\s+(@?\w+)\s*;", line)
                if fm:
                    members.append((convert_type(fm.group(2)), fm.group(3)))
            unions.append((uname, members))
            field = "u%s" % (index if index else "")
            props = ["\t%s %s;" % (uname, field)]
            for t, n in members:
                props.append("\t[UnscopedRef] public ref %s %s => ref %s.%s;" % (t, n, field, n))
            return "\n".join(props)

        body = re.sub(r"^\s*union\s*\{(.*?)\};", union, body, flags=re.S | re.M)
        body = "\n".join(convert_field(l, True) for l in body.split("\n"))
        kw = "sealed class" if name in CLASS_TYPES else "struct"
        out = "internal %s %s\n{%s\n}" % (kw, name, body)
        for uname, members in unions:
            out += "\n\n[StructLayout( LayoutKind.Explicit )]\ninternal struct %s\n{\n" % uname
            out += "\n".join("\t[FieldOffset( 0 )] public %s %s;" % (t, n) for t, n in members)
            out += "\n}"
        return out

    src = re.sub(r"\bb2Array\(\s*(\w+)\s*\)", lambda m: "B2Array<%s>" % convert_type(m.group(1)), src)
    src = re.sub(r"^b2DeclareArray\w*\(.*?\);\s*$\n?", "", src, flags=re.M)
    src = re.sub(r"typedef struct (b2\w+)\s*\{(.*?)\n\}\s*\1\s*;", struct, src, flags=re.S)
    src = re.sub(r"typedef struct ([A-Za-z]\w*)\s*\{(.*?)\n\}\s*\1\s*;", lambda m: struct(m, m.group(1)[2:] if m.group(1).startswith("b2") else m.group(1), m.group(2)), src, flags=re.S)
    src = re.sub(r"typedef struct (\w+)\s*\{(.*?)\n\}\s*(\w+)\s*;", lambda m: struct(m, m.group(3)[2:] if m.group(3).startswith("b2") else m.group(3), m.group(2)), src, flags=re.S)
    src = re.sub(r"typedef struct\s*\{(.*?)\n\}\s*(b2\w+)\s*;", lambda m: struct(m, m.group(2)[2:], m.group(1)), src, flags=re.S)
    src = re.sub(r"^struct (\w+)\s*\{(.*?)\n\};", lambda m: struct(m, m.group(1)[2:] if m.group(1).startswith("b2") else m.group(1), m.group(2)), src, flags=re.S | re.M)
    src = re.sub(r"^typedef struct b2\w+ b2\w+;\s*$\n?", "", src, flags=re.M)

    # local multi-array declarations
    def multi_array(m):
        indent, typ, decls = m.group(1), m.group(2), m.group(3)
        cs = convert_type(typ)
        out = []
        for d in decls.split(","):
            dm = re.match(r"^\s*(\w+)\s*\[(.+?)\]\s*$", d)
            if not dm:
                return m.group(0)
            out.append("%sSpan<%s> %s = stackalloc %s[%s];" % (indent, cs, dm.group(1), cs, dm.group(2)))
        return "\n".join(out)

    src = re.sub(r"^(\s+)(\w+)\s+(\w+\s*\[[^\]]+\](?:\s*,\s*\w+\s*\[[^\]]+\])+)\s*;", multi_array, src, flags=re.M)

    # local arrays
    src = "\n".join(
        convert_field(l, False) if re.match(r"^\s+(const\s+)?\w+\s+\w+\s*\[.+\]\s*;", l) else l for l in src.split("\n")
    )

    # numeric defines become constants and are evaluated in #if comparisons
    src = re.sub(r"^#define (\w+) \(?\s*(-?\d+\.\d*f?)\s*\)?\s*$", r"private const float \1 = \2;", src, flags=re.M)
    defines = dict(re.findall(r"^#define (\w+) (-?\d+)\s*$", src, flags=re.M))
    src = re.sub(r"^#define (\w+) (-?\d+)\s*$", r"private const int \1 = \2;", src, flags=re.M)
    src = re.sub(
        r"^#if (\w+) == (\d+)\s*$",
        lambda m: "#if %s" % ("true" if defines.get(m.group(1)) == m.group(2) else "false")
        if m.group(1) in defines
        else "#if %s" % m.group(1),
        src,
        flags=re.M,
    )
    src = re.sub(r"^(#(?:el)?if .*)$", lambda m: re.sub(r"!?\s*defined\(\s*_CRT_\w+\s*\)", "true", re.sub(r"\b_MSC_VER\b", "false", m.group(1))), src, flags=re.M)
    src = re.sub(r"^(#(?:el)?if .*)$", lambda m: re.sub(r"defined\(\s*(\w+)\s*\)", r"\1", m.group(1)), src, flags=re.M)
    src = re.sub(r"^#if defined\(\s*(\w+)\s*\)", r"#if \1", src, flags=re.M)
    src = re.sub(r"^#ifdef (\w+)", r"#if \1", src, flags=re.M)
    src = re.sub(r"^#ifndef (\w+)", r"#if !\1", src, flags=re.M)
    src = re.sub(r"^#define", "// #define", src, flags=re.M)
    src = re.sub(r"^#if 0\b", "#if false", src, flags=re.M)
    src = re.sub(r"^#if 1\b", "#if true", src, flags=re.M)
    src = re.sub(r"(#if .*)\|\| defined\(\s*(\w+)\s*\)", r"\1|| \2", src, flags=re.M)
    return src


CALLBACK_KINDS = {
    "int proxyId, ulong userData": ("ITreeQueryCallback", "bool", "int proxyId, ulong userData", "proxyId, userData"),
    "in RayCastInput input, int proxyId, ulong userData": (
        "ITreeRayCastCallback", "float", "in RayCastInput input, int proxyId, ulong userData", "input, proxyId, userData"),
    "in BoxCastInput input, int proxyId, ulong userData": (
        "ITreeBoxCastCallback", "float", "in BoxCastInput input, int proxyId, ulong userData", "input, proxyId, userData"),
}


def convert_tree_callbacks(text):
    """Tree query callbacks with a void* context: the context struct becomes a ref struct that
    implements the callback interface, and pointer fields in it become ref fields."""
    contexts = {}
    out = []
    pos = 0
    pattern = re.compile(r"(\t\w+) static (bool|float) (\w+)\( (.*?), object\? context \)\n\t\{\n")
    while True:
        m = pattern.search(text, pos)
        if not m:
            out.append(text[pos:])
            break
        out.append(text[pos:m.start()])
        kind = CALLBACK_KINDS.get(m.group(4))
        cm = re.compile(r"^\s*(?:struct\s+)?(?:ref\s+)?(\w+)\*?\s+(\w+)\s*=\s*(?:ref\s+)?context;\s*\n", re.M).search(
            text, m.end(), m.end() + 400)
        if not kind or not cm:
            out.append(m.group(0))
            pos = m.end()
            continue
        ctx_type, ctx_name = cm.group(1), cm.group(2)
        contexts.setdefault(ctx_type, []).append((m.group(3), kind))
        out.append("%s static %s %s( %s, ref %s %s )\n\t{\n" % (m.group(1), m.group(2), m.group(3), m.group(4), ctx_type, ctx_name))
        out.append(text[m.end():cm.start()])
        pos = cm.end()
    text = "".join(out)

    for ctx_type, fns in contexts.items():
        name, (iface, kret, kparams, kargs) = fns[0]
        sm = re.search(r"\tinternal struct %s\n\t\{\n(.*?)\n\t\}" % ctx_type, text, re.S)
        if not sm:
            continue
        body = sm.group(1)
        body = re.sub(r"public /\*TODO pointer\*/ (\w+)\[\] (\w+);", r"public ref \1 \2;", body)
        body = re.sub(r"public (World|DynamicTree|ArenaAllocator|BroadPhase|ConstraintGraph) (\w+);", r"public \1 \2;", body)
        method = "\t\tpublic %s Invoke( %s ) => %s( %s, ref this );\n" % (kret, kparams, name, kargs)
        new = "\tinternal ref struct %s : %s\n\t{\n%s\n%s\n\t}" % (ctx_type, iface, method, body)
        text = text[:sm.start()] + new + text[sm.end():]
        for fname, _ in fns:
            text = re.sub(r",\s*%s\s*,\s*ref\s+(\w+)\s*\)" % fname, r", ref \1 )", text)
    return text


def convert_file(root, path, typed_enums, flag_enums, public_fns):
    src = open(path, encoding="utf-8").read()
    is_header = path.endswith(".h")
    src = re.sub(r"^#include.*$\n?", "", src, flags=re.M)
    src = re.sub(r"^#pragma once.*$\n?", "", src, flags=re.M)
    src = re.sub(r"^_Static_assert.*$\n?", "", src, flags=re.M)
    src = re.sub(r"^// SPDX.*$\n?", "", src, flags=re.M)
    src = re.sub(r"^\n+", "", src)
    # C identifiers that are C# keywords
    src = escape_keywords(src)
    src = convert_types(src)

    src = re.sub(
        r"^(?:B2_API\s+)?(?:const\s+)?\w+\s*\*?\s*b2\w+\s*\([^;{}]*?\)\s*;[ \t]*$\n?", "", src, flags=re.M | re.S
    )

    lines = src.split("\n")
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        # gather multi-line signatures
        if re.match(r"^(static\s+|B2_INLINE\s+|B2_FORCE_INLINE\s+|inline\s+)*(const\s+)?\w+\s*\*?\s*%s\s*\(" % (r"(?!return|else|if|while|for|switch)\w+"), line):
            sig = line
            j = i
            while sig.count("(") > sig.count(")") and j + 1 < len(lines):
                j += 1
                sig += " " + lines[j].strip()
            if j + 1 < len(lines) and lines[j + 1].strip() == "{":
                conv = convert_signature(sig.strip(), public_fns, is_header)
                if conv:
                    out.append("\t" + conv[0])
                    i = j + 1
                    continue
        out.append(line)
        i += 1

    body = "\n".join(out)
    body = convert_body(body, typed_enums, flag_enums)

    # indent everything that is not already part of a signature by one tab for the class body
    res = []
    for line in body.split("\n"):
        if line.startswith("\tpublic static") or line.startswith("\tinternal static") or line.startswith("\tprivate static"):
            res.append(line)
        elif line.startswith("#"):
            res.append(line)
        elif line.strip() == "":
            res.append("")
        else:
            res.append("\t" + line)
    return convert_tree_callbacks("\n".join(res))


def main():
    global TEST_MODE
    sys.stdout.reconfigure(encoding="utf-8", newline="\n")
    args = sys.argv[1:]
    test_class = None
    if args[0] == "--test":
        TEST_MODE = True
        test_class = args[1]
        args = args[2:]
    root = args[0]
    typed_enums, flag_enums = load_enums(root)
    public_fns = load_public_functions(root)
    load_structs(root)
    print("// SPDX-FileCopyrightText: 2023 Erin Catto")
    print("// SPDX-License-Identifier: MIT")
    print()
    print("using System;")
    print("using System.Diagnostics.CodeAnalysis;")
    print("using System.Runtime.CompilerServices;")
    print("using System.Runtime.InteropServices;")
    if test_class:
        print("using static Box2DNet.B2;")
        print("using static Box2DNet.Tests.TestMacros;")
    print()
    print("namespace Box2DNet%s;" % (".Tests" if test_class else ""))
    print()
    print("public static partial class %s" % (test_class or "B2"))
    print("{")
    for f in args[1:]:
        print(convert_file(root, os.path.join(root, f), typed_enums, flag_enums, public_fns))
    print("}")


if __name__ == "__main__":
    main()
