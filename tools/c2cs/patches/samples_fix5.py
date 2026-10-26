#!/usr/bin/env python3
"""Member arrays declared as T m_x[N] = {} lost their size in the conversion. Restore it from the C++,
matching the declarations of each file in order."""

import glob
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SAMPLES = os.path.abspath(os.path.join(HERE, "..", "..", "..", "examples", "Testbed", "Samples"))
UPSTREAM = "C:/github/box2d/samples"

for path in glob.glob(os.path.join(SAMPLES, "*.cs")):
    s = open(path, encoding="utf-8").read()
    if not re.search(r"public [\w.]+\[\] m_\w+ = \{\s*\};", s):
        continue
    base = os.path.splitext(os.path.basename(path))[0]
    cpp_name = re.sub(r"(?<!^)([A-Z])", r"_\1", base).lower() + ".cpp"
    sources = [os.path.join(UPSTREAM, cpp_name)]
    header = os.path.join(UPSTREAM, cpp_name[:-4] + ".h")
    if os.path.exists(header):
        sources.insert(0, header)
    decls = []
    for src in sources:
        text = open(src, encoding="utf-8").read()
        decls += re.findall(r"^\s*(?:const\s+)?\w+\s*\*?\s*(m_\w+)\s*\[\s*([^\]]+?)\s*\]\s*=\s*\{\s*\}\s*;", text, re.M)
    queue = list(decls)

    def fix(m):
        typ, name = m.group(1), m.group(2)
        for i, (n, size) in enumerate(queue):
            if n == name:
                del queue[i]
                size = re.sub(r"\b(\w+)::(\w+)\b", r"\1.\2", size)
                return "public %s[] %s = new %s[%s];" % (typ, name, typ, size)
        print("%s: no size for %s" % (base, name))
        return m.group(0)

    s = re.sub(r"public ([\w.]+)\[\] (m_\w+) = \{\s*\};", fix, s)
    open(path, "w", encoding="utf-8", newline="\n").write(s)
print("ok")
