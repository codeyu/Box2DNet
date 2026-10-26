# Manual fixups applied after converting contact.c
import re
import sys

p = sys.argv[1]
s = open(p, encoding='utf-8').read()

# The designated 2D registry array becomes a table built at startup
a = s.index('\tstatic readonly ContactRegister contactRegistry[')
b = s.index('\t};', a) + 3
block = s[a:b]
entries = []
outer = None
for line in block.split('\n'):
    m = re.match(r'^\t\t\[ShapeType\.(\w+)\] =\s*$', line)
    if m:
        outer = m.group(1)
        continue
    m = re.match(r'^\s*\[ShapeType\.(\w+)\] = \{ \.fcn = (\w+), \.primary = (true|false) \},', line)
    if m:
        entries.append((outer, m.group(1), m.group(2), m.group(3)))
table = ['\tprivate static readonly ContactRegister[,] contactRegistry = MakeContactRegistry();', '',
         '\tprivate static ContactRegister[,] MakeContactRegistry()', '\t{',
         '\t\tContactRegister[,] r = new ContactRegister[(int)ShapeType.shapeTypeCount, (int)ShapeType.shapeTypeCount];']
for o, i, fcn, primary in entries:
    table.append('\t\tr[(int)ShapeType.%s, (int)ShapeType.%s] = new ContactRegister { fcn = %s, primary = %s };' % (o, i, fcn, primary))
table += ['\t\treturn r;', '\t}']
s = s[:a] + '\n'.join(table) + s[b:]
s = re.sub(r'contactRegistry\[([^\]]+)\]\[([^\]]+)\]', r'contactRegistry[(int)\1, (int)\2]', s)
open(p, 'w', encoding='utf-8', newline='\n').write(s)
