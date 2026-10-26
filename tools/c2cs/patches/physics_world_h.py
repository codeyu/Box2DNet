# Manual fixups for physics_world.h: the world is a reference type
import sys

p = sys.argv[1]
s = open(p, encoding='utf-8').read()
s = s.replace('internal struct World\n', 'internal sealed class World\n')
s = s.replace('#pragma warning', '#pragma warning')
s = s.replace('namespace Box2DNet;\n', 'namespace Box2DNet;\n\n#pragma warning disable CS8618 // fields are set by CreateWorld\n', 1)
open(p, 'w', encoding='utf-8', newline='\n').write(s)
s = open(p, encoding='utf-8').read()
s = s.replace('public Recording recording;', 'public object? recording;')
open(p, 'w', encoding='utf-8', newline='\n').write(s)
