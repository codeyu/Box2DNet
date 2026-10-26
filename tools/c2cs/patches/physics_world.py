# Manual fixups applied after converting physics_world.c
import re
import sys

p = sys.argv[1]
s = open(p, encoding='utf-8').read()

# The world table holds world objects. A slot is reset by replacing the object.
s = s.replace('\tstatic World worlds[MaxWorlds];', '\tprivate static readonly World[] worlds = MakeWorlds();\n\n'
              '\tprivate static World[] MakeWorlds()\n\t{\n\t\tWorld[] w = new World[MaxWorlds];\n'
              '\t\tfor ( int i = 0; i < MaxWorlds; ++i )\n\t\t{\n\t\t\tw[i] = new World();\n\t\t}\n\n\t\treturn w;\n\t}')
s = re.sub(r'\bworlds \+ \(?\s*([^;]+?)\s*\)?;', lambda m: 'worlds[%s];' % m.group(1), s)
s = s.replace('\t\tWorld world = worlds[worldId];\n\t\tushort generation = world.generation;\n\n\t\tworld = default(World);',
              '\t\tushort generation = worlds[worldId].generation;\n\n\t\tWorld world = worlds[worldId] = new World();')
s = s.replace('\t\tushort generation = world.generation;\n\t\tworld = default(World);',
              '\t\tushort generation = world.generation;\n\t\tworld = worlds[world.worldId] = new World();')
s = s.replace('#if false && true\n', '#if false\n')
open(p, 'w', encoding='utf-8', newline='\n').write(s)

s = open(p, encoding='utf-8').read()
# recording is not ported yet
s = re.sub(r'\tpublic static (\w+) (World_\w*Record\w*)\(', r'\tinternal static \1 \2(', s)
open(p, 'w', encoding='utf-8', newline='\n').write(s)
