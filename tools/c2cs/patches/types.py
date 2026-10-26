# Manual fixups applied after converting include/box2d/types.h (the public API types)
import re
import sys

p = sys.argv[1]
s = open(p, encoding='utf-8').read()

# everything in types.h is public
s = s.replace('\tinternal struct ', '\tpublic struct ')
s = s.replace('\tinternal enum ', '\tpublic enum ')
s = s.replace('\tinternal delegate ', '\tpublic delegate ')

# defines
s = s.replace('\t// #define DefaultCategoryBits 1', '\tpublic const ulong DefaultCategoryBits = 1;')
s = s.replace('\t// #define DefaultMaskBits ulong.MaxValue', '\tpublic const ulong DefaultMaskBits = ulong.MaxValue;')

# function declarations implemented elsewhere
s = re.sub(r'^\tB2_API .*?;\n', '', s, flags=re.M | re.S)


# function pointer typedefs become delegates
def param(p_):
    p_ = p_.strip()
    p_ = re.sub(r'\bvoid\s*\*\s*', 'object? ', p_)
    p_ = re.sub(r'\bconst\s+char\s*\*\s*', 'string ', p_)
    p_ = re.sub(r'\bconst\s+Vec2\s*\*\s*(\w+)', r'ReadOnlySpan<Vec2> \1', p_)
    p_ = re.sub(r'\bconst\s+(\w+)\s*\*\s*', r'in \1 ', p_)
    p_ = re.sub(r'\b(\w+Fcn)\s*\*\s*', r'\1 ', p_)
    p_ = re.sub(r'\b(\w+)\s*\*\s*', r'ref \1 ', p_)
    return p_


def delegate(m):
    ret, name, params = m.group(1), m.group(2), m.group(3)
    if ret == 'void*':
        ret = 'object?'
    params = ', '.join(param(x) for x in params.replace('\n', ' ').split(','))
    return '\tpublic delegate %s %s( %s );' % (ret, name, params)


s = re.sub(r'\ttypedef (\w+\*?) (\w+Fcn)\(\s*(.*?)\s*\);', delegate, s, flags=re.S)

# delegate fields
s = re.sub(r'public /\*TODO pointer\*/ (\w+Fcn)\[\] (\w+);', r'public \1? \2;', s)
# array pointers in definitions and events
s = re.sub(r'public /\*TODO pointer\*/ (\w+)\[\] (\w+);', r'public \1[] \2;', s)
s = s.replace('public /*TODO fixed 24*/ int[] colorCounts;', 'public FixedArray24<int> colorCounts;')


# debug draw function pointers become delegates
def draw_fcn(m):
    name, params = m.group(1), m.group(2)
    params = ', '.join(param(x) for x in params.replace('\n', ' ').split(','))
    return '\t\tpublic %sDelegate? %s;\n\n\t\tpublic delegate void %sDelegate( %s );' % (name, name, name, params)


s = re.sub(r'\t\tvoid \( (\w+Fcn) \)\(\s*(.*?)\s*\);', draw_fcn, s, flags=re.S)
s = s.replace('\tpublic struct DebugDraw\n', '\tpublic struct DebugDraw\n')

# the types live at namespace level
s = s.replace('public static partial class B2\n{', 'public static partial class B2\n{\n\tpublic const ulong DefaultCategoryBitsValue = 1;\n}\n\n#pragma warning disable CS1587\n', 1)
s = s.rstrip()
assert s.endswith('}')
s = s[:-1].rstrip() + '\n'
s = s.replace('\tpublic const ulong DefaultCategoryBits = 1;', '')
s = s.replace('\tpublic const ulong DefaultMaskBits = ulong.MaxValue;', '')
s = s.replace('public static partial class B2\n{\n\tpublic const ulong DefaultCategoryBitsValue = 1;\n}',
              'public static partial class B2\n{\n\t/// Default filter category bits.\n\tpublic const ulong DefaultCategoryBits = 1;\n\n'
              '\t/// Default filter mask bits.\n\tpublic const ulong DefaultMaskBits = ulong.MaxValue;\n}')
s = re.sub(r'^\t', '', s, flags=re.M)
open(p, 'w', encoding='utf-8', newline='\n').write(s)

s = open(p, encoding='utf-8').read()
s = s.replace('private const int DefaultCategoryBits = 1;\n', '')
s = s.replace('{\n/// Default filter category bits.\npublic const ulong DefaultCategoryBits = 1;\n\n/// Default filter mask bits.\npublic const ulong DefaultMaskBits = ulong.MaxValue;\n}',
              '{\n\t/// Default filter category bits.\n\tpublic const ulong DefaultCategoryBits = 1;\n\n\t/// Default filter mask bits.\n\tpublic const ulong DefaultMaskBits = ulong.MaxValue;\n}')
open(p, 'w', encoding='utf-8', newline='\n').write(s)
