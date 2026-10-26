# Manual fixups applied after converting manifold.c
import re
import sys

p = sys.argv[1]
s = open(p, encoding='utf-8').read()
s = re.sub(r'#if B2_SIMD_SSE2 \|\| B2_SIMD_AVX2\n.*?#endif\n', '', s, flags=re.S)
s = s.replace('\t// #define B2_MAKE_ID( A, B ) ( (byte)( A ) << 8 | (byte)( B ) )',
              '\t[MethodImpl( MethodImplOptions.AggressiveInlining )]\n'
              '\tprivate static ushort B2_MAKE_ID( int a, int b ) => (ushort)( (byte)a << 8 | (byte)b );')
s = s.replace('internal const int b2_normalSkip = /*TODO*/;', 'b2_normalSkip,')
s = s.replace('internal const int b2_normalAdmit = /*TODO*/;', 'b2_normalAdmit,')
s = s.replace('internal const int b2_normalSnap = /*TODO*/;', 'b2_normalSnap')
s = s.replace('\t// enum NormalType\n', '\tprivate enum NormalType\n\t{\n')
s = s.replace('b2_normalSnap\n\n\tinternal struct', 'b2_normalSnap\n\t}\n\n\tprivate struct')
s = re.sub(r'(?<![\w.])b2_normal(Skip|Admit|Snap)\b(?!,|\n)', lambda m: 'NormalType.b2_normal' + m.group(1), s)
s = s.replace('\tstatic NormalType ClassifyNormal( struct ChainSegmentParams @params, Vec2 normal )',
              '\tprivate static NormalType ClassifyNormal( in ChainSegmentParams @params, Vec2 normal )')
open(p, 'w', encoding='utf-8', newline='\n').write(s)

s = open(p, encoding='utf-8').read()
# pointer increments
s = s.replace('\t\t\t\tmanifold.pointCount += 1;\n\t\t\t\tcp += 1;\n', '\t\t\t\tmanifold.pointCount += 1;\n\t\t\t\tcp = ref manifold.points[1];\n')
s = s.replace('\t\t\tref LocalManifoldPoint cp = ref manifold.points[0];', '\t\t\tscoped ref LocalManifoldPoint cp = ref manifold.points[0];')
# pointer arguments passed through
s = s.replace('return CollideChainSegmentAndPolygon( segmentA, ref polyB, xf, cache );', 'return CollideChainSegmentAndPolygon( segmentA, polyB, xf, ref cache );')
s = s.replace('input.proxyA = MakeProxy( ref segmentA.segment.point1, 2, 0.0f );', 'input.proxyA = MakeProxy2( segmentA.segment.point1, segmentA.segment.point2, 0.0f );')
s = s.replace('DistanceOutput output = ComputeShapeDistance( ref input, cache );', 'DistanceOutput output = ComputeShapeDistance( input, ref cache );')
# C allows inner scopes to reuse names declared later in the outer scope
a = s.index('int ib1 = cache.indexB[0];')
b = s.index('// Find incident polygon normal: normal adjacent to deepest vertex')
inner = s[a:b]
for old, new in (('ib1', 'jb1'), ('ib2', 'jb2'), ('b1', 'vb1'), ('b2', 'vb2')):
    inner = re.sub(r'\b%s\b' % old, new, inner)
s = s[:a] + inner + s[b:]
open(p, 'w', encoding='utf-8', newline='\n').write(s)
