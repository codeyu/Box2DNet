#!/usr/bin/env python3
"""Hand fixes for the converted samples, applied once after conversion."""

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SAMPLES = os.path.abspath(os.path.join(HERE, "..", "..", "..", "examples", "Testbed", "Samples"))


def patch(name, pairs, regex=()):
    p = os.path.join(SAMPLES, name)
    s = open(p, encoding="utf-8").read()
    for a, b in pairs:
        if a not in s:
            print("%s missing: %s" % (name, a[:100]))
        s = s.replace(a, b)
    for a, b in regex:
        s, n = re.subn(a, b, s, flags=re.M)
        if n == 0:
            print("%s no match: %s" % (name, a[:100]))
    open(p, "w", encoding="utf-8", newline="\n").write(s)


# The rollback sample needs world snapshots (snapshot.c), which are not ported
p = os.path.join(SAMPLES, "SampleDeterminism.cs")
s = open(p, encoding="utf-8").read()
a = s.index("public class Rollback : Sample")
s = s[:a].rstrip() + "\n"
s = s.replace("\n\t\t\t\tFinishRecording();\n", "\n")
open(p, "w", encoding="utf-8", newline="\n").write(s)
patch("SampleList.cs", [('\t\tRegisterSample( "Determinism", "Rollback", Rollback.Create );\n', '')])

patch("SampleIssues.cs", [], [(r"ref \( polygon = (MakeBox\( \d+, \d+ \)) \)", r"\1")])

patch("SampleJoints.cs", [
    ("{ -shadeMouthRadius, mouth } };", "new( -shadeMouthRadius, mouth ) };"),
    ("shapeDef.material.customColor = colors[colorIndex];", "shapeDef.material.customColor = (uint)colors[colorIndex];"),
])
patch("SampleWorld.cs", [
    ("shapeDef.material.customColor = colors[colorIndex];", "shapeDef.material.customColor = (uint)colors[colorIndex];"),
    ("float gridStep = nextafterf( fx, float.MaxValue ) - fx;", "float gridStep = MathF.BitIncrement( fx ) - fx;"),
])

patch("SampleRobustness.cs", [
    ("m_bodyIds = (BodyId*)realloc( m_bodyIds, m_bodyCount * Unsafe.SizeOf<BodyId>() );", "Array.Resize( ref m_bodyIds, m_bodyCount );"),
])

patch("SampleStacking.cs", [
    ('''	public struct Event
	{
		public int indexA, indexB;
	}

	public CircleStack(''', '''	public struct Event
	{
		public int indexA, indexB;

		public Event( int indexA, int indexB )
		{
			this.indexA = indexA;
			this.indexB = indexB;
		}
	}

	public CircleStack('''),
    ("{ 2.405937953536585f, 39.09554102558315f } };", "new( 2.405937953536585f, 39.09554102558315f ) };"),
    ("{ 3.016931552701656f, 44.28891593799322f } };", "new( 3.016931552701656f, 44.28891593799322f ) };"),
    ("{ -ps2[i + 1].x, ps2[i + 1].y } };", "new( -ps2[i + 1].x, ps2[i + 1].y ) };"),
    ("while ( Nb )", "while ( Nb != 0 )"),
])

p = os.path.join(SAMPLES, "SampleShapes.cs")
s = open(p, encoding="utf-8").read()
a = s.index("public class ModifyGeometry : Sample")
b = s.index("public class ChainLink : Sample")
s = s[:a] + s[a:b].replace("public int m_shapeType;", "public ShapeType m_shapeType;") + s[b:]
open(p, "w", encoding="utf-8", newline="\n").write(s)

patch("SampleRestitution.cs", [], [(r"ComputeEnergy\( m_worldId, (m_\w+(?:\[i\])?), 1,", r"ComputeEnergy( m_worldId, new ReadOnlySpan<BodyId>( in \1 ), 1,")])
print("ok")
