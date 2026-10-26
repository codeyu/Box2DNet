#!/usr/bin/env python3
"""Hand fixes for the converted samples that the converter does not handle (syntax level)."""

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SAMPLES = os.path.abspath(os.path.join(HERE, "..", "..", "..", "examples", "Testbed", "Samples"))


def patch(name, pairs):
    p = os.path.join(SAMPLES, name)
    s = open(p, encoding="utf-8").read()
    for a, b in pairs:
        if a not in s:
            print("%s missing: %s" % (name, a[:80]))
        s = s.replace(a, b)
    open(p, "w", encoding="utf-8", newline="\n").write(s)


patch("SampleBenchmark.cs", [
    ('''	inline bool operator<( BodyId a, BodyId b )
	{
		ulong ua = StoreBodyId( a );
		ulong ub = StoreBodyId( b );
		return ua < ub;
	}
''', ''),
])

patch("SampleCharacter.cs", [
    ('moverDef.capsule = { new( 0.0f, -0.5f ), new( 0.0f, 0.5f ), 0.3f };',
     'moverDef.capsule = new Capsule { center1 = new( 0.0f, -0.5f ), center2 = new( 0.0f, 0.5f ), radius = 0.3f };'),
    ('def.capsule = { new( 0.0f, -0.5f ), new( 0.0f, 0.5f ), 0.3f };',
     'def.capsule = new Capsule { center1 = new( 0.0f, -0.5f ), center2 = new( 0.0f, 0.5f ), radius = 0.3f };'),
])

patch("SampleCollision.cs", [
    ('SimplexVertex* vertices[3] = new( &simplex.v1, ref simplex.v2, ref simplex.v3 );',
     'SimplexVertex[] vertices = { simplex.v1, simplex.v2, simplex.v3 };'),
])

patch("SampleJoints.cs", [
    ('''		static float hertz = 3.0f;
		static float zeta = 0.7f;
		static float maxForce = 1000.0f;
''', '''		float hertz = 3.0f;
		float zeta = 0.7f;
		float maxForce = 1000.0f;
'''),
])

patch("SampleRestitution.cs", [
    ('static string[] labels = { "ground", "one crate", "two crates" };', 'string[] labels = { "ground", "one crate", "two crates" };'),
])

patch("SampleShapes.cs", [
    ('int count = Unsafe.SizeOf<points>() / Unsafe.SizeOf<points[0]>();', 'int count = points.Length;'),
    ('''	union
	{
		Circle m_circle = default;
		Capsule m_capsule = default;
		Segment m_segment = default;
		Polygon m_polygon = default;
	};
''', '''	public Circle m_circle;
	public Capsule m_capsule;
	public Segment m_segment;
	public Polygon m_polygon;
'''),
])
p = os.path.join(SAMPLES, "SampleCollision.cs")
t = open(p, encoding="utf-8").read()
t = re.sub(r"\bShapeUserData\b", "CollisionShapeUserData", t)
open(p, "w", encoding="utf-8", newline="\n").write(t)

patch("SampleCharacter.cs", [
    ("m_rejectedCount.store( 0, std.memory_order_relaxed );", "Volatile.Write( ref m_rejectedCount, 0 );"),
    ("m_rejectedCount.fetch_add( 1, std.memory_order_relaxed );", "Interlocked.Increment( ref m_rejectedCount );"),
    ("m_rejectedCount.load( std.memory_order_relaxed )", "Volatile.Read( ref m_rejectedCount )"),
    ("public std.atomic<int> m_rejectedCount;", "public int m_rejectedCount;"),
    ("using System.Numerics;\n", "using System.Numerics;\nusing System.Threading;\n"),
])
print("ok")
