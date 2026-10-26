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


patch("DynamicMover.cs", [
    ('''public struct DynamicMoverProbe
{
	public Pos origin;
	public Vec2 translation;
	public float fraction;
	public bool hit;
}''', '''public struct DynamicMoverProbe
{
	public Pos origin;
	public Vec2 translation;
	public float fraction;
	public bool hit;

	public DynamicMoverProbe( Pos origin, Vec2 translation, float fraction, bool hit )
	{
		this.origin = origin;
		this.translation = translation;
		this.fraction = fraction;
		this.hit = hit;
	}
}'''),
    ('''		foreach ( ref DynamicMoverProbe probe in m_probes )
		{
			probe = new( Pos_zero, Vec2_zero, 1.0f, false );
		}''', '''		for ( int i = 0; i < m_probes.Length; ++i )
		{
			m_probes[i] = new( Pos_zero, Vec2_zero, 1.0f, false );
		}'''),
    ('DynamicMoverCastResult result = default;', 'DynamicMoverCastResult result = new();'),
    ('public static float CastCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )',
     'public static float DynamicMoverCastCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )'),
    ('DynamicMoverCastResult? result = (DynamicMoverCastResult?)context;', 'DynamicMoverCastResult result = (DynamicMoverCastResult)context!;'),
], [(r"\bCastCallback, ", "DynamicMoverCastCallback, ")])

patch("GeometricMover.cs", [
    ('self.m_planes[self.m_planeCount] = new( planeResult.plane, maxPush, 0.0f, clipVelocity );',
     'self.m_planes[self.m_planeCount] = new CollisionPlane { plane = planeResult.plane, pushLimit = maxPush, push = 0.0f, clipVelocity = clipVelocity };'),
    ('public static float CastCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )',
     'public static float GeometricMoverCastCallback( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context )'),
], [(r"\bCastCallback, ", "GeometricMoverCastCallback, ")])
print("ok")
