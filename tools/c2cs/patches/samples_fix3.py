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


# helper objects that are values in C++ are created up front
for f, t in (("SampleCharacter.cs", "GeometricMover"), ("SampleCharacter.cs", "DynamicMover"), ("SampleJoints.cs", "Car"),
             ("SampleJoints.cs", "Donut"), ("SampleWorld.cs", "Car")):
    patch(f, [], [(r"public %s (m_\w+);" % t, r"public %s \1 = new();" % t)])
patch("SampleEvents.cs", [("public Donut[] m_donuts = new Donut[e_count];", "public Donut[] m_donuts = NewArray<Donut>( e_count );")])

patch("SampleCharacter.cs", [
    ("shapeDef.userData = &m_friendlyShape;", "shapeDef.userData = m_friendlyShape;"),
    ("shapeDef.userData = &m_elevatorShape;", "shapeDef.userData = m_elevatorShape;"),
    ('''		foreach ( ref BodyId platformId in m_platformIds )
		{
			platformId = nullBodyId;
		}''', '''		for ( int i = 0; i < m_platformIds.Length; ++i )
		{
			m_platformIds[i] = nullBodyId;
		}'''),
    ("(int)m_mover.m_onGround", "m_mover.m_onGround ? 1 : 0"),
    ("(int)m_mover.m_walkable", "m_mover.m_walkable ? 1 : 0"),
    ("(int)m_mover.m_jumping", "m_mover.m_jumping ? 1 : 0"),
    ('''	public struct MovingPlatform
	{
		public Pos @base;
		public float amplitude;
		public float rate;
	}''', '''	public struct MovingPlatform
	{
		public Pos @base;
		public float amplitude;
		public float rate;

		public MovingPlatform( Pos @base, float amplitude, float rate )
		{
			this.@base = @base;
			this.amplitude = amplitude;
			this.rate = rate;
		}
	}'''),
    ("customColor = i % 2 == 0 ? HexColor.colorGray : HexColor.colorDarkGray;", "customColor = (uint)( i % 2 == 0 ? HexColor.colorGray : HexColor.colorDarkGray );"),
], [(r"^\t\tpoint;\n", "")])

patch("SampleCollision.cs", [
    ("ComputeShapeDistance( ref input, ref m_cache, m_simplexes, m_simplexCapacity )", "ComputeShapeDistance( input, ref m_cache, m_simplexes )"),
    ("Vec2 pointA, pointB = default;", "Vec2 pointA = default, pointB = default;"),
    ("p.proxyId = CreateTreeProxy( m_tree, p.fatBox, DefaultCategoryBits, m_proxyCount );", "p.proxyId = CreateTreeProxy( m_tree, p.fatBox, DefaultCategoryBits, (ulong)m_proxyCount );"),
    ("m_castType = CastType( castType );", "m_castType = castType;"),
    ("m_mode = Mode( mode );", "m_mode = mode;"),
    ("m_segments[i] = new( g1, new( p1, p2 ), g2, -1 );", "m_segments[i] = new ChainSegment { ghost1 = g1, segment = new Segment { point1 = p1, point2 = p2 }, ghost2 = g2, chainId = -1 };"),
    ("public int m_countA = m_verticesA.Length;", "public int m_countA = 4;"),
    ("public int m_countB = m_verticesB.Length;", "public int m_countB = 2;"),
], [
    (r"new\( -0\.5f \* w, b \),(\s*)\{ -0\.5f \* s, 0\.0f \} \};", r"new( -0.5f * w, b ),\1new( -0.5f * s, 0.0f ) };"),
    (r"^(\t+)default:\n(\t+)Debug\.Assert\( false \);\n(\t+)\}", r"\1default:\n\2Debug.Assert( false );\n\2break;\n\3}"),
])

patch("SampleContinuous.cs", [
    ("float m = 1.0f / sqrt( 2.0f );", "float m = 1.0f / MathF.Sqrt( 2.0f );"),
    ("float mm = 2.0f * ( sqrt( 2.0f ) - 1.0f );", "float mm = 2.0f * ( MathF.Sqrt( 2.0f ) - 1.0f );"),
], [
    (r"Body_SetMotionLocks\( m_ballId, new\( false, false, true \) \)", "Body_SetMotionLocks( m_ballId, new MotionLocks { linearX = false, linearY = false, angularZ = true } )"),
    (r"Body_GetContactData\( (\w+), data, 1 \)", r"Body_GetContactData( \1, new Span<ContactData>( ref data ), 1 )"),
])

patch("SampleEvents.cs", [
    ("int index = (int)( donut - m_donuts );", "int index = Array.IndexOf( m_donuts, donut );"),
    ('''		ref Human human = ref (Human*)Body_GetUserData( bodyId );
				if ( human != null )
				{
					int index = (int)( human - m_humans );''', '''		object? humanIndex = Body_GetUserData( bodyId );
				if ( humanIndex != null )
				{
					int index = (int)humanIndex;'''),
    ("CreateHuman( ref human, m_worldId, center, scale, jointFriction, jointHertz, jointDamping, index + 1, human, colorize );",
     "CreateHuman( ref human, m_worldId, center, scale, jointFriction, jointHertz, jointDamping, index + 1, index, colorize );"),
    ("int count = Shape_GetSensorData( m_sensorId, m_visitorIds, capacity );", "int count = Shape_GetSensorData( m_sensorId, m_visitorIds.AsSpan(), capacity );"),
    ("int countA = Shape_GetContactData( @event.shapeIdA, contactData, capacityA );", "int countA = Shape_GetContactData( @event.shapeIdA, contactData.AsSpan(), capacityA );"),
    ("int countB = Shape_GetContactData( @event.shapeIdB, contactData, capacityB );", "int countB = Shape_GetContactData( @event.shapeIdB, contactData.AsSpan(), capacityB );"),
    ("Body_GetShapes( debrisId, shapeId, 1 );", "Body_GetShapes( debrisId, new Span<ShapeId>( ref shapeId ), 1 );"),
    ("B2_ASSERT( ", "Debug.Assert( "),
    ('''			ref BodyId bodyId = ref (BodyId*)( @event.userData );
			ptrdiff_t diff = bodyId - m_bodyIds;''', '''			int diff = (int)@event.userData!;'''),
    ("bodyDef.userData = m_bodyIds[m_count];", "bodyDef.userData = m_count;"),
    ("int count = Shape_GetSensorData( sensorShapeId, m_visitorIds, capacity );", "int count = Shape_GetSensorData( sensorShapeId, m_visitorIds.AsSpan(), capacity );"),
    ('''		int start = buffer = Fmt( "%s: ", prefix );
		for ( int i = 0; i < count && start < Unsafe.SizeOf<buffer>(); ++i )''', '''		buffer = Fmt( "%s: ", prefix );
		for ( int i = 0; i < count && buffer.Length < 256; ++i )'''),
    ('start += buffer + start = Fmt( "%s, ", name );', 'buffer += Fmt( "%s, ", name );'),
], [
    (r"^(\t+)default:\n(\t+)Debug\.Assert\( false \);\n(\t+)\}", r"\1default:\n\2Debug.Assert( false );\n\2break;\n\3}"),
])
print("ok")
