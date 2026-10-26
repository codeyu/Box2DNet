#!/usr/bin/env python3
"""Hand fixes for the converted samples, applied once after conversion."""

from samples_fix1 import patch

patch("SampleBenchmark.cs", [
    ('''		memset( m_bodyIds, 0, m_bodyCount * Unsafe.SizeOf<BodyId>() );
''', ''),
    ('shapeDef.filter.categoryBits = 1 << category;', 'shapeDef.filter.categoryBits = 1ul << category;'),
    ('m_queryType = QueryType( queryType );', 'm_queryType = queryType;'),
    ('MakeProxy( ref Vec2_zero, 1, m_radius )', 'MakeProxy( in Vec2_zero, 1, m_radius )'),
    ('m_outputs = (DistanceOutput*)calloc( m_count, Unsafe.SizeOf<DistanceOutput>() );', 'm_outputs = new DistanceOutput[m_count];'),
    ('shapeDefInner.userData = &m_activeSensor;', 'shapeDefInner.userData = m_activeSensor;'),
    ('std.set<BodyId> zombies;', 'SortedSet<BodyId> zombies = new( Comparer<BodyId>.Create( CppCompat.Compare ) );'),
    ('zombies.emplace( ', 'zombies.Add( '),
])
print("ok")
