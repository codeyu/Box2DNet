// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace Box2DNet;

public static partial class B2
{
	private static ContactSim s_zeroContactSim;

	// A lane of a wide constraint maps to a contact sim, or to the zero contact sim for empty lanes
	[MethodImpl( MethodImplOptions.AggressiveInlining )]
	private static ref ContactSim LaneSim( Span<ContactSim> contactSims, int index )
	{
		if ( index < 0 )
		{
			return ref s_zeroContactSim;
		}

		return ref contactSims[index];
	}

	// Soft contact constraints with sub-stepping support.
	// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules).
	// http://mmacklin.com/smallsteps.pdf
	// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf
	internal struct ContactConstraintW4
	{
		// Read common
		public FixedArray4<int> indexA;
		public FixedArray4<int> indexB;

		public Vector128<float> invMassA, invMassB;
		public Vector128<float> invIA, invIB;
		public Vec2W normal;
		public Vec2W anchorA1, anchorB1;
		public Vec2W anchorA2, anchorB2;
		public Vector128<float> normalMass1, normalMass2;
		public Vector128<float> baseSeparation1;
		public Vector128<float> baseSeparation2;

		// Write in push and solve
		public Vector128<float> normalImpulse1, normalImpulse2;
		public Vector128<float> totalNormalImpulse1, totalNormalImpulse2;

		// Write in solve
		public Vector128<float> tangentImpulse1, tangentImpulse2;
		public Vector128<float> rollingImpulse;

		// Read in solve
		public Vector128<float> friction;
		public Vector128<float> tangentSpeed;
		public Vector128<float> rollingResistance;
		public Vector128<float> tangentMass1, tangentMass2;
		public Vector128<float> restitution;
		public Vector128<float> relativeVelocity1, relativeVelocity2;
		public Vector128<float> restitutionImpulse1, restitutionImpulse2;
	}

	internal static int GetWideContactConstraintByteCountW4() => Unsafe.SizeOf<ContactConstraintW4>();

	// Macro for gathering contact data across lanes.

	// Layout needed for transpose gathers

	// Prepare wide contact constraints.
	// Potential optimization: for SSE2 load 4 manifold points and transpose, like the body gather/scatter.
	// I would need to reorder the manifold point so that constraint data comes first.
	internal static void PrepareContacts_WideW4( SolverBlock block, ref StepContext context )
	{
		World world = context.world;
		ReadOnlySpanStates states = new ReadOnlySpanStates( context.states );
		Span<ContactPrepareSpan> spans = context.contactPrepareSpans;
		Span<ContactConstraintW4> wideBase = context.wideContactConstraints;
#if BOX2D_VALIDATE
		Span<Body> bodies = world.bodies.data;
#endif

		Vector128<float> zeroW = ZeroW();
		Vector128<float> oneW = SplatW( 1.0f );
		Vector128<float> warmStartScale = world.enableWarmStarting ? oneW : zeroW;
		bool anyRestitution = false;

		int wideIndex = block.startIndex;
		int endWideIndex = block.startIndex + block.count;

		// Find color for start index. Linear search but fast.
		int colorIndex = 0;
		while ( spans[colorIndex + 1].start <= wideIndex )
		{
			colorIndex += 1;
		}

		// Loop over block
		while ( wideIndex < endWideIndex )
		{
			int colorWideStart = spans[colorIndex].start;
			int colorWideEndIndex = MinInt( spans[colorIndex + 1].start, endWideIndex );
			int colorContactCount = spans[colorIndex].count;
			Span<ContactSim> contactSims = spans[colorIndex].contacts;
#if BOX2D_VALIDATE
			int expectedWide = colorContactCount > 0 ? ( ( colorContactCount - 1 ) >> 2 ) + 1 : 0;
			B2_ASSERT( spans[colorIndex + 1].start - spans[colorIndex].start == expectedWide );
#endif

			// Loop over color
			for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
			{
				ref ContactConstraintW4 cw = ref wideBase[wideIndex];
				int localWideIndex = wideIndex - colorWideStart;

				FixedArray4<int> contactLanes = default;
				uint hitEvents = 0;

				for ( int laneIndex = 0; laneIndex < 4; ++laneIndex )
				{
					int contactIndex = 4 * localWideIndex + laneIndex;
					if ( contactIndex < colorContactCount )
					{
						ref ContactSim c = ref contactSims[contactIndex];
						contactLanes[laneIndex] = contactIndex;

						// index base-1
						cw.indexA[laneIndex] = DecodeAwakeIndex( c.encodedBodySimA ) + 1;
						cw.indexB[laneIndex] = DecodeAwakeIndex( c.encodedBodySimB ) + 1;
						hitEvents |= ( c.simFlags & b2_simEnableHitEvent );

#if BOX2D_VALIDATE
						ref Body bodyA = ref bodies[c.bodyIdA];
						ref Body bodyB = ref bodies[c.bodyIdB];
						B2_ASSERT( c.encodedBodySimA == EncodeBodySimIndex( bodyA ) );
						B2_ASSERT( c.encodedBodySimB == EncodeBodySimIndex( bodyB ) );
#endif
					}
					else
					{
						contactLanes[laneIndex] = -1;
					}
				}

				cw.invMassA = SetW( LaneSim( contactSims, contactLanes[0] ).invMassA, LaneSim( contactSims, contactLanes[1] ).invMassA, LaneSim( contactSims, contactLanes[2] ).invMassA, LaneSim( contactSims, contactLanes[3] ).invMassA );
				cw.invMassB = SetW( LaneSim( contactSims, contactLanes[0] ).invMassB, LaneSim( contactSims, contactLanes[1] ).invMassB, LaneSim( contactSims, contactLanes[2] ).invMassB, LaneSim( contactSims, contactLanes[3] ).invMassB );
				cw.invIA = SetW( LaneSim( contactSims, contactLanes[0] ).invIA, LaneSim( contactSims, contactLanes[1] ).invIA, LaneSim( contactSims, contactLanes[2] ).invIA, LaneSim( contactSims, contactLanes[3] ).invIA );
				cw.invIB = SetW( LaneSim( contactSims, contactLanes[0] ).invIB, LaneSim( contactSims, contactLanes[1] ).invIB, LaneSim( contactSims, contactLanes[2] ).invIB, LaneSim( contactSims, contactLanes[3] ).invIB );
				cw.normal.X = SetW( LaneSim( contactSims, contactLanes[0] ).manifold.normal.x, LaneSim( contactSims, contactLanes[1] ).manifold.normal.x, LaneSim( contactSims, contactLanes[2] ).manifold.normal.x, LaneSim( contactSims, contactLanes[3] ).manifold.normal.x );
				cw.normal.Y = SetW( LaneSim( contactSims, contactLanes[0] ).manifold.normal.y, LaneSim( contactSims, contactLanes[1] ).manifold.normal.y, LaneSim( contactSims, contactLanes[2] ).manifold.normal.y, LaneSim( contactSims, contactLanes[3] ).manifold.normal.y );
				cw.friction = SetW( LaneSim( contactSims, contactLanes[0] ).friction, LaneSim( contactSims, contactLanes[1] ).friction, LaneSim( contactSims, contactLanes[2] ).friction, LaneSim( contactSims, contactLanes[3] ).friction );
				cw.tangentSpeed = SetW( LaneSim( contactSims, contactLanes[0] ).tangentSpeed, LaneSim( contactSims, contactLanes[1] ).tangentSpeed, LaneSim( contactSims, contactLanes[2] ).tangentSpeed, LaneSim( contactSims, contactLanes[3] ).tangentSpeed );
				cw.rollingResistance = SetW( LaneSim( contactSims, contactLanes[0] ).rollingResistance, LaneSim( contactSims, contactLanes[1] ).rollingResistance, LaneSim( contactSims, contactLanes[2] ).rollingResistance, LaneSim( contactSims, contactLanes[3] ).rollingResistance );
				cw.rollingImpulse = SetW( LaneSim( contactSims, contactLanes[0] ).manifold.rollingImpulse, LaneSim( contactSims, contactLanes[1] ).manifold.rollingImpulse, LaneSim( contactSims, contactLanes[2] ).manifold.rollingImpulse, LaneSim( contactSims, contactLanes[3] ).manifold.rollingImpulse );
				cw.rollingImpulse = MulW( warmStartScale, cw.rollingImpulse );

				cw.restitution = SetW( LaneSim( contactSims, contactLanes[0] ).restitution, LaneSim( contactSims, contactLanes[1] ).restitution, LaneSim( contactSims, contactLanes[2] ).restitution, LaneSim( contactSims, contactLanes[3] ).restitution );

				Vec2W tangent = RightPerpW( cw.normal );

				Vector128<float> pointCounts = default;
				pointCounts = SetW( (float)LaneSim( contactSims, contactLanes[0] ).manifold.pointCount, (float)LaneSim( contactSims, contactLanes[1] ).manifold.pointCount,
									  (float)LaneSim( contactSims, contactLanes[2] ).manifold.pointCount, (float)LaneSim( contactSims, contactLanes[3] ).manifold.pointCount );
				Vector128<float> twoPointMask = GreaterThanW( pointCounts, oneW );

				{
					// Optimized transpose gather. Similar to GatherBodies.
					Vector128<float> m1a = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[0] ).manifold.points[0], 0 );
					Vector128<float> m1b = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[0] ).manifold.points[0], 4 );
					Vector128<float> m2a = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[1] ).manifold.points[0], 0 );
					Vector128<float> m2b = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[1] ).manifold.points[0], 4 );
					Vector128<float> m3a = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[2] ).manifold.points[0], 0 );
					Vector128<float> m3b = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[2] ).manifold.points[0], 4 );
					Vector128<float> m4a = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[3] ).manifold.points[0], 0 );
					Vector128<float> m4b = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[3] ).manifold.points[0], 4 );

					Vector128<float> t1a = UnpackLoW( m1a, m3a );
					Vector128<float> t2a = UnpackLoW( m2a, m4a );
					Vector128<float> t3a = UnpackHiW( m1a, m3a );
					Vector128<float> t4a = UnpackHiW( m2a, m4a );
					Vector128<float> t1b = UnpackLoW( m1b, m3b );
					Vector128<float> t2b = UnpackLoW( m2b, m4b );
					Vector128<float> t3b = UnpackHiW( m1b, m3b );
					Vector128<float> t4b = UnpackHiW( m2b, m4b );

					cw.anchorA1.X = UnpackLoW( t1a, t2a );
					cw.anchorA1.Y = UnpackHiW( t1a, t2a );
					cw.anchorB1.X = UnpackLoW( t3a, t4a );
					cw.anchorB1.Y = UnpackHiW( t3a, t4a );
					cw.baseSeparation1 = UnpackLoW( t1b, t2b );
					cw.normalImpulse1 = UnpackHiW( t1b, t2b );
					cw.tangentImpulse1 = UnpackLoW( t3b, t4b );

					// separation1 = cw.baseSeparation1;

					Vector128<float> offset = DotW( SubVW( cw.anchorB1, cw.anchorA1 ), cw.normal );
					cw.baseSeparation1 = SubW( cw.baseSeparation1, offset );
					cw.normalImpulse1 = MulW( warmStartScale, cw.normalImpulse1 );
					cw.tangentImpulse1 = MulW( warmStartScale, cw.tangentImpulse1 );
					cw.totalNormalImpulse1 = zeroW;
					cw.relativeVelocity1 = zeroW;
					cw.restitutionImpulse1 = zeroW;

					// Effective mass along normal.
					{
						Vector128<float> rnA = CrossW( cw.anchorA1, cw.normal );
						Vector128<float> rnB = CrossW( cw.anchorB1, cw.normal );
						Vector128<float> k = AddW( cw.invMassA, cw.invMassB );
						k = MulAddW( k, cw.invIA, MulW( rnA, rnA ) );
						k = MulAddW( k, cw.invIB, MulW( rnB, rnB ) );
						Vector128<float> test = GreaterThanW( k, ZeroW() );
						cw.normalMass1 = BlendW( ZeroW(), DivW( oneW, k ), test );
					}

					// Effective mass along tangent.
					{
						Vector128<float> rtA = CrossW( cw.anchorA1, tangent );
						Vector128<float> rtB = CrossW( cw.anchorB1, tangent );
						Vector128<float> k = AddW( cw.invMassA, cw.invMassB );
						k = MulAddW( k, cw.invIA, MulW( rtA, rtA ) );
						k = MulAddW( k, cw.invIB, MulW( rtB, rtB ) );
						Vector128<float> test = GreaterThanW( k, ZeroW() );
						cw.tangentMass1 = BlendW( ZeroW(), DivW( oneW, k ), test );
					}
				}

				{
					Vector128<float> m1a = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[0] ).manifold.points[1], 0 );
					Vector128<float> m1b = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[0] ).manifold.points[1], 4 );
					Vector128<float> m2a = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[1] ).manifold.points[1], 0 );
					Vector128<float> m2b = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[1] ).manifold.points[1], 4 );
					Vector128<float> m3a = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[2] ).manifold.points[1], 0 );
					Vector128<float> m3b = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[2] ).manifold.points[1], 4 );
					Vector128<float> m4a = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[3] ).manifold.points[1], 0 );
					Vector128<float> m4b = LoadManifoldPoint( ref LaneSim( contactSims, contactLanes[3] ).manifold.points[1], 4 );

					Vector128<float> t1a = UnpackLoW( m1a, m3a );
					Vector128<float> t2a = UnpackLoW( m2a, m4a );
					Vector128<float> t3a = UnpackHiW( m1a, m3a );
					Vector128<float> t4a = UnpackHiW( m2a, m4a );
					Vector128<float> t1b = UnpackLoW( m1b, m3b );
					Vector128<float> t2b = UnpackLoW( m2b, m4b );
					Vector128<float> t3b = UnpackHiW( m1b, m3b );
					Vector128<float> t4b = UnpackHiW( m2b, m4b );

					cw.anchorA2.X = UnpackLoW( t1a, t2a );
					cw.anchorA2.Y = UnpackHiW( t1a, t2a );
					cw.anchorB2.X = UnpackLoW( t3a, t4a );
					cw.anchorB2.Y = UnpackHiW( t3a, t4a );
					cw.baseSeparation2 = UnpackLoW( t1b, t2b );
					cw.normalImpulse2 = UnpackHiW( t1b, t2b );
					cw.tangentImpulse2 = UnpackLoW( t3b, t4b );

					// separation2 = cw.baseSeparation2;

					Vector128<float> offset = DotW( SubVW( cw.anchorB2, cw.anchorA2 ), cw.normal );
					cw.baseSeparation2 = SubW( cw.baseSeparation2, offset );
					cw.normalImpulse2 = MulW( warmStartScale, cw.normalImpulse2 );
					cw.tangentImpulse2 = MulW( warmStartScale, cw.tangentImpulse2 );
					cw.totalNormalImpulse2 = zeroW;
					cw.relativeVelocity2 = zeroW;
					cw.restitutionImpulse2 = zeroW;

					// Effective mass along normal.
					{
						Vector128<float> rnA = CrossW( cw.anchorA2, cw.normal );
						Vector128<float> rnB = CrossW( cw.anchorB2, cw.normal );
						Vector128<float> k = AddW( cw.invMassA, cw.invMassB );
						k = MulAddW( k, cw.invIA, MulW( rnA, rnA ) );
						k = MulAddW( k, cw.invIB, MulW( rnB, rnB ) );
						Vector128<float> test = GreaterThanW( k, ZeroW() );
						cw.normalMass2 = BlendW( ZeroW(), DivW( oneW, k ), test );
					}

					// Effective mass along tangent.
					{
						Vector128<float> rtA = CrossW( cw.anchorA2, tangent );
						Vector128<float> rtB = CrossW( cw.anchorB2, tangent );
						Vector128<float> k = AddW( cw.invMassA, cw.invMassB );
						k = MulAddW( k, cw.invIA, MulW( rtA, rtA ) );
						k = MulAddW( k, cw.invIB, MulW( rtB, rtB ) );
						Vector128<float> test = GreaterThanW( k, ZeroW() );
						cw.tangentMass2 = BlendW( ZeroW(), DivW( oneW, k ), test );
					}

					// Zero effective mass on lanes that only have one point.
					cw.normalMass2 = BlendW( zeroW, cw.normalMass2, twoPointMask );
					cw.tangentMass2 = BlendW( zeroW, cw.tangentMass2, twoPointMask );
				}

				// Only sample contact point normal velocity if needed.
				Vector128<float> restitutionMask = GreaterThanW( cw.restitution, zeroW );
				bool haveRestitution = AnyTrueW( restitutionMask );
				anyRestitution = anyRestitution || haveRestitution;

				if ( ( hitEvents & b2_simEnableHitEvent ) != 0 || haveRestitution )
				{
					BodyStateW bA = GatherBodies( states, cw.indexA );
					BodyStateW bB = GatherBodies( states, cw.indexB );

					{
						Vec2W rA = cw.anchorA1;
						Vec2W rB = cw.anchorB1;

						// Relative velocity at contact
						Vector128<float> dvx = SubW( SubW( bB.v.X, MulW( bB.w, rB.Y ) ), SubW( bA.v.X, MulW( bA.w, rA.Y ) ) );
						Vector128<float> dvy = SubW( AddW( bB.v.Y, MulW( bB.w, rB.X ) ), AddW( bA.v.Y, MulW( bA.w, rA.X ) ) );
						Vector128<float> vn = AddW( MulW( dvx, cw.normal.X ), MulW( dvy, cw.normal.Y ) );
						cw.relativeVelocity1 = vn;

						Vector128<float> normalVelocities = vn;

						for ( int lane = 0; lane < 4; ++lane )
						{
							// Check the point count to avoid a race writing to zeroContactSim.
							if ( LaneSim( contactSims, contactLanes[lane] ).manifold.pointCount > 0 )
							{
								LaneSim( contactSims, contactLanes[lane] ).manifold.points[0].normalVelocity = normalVelocities[lane];
							}
						}
					}

					{
						Vec2W rA = cw.anchorA2;
						Vec2W rB = cw.anchorB2;

						// Relative velocity at contact
						Vector128<float> dvx = SubW( SubW( bB.v.X, MulW( bB.w, rB.Y ) ), SubW( bA.v.X, MulW( bA.w, rA.Y ) ) );
						Vector128<float> dvy = SubW( AddW( bB.v.Y, MulW( bB.w, rB.X ) ), AddW( bA.v.Y, MulW( bA.w, rA.X ) ) );
						Vector128<float> vn = AddW( MulW( dvx, cw.normal.X ), MulW( dvy, cw.normal.Y ) );
						cw.relativeVelocity2 = vn;

						Vector128<float> normalVelocities = vn;

						for ( int lane = 0; lane < 4; ++lane )
						{
							if ( LaneSim( contactSims, contactLanes[lane] ).manifold.pointCount > 1 )
							{
								LaneSim( contactSims, contactLanes[lane] ).manifold.points[1].normalVelocity = normalVelocities[lane];
							}
						}
					}
				}
			}

			// Advance to next color
			colorIndex += 1;
		}

		if ( anyRestitution )
		{
			AtomicStoreInt( ref context.anyRestitution, 1 );
		}
	}

	internal static void WarmStartContacts_WideW4( SolverBlock block, ref StepContext context )
	{
		ReadOnlySpanStates states = new ReadOnlySpanStates( context.states );
		Span<ContactConstraintW4> constraints = context.wideContactConstraints.AsSpan( context.graph.colors[block.colorIndex].wideConstraintStart );
		for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
		{
			ref ContactConstraintW4 c = ref constraints[i];
			BodyStateW bA = GatherBodies( states, c.indexA );
			BodyStateW bB = GatherBodies( states, c.indexB );

			Vector128<float> tangentX = c.normal.Y;
			Vector128<float> tangentY = SubW( ZeroW(), c.normal.X );

			{
				// fixed anchors
				Vec2W rA = c.anchorA1;
				Vec2W rB = c.anchorB1;

				Vec2W P = default;
				P.X = AddW( MulW( c.normalImpulse1, c.normal.X ), MulW( c.tangentImpulse1, tangentX ) );
				P.Y = AddW( MulW( c.normalImpulse1, c.normal.Y ), MulW( c.tangentImpulse1, tangentY ) );
				bA.w = MulSubW( bA.w, c.invIA, CrossW( rA, P ) );
				bA.v.X = MulSubW( bA.v.X, c.invMassA, P.X );
				bA.v.Y = MulSubW( bA.v.Y, c.invMassA, P.Y );
				bB.w = MulAddW( bB.w, c.invIB, CrossW( rB, P ) );
				bB.v.X = MulAddW( bB.v.X, c.invMassB, P.X );
				bB.v.Y = MulAddW( bB.v.Y, c.invMassB, P.Y );

				c.totalNormalImpulse1 = AddW( c.totalNormalImpulse1, c.normalImpulse1 );
			}

			{
				// fixed anchors
				Vec2W rA = c.anchorA2;
				Vec2W rB = c.anchorB2;

				Vec2W P = default;
				P.X = AddW( MulW( c.normalImpulse2, c.normal.X ), MulW( c.tangentImpulse2, tangentX ) );
				P.Y = AddW( MulW( c.normalImpulse2, c.normal.Y ), MulW( c.tangentImpulse2, tangentY ) );
				bA.w = MulSubW( bA.w, c.invIA, CrossW( rA, P ) );
				bA.v.X = MulSubW( bA.v.X, c.invMassA, P.X );
				bA.v.Y = MulSubW( bA.v.Y, c.invMassA, P.Y );
				bB.w = MulAddW( bB.w, c.invIB, CrossW( rB, P ) );
				bB.v.X = MulAddW( bB.v.X, c.invMassB, P.X );
				bB.v.Y = MulAddW( bB.v.Y, c.invMassB, P.Y );

				c.totalNormalImpulse2 = AddW( c.totalNormalImpulse2, c.normalImpulse2 );
			}

			bA.w = MulSubW( bA.w, c.invIA, c.rollingImpulse );
			bB.w = MulAddW( bB.w, c.invIB, c.rollingImpulse );

			ScatterBodies( states, c.indexA, ref bA );
			ScatterBodies( states, c.indexB, ref bB );
		}
	}

	internal static void PushContacts_WideW4( SolverBlock block, ref StepContext context )
	{
		ReadOnlySpanStates states = new ReadOnlySpanStates( context.states );
		ref GraphColor color = ref context.graph.colors[block.colorIndex];
		Span<ContactConstraintW4> constraints = context.wideContactConstraints.AsSpan( color.wideConstraintStart );
		Vector128<float> inv_h = SplatW( context.inv_h );
		Vector128<float> contactSpeed = SplatW( -context.world.contactSpeed );
		Vector128<float> oneW = SplatW( 1.0f );
		Vector128<float> zeroW = ZeroW();

		Vector128<float> dynamicBiasRate = SplatW( context.contactSoftness.massScale * context.contactSoftness.biasRate );
		Vector128<float> dynamicMassScale = SplatW( context.contactSoftness.massScale );
		Vector128<float> dynamicImpulseScale = SplatW( context.contactSoftness.impulseScale );
		Vector128<float> staticBiasRate = SplatW( context.staticSoftness.massScale * context.staticSoftness.biasRate );
		Vector128<float> staticMassScale = SplatW( context.staticSoftness.massScale );
		Vector128<float> staticImpulseScale = SplatW( context.staticSoftness.impulseScale );

		for ( int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; ++wideIndex )
		{
			ref ContactConstraintW4 c = ref constraints[wideIndex];
			BodyStateW bA = GatherBodies( states, c.indexA );
			BodyStateW bB = GatherBodies( states, c.indexB );

			Vector128<float> softMask = SoftMaskW( c.indexA, c.indexB );
			Vector128<float> biasRate = BlendW( dynamicBiasRate, staticBiasRate, softMask );
			Vector128<float> massScale = BlendW( dynamicMassScale, staticMassScale, softMask );
			Vector128<float> impulseScale = BlendW( dynamicImpulseScale, staticImpulseScale, softMask );

			Vec2W normal = c.normal;
			Vec2W dp = new Vec2W { X = SubW( bB.dp.X, bA.dp.X ), Y = SubW( bB.dp.Y, bA.dp.Y ) };

			Vector128<float> velocityBias1, massScale1, impulseScale1;
			{
				// Moving anchors for current separation
				Vec2W rsA = RotateVectorW( bA.dq, c.anchorA1 );
				Vec2W rsB = RotateVectorW( bB.dq, c.anchorB1 );

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				Vec2W ds = new Vec2W { X = AddW( dp.X, SubW( rsB.X, rsA.X ) ), Y = AddW( dp.Y, SubW( rsB.Y, rsA.Y ) ) };
				Vector128<float> s = AddW( DotW( normal, ds ), c.baseSeparation1 );

				// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
				// The contactSpeed is meant to limit stiffness, not increase it.
				Vector128<float> separated = GreaterThanW( s, zeroW );

				// Speculative bias - positive
				Vector128<float> specBias = MulW( s, inv_h );

				// Overlap bias - negative when overlapped
				Vector128<float> overlapBias = MaxW( MulW( biasRate, s ), contactSpeed );

				// velocity bias = s > 0 ? spec : overlap
				velocityBias1 = BlendW( overlapBias, specBias, separated );

				massScale1 = BlendW( massScale, oneW, separated );
				impulseScale1 = BlendW( impulseScale, zeroW, separated );
			}

			Vector128<float> velocityBias2, massScale2, impulseScale2;
			{
				Vec2W rsA = RotateVectorW( bA.dq, c.anchorA2 );
				Vec2W rsB = RotateVectorW( bB.dq, c.anchorB2 );

				Vec2W ds = new Vec2W { X = AddW( dp.X, SubW( rsB.X, rsA.X ) ), Y = AddW( dp.Y, SubW( rsB.Y, rsA.Y ) ) };
				Vector128<float> s = AddW( DotW( normal, ds ), c.baseSeparation2 );

				Vector128<float> separated = GreaterThanW( s, zeroW );
				Vector128<float> specBias = MulW( s, inv_h );
				Vector128<float> overlapBias = MaxW( MulW( biasRate, s ), contactSpeed );
				velocityBias2 = BlendW( overlapBias, specBias, separated );

				massScale2 = BlendW( massScale, oneW, separated );
				impulseScale2 = BlendW( impulseScale, zeroW, separated );
			}

			Vector128<float> rnA1 = CrossW( c.anchorA1, normal );
			Vector128<float> rnB1 = CrossW( c.anchorB1, normal );
			Vector128<float> rnA2 = CrossW( c.anchorA2, normal );
			Vector128<float> rnB2 = CrossW( c.anchorB2, normal );

			Vector128<float> k21 = AddW( AddW( c.invMassA, c.invMassB ),
								   AddW( MulW( MulW( c.invIA, rnA1 ), rnA2 ), MulW( MulW( c.invIB, rnB1 ), rnB2 ) ) );

			Vector128<float> dn = DotW( SubVW( bB.v, bA.v ), normal );
			Vector128<float> vn1 = AddW( dn, SubW( MulW( bB.w, rnB1 ), MulW( bA.w, rnA1 ) ) );
			Vector128<float> vn2 = AddW( dn, SubW( MulW( bB.w, rnB2 ), MulW( bA.w, rnA2 ) ) );

			Vector128<float> negImpulse1 = AddW( MulW( c.normalMass1, AddW( MulW( massScale1, vn1 ), velocityBias1 ) ),
										   MulW( impulseScale1, c.normalImpulse1 ) );
			Vector128<float> impulse1 = MaxW( NegW( negImpulse1 ), NegW( c.normalImpulse1 ) );
			c.normalImpulse1 = AddW( c.normalImpulse1, impulse1 );
			c.totalNormalImpulse1 = AddW( c.totalNormalImpulse1, impulse1 );

			vn2 = MulAddW( vn2, k21, impulse1 );

			Vector128<float> negImpulse2 = AddW( MulW( c.normalMass2, AddW( MulW( massScale2, vn2 ), velocityBias2 ) ),
										   MulW( impulseScale2, c.normalImpulse2 ) );
			Vector128<float> impulse2 = MaxW( NegW( negImpulse2 ), NegW( c.normalImpulse2 ) );
			c.normalImpulse2 = AddW( c.normalImpulse2, impulse2 );
			c.totalNormalImpulse2 = AddW( c.totalNormalImpulse2, impulse2 );

			Vector128<float> impulse = AddW( impulse1, impulse2 );
			Vector128<float> Px = MulW( impulse, normal.X );
			Vector128<float> Py = MulW( impulse, normal.Y );
			Vector128<float> angularA = AddW( MulW( impulse1, rnA1 ), MulW( impulse2, rnA2 ) );
			Vector128<float> angularB = AddW( MulW( impulse1, rnB1 ), MulW( impulse2, rnB2 ) );

			bA.v.X = MulSubW( bA.v.X, c.invMassA, Px );
			bA.v.Y = MulSubW( bA.v.Y, c.invMassA, Py );
			bA.w = MulSubW( bA.w, c.invIA, angularA );

			bB.v.X = MulAddW( bB.v.X, c.invMassB, Px );
			bB.v.Y = MulAddW( bB.v.Y, c.invMassB, Py );
			bB.w = MulAddW( bB.w, c.invIB, angularB );

			ScatterBodies( states, c.indexA, ref bA );
			ScatterBodies( states, c.indexB, ref bB );
		}
	}

	// Solve the normal constraint, friction, and rolling resistance.
	internal static void SolveContacts_WideW4( SolverBlock block, ref StepContext context )
	{
		ReadOnlySpanStates states = new ReadOnlySpanStates( context.states );
		ref GraphColor color = ref context.graph.colors[block.colorIndex];
		Span<ContactConstraintW4> constraints = context.wideContactConstraints.AsSpan( color.wideConstraintStart );
		Vector128<float> inv_h = SplatW( context.inv_h );
		Vector128<float> zeroW = ZeroW();

		for ( int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; ++wideIndex )
		{
			ref ContactConstraintW4 c = ref constraints[wideIndex];
			BodyStateW bA = GatherBodies( states, c.indexA );
			BodyStateW bB = GatherBodies( states, c.indexB );

			Vec2W normal = c.normal;
			Vec2W tangent = RightPerpW( normal );
			Vec2W dp = new Vec2W { X = SubW( bB.dp.X, bA.dp.X ), Y = SubW( bB.dp.Y, bA.dp.Y ) };

			Vector128<float> velocityBias1, velocityBias2;
			{
				// Moving anchors for current separation
				Vec2W rsA = RotateVectorW( bA.dq, c.anchorA1 );
				Vec2W rsB = RotateVectorW( bB.dq, c.anchorB1 );

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				Vec2W ds = new Vec2W { X = AddW( dp.X, SubW( rsB.X, rsA.X ) ), Y = AddW( dp.Y, SubW( rsB.Y, rsA.Y ) ) };
				Vector128<float> s = AddW( DotW( normal, ds ), c.baseSeparation1 );

				// Speculative bias, positive if separated and zero if overlapped
				velocityBias1 = MaxW( zeroW, MulW( s, inv_h ) );
			}

			{
				Vec2W rsA = RotateVectorW( bA.dq, c.anchorA2 );
				Vec2W rsB = RotateVectorW( bB.dq, c.anchorB2 );

				Vec2W ds = new Vec2W { X = AddW( dp.X, SubW( rsB.X, rsA.X ) ), Y = AddW( dp.Y, SubW( rsB.Y, rsA.Y ) ) };
				Vector128<float> s = AddW( DotW( normal, ds ), c.baseSeparation2 );

				velocityBias2 = MaxW( zeroW, MulW( s, inv_h ) );
			}

			Vector128<float> rnA1 = CrossW( c.anchorA1, normal );
			Vector128<float> rnB1 = CrossW( c.anchorB1, normal );
			Vector128<float> rnA2 = CrossW( c.anchorA2, normal );
			Vector128<float> rnB2 = CrossW( c.anchorB2, normal );
			Vector128<float> rtA1 = CrossW( c.anchorA1, tangent );
			Vector128<float> rtB1 = CrossW( c.anchorB1, tangent );
			Vector128<float> rtA2 = CrossW( c.anchorA2, tangent );
			Vector128<float> rtB2 = CrossW( c.anchorB2, tangent );

			Vector128<float> wnA1 = MulW( c.invIA, rnA1 );
			Vector128<float> wnB1 = MulW( c.invIB, rnB1 );
			Vector128<float> wnA2 = MulW( c.invIA, rnA2 );
			Vector128<float> wnB2 = MulW( c.invIB, rnB2 );
			Vector128<float> wtA1 = MulW( c.invIA, rtA1 );
			Vector128<float> wtB1 = MulW( c.invIB, rtB1 );

			Vector128<float> invMass = AddW( c.invMassA, c.invMassB );
			Vector128<float> k21 = AddW( invMass, AddW( MulW( wnA1, rnA2 ), MulW( wnB1, rnB2 ) ) );
			Vector128<float> k31 = AddW( MulW( wnA1, rtA1 ), MulW( wnB1, rtB1 ) );
			Vector128<float> k32 = AddW( MulW( wnA2, rtA1 ), MulW( wnB2, rtB1 ) );
			Vector128<float> k41 = AddW( MulW( wnA1, rtA2 ), MulW( wnB1, rtB2 ) );
			Vector128<float> k42 = AddW( MulW( wnA2, rtA2 ), MulW( wnB2, rtB2 ) );
			Vector128<float> k43 = AddW( invMass, AddW( MulW( wtA1, rtA2 ), MulW( wtB1, rtB2 ) ) );

			Vec2W dv = SubVW( bB.v, bA.v );
			Vector128<float> dn = DotW( dv, normal );
			Vector128<float> dt = SubW( DotW( dv, tangent ), c.tangentSpeed );

			Vector128<float> vn1 = AddW( dn, SubW( MulW( bB.w, rnB1 ), MulW( bA.w, rnA1 ) ) );
			Vector128<float> vn2 = AddW( dn, SubW( MulW( bB.w, rnB2 ), MulW( bA.w, rnA2 ) ) );
			Vector128<float> vt1 = AddW( dt, SubW( MulW( bB.w, rtB1 ), MulW( bA.w, rtA1 ) ) );
			Vector128<float> vt2 = AddW( dt, SubW( MulW( bB.w, rtB2 ), MulW( bA.w, rtA2 ) ) );

			Vector128<float> normalImpulse1 = MaxW( NegW( MulW( c.normalMass1, AddW( vn1, velocityBias1 ) ) ),
											  NegW( c.normalImpulse1 ) );
			c.normalImpulse1 = AddW( c.normalImpulse1, normalImpulse1 );
			c.totalNormalImpulse1 = AddW( c.totalNormalImpulse1, normalImpulse1 );

			vn2 = MulAddW( vn2, k21, normalImpulse1 );

			Vector128<float> normalImpulse2 = MaxW( NegW( MulW( c.normalMass2, AddW( vn2, velocityBias2 ) ) ),
											  NegW( c.normalImpulse2 ) );
			c.normalImpulse2 = AddW( c.normalImpulse2, normalImpulse2 );
			c.totalNormalImpulse2 = AddW( c.totalNormalImpulse2, normalImpulse2 );

			vt1 = MulAddW( MulAddW( vt1, k31, normalImpulse1 ), k32, normalImpulse2 );
			vt2 = MulAddW( MulAddW( vt2, k41, normalImpulse1 ), k42, normalImpulse2 );

			Vector128<float> angularA = AddW( MulW( normalImpulse1, rnA1 ), MulW( normalImpulse2, rnA2 ) );
			Vector128<float> angularB = AddW( MulW( normalImpulse1, rnB1 ), MulW( normalImpulse2, rnB2 ) );

			// Rolling resistance
			if ( AllZeroW( c.rollingResistance ) == false )
			{
				Vector128<float> relativeW = SubW( SubW( bA.w, bB.w ), AddW( MulW( AddW( wnA1, wnB1 ), normalImpulse1 ),
																		  MulW( AddW( wnA2, wnB2 ), normalImpulse2 ) ) );
				Vector128<float> k = AddW( c.invIA, c.invIB );
				Vector128<float> mask = GreaterThanW( k, zeroW );
				Vector128<float> deltaLambda = BlendW( zeroW, DivW( relativeW, k ), mask );
				Vector128<float> lambda = c.rollingImpulse;
				Vector128<float> maxLambda = MulW( c.rollingResistance, AddW( c.normalImpulse1, c.normalImpulse2 ) );
				c.rollingImpulse = SymClampW( AddW( lambda, deltaLambda ), maxLambda );
				deltaLambda = SubW( c.rollingImpulse, lambda );

				Vector128<float> wtA2 = MulW( c.invIA, rtA2 );
				Vector128<float> wtB2 = MulW( c.invIB, rtB2 );
				vt1 = MulAddW( vt1, AddW( wtA1, wtB1 ), deltaLambda );
				vt2 = MulAddW( vt2, AddW( wtA2, wtB2 ), deltaLambda );

				angularA = AddW( angularA, deltaLambda );
				angularB = AddW( angularB, deltaLambda );
			}

			Vector128<float> maxFriction1 = MulW( c.friction, c.normalImpulse1 );
			Vector128<float> tangentImpulse1 = NegW( MulW( c.tangentMass1, vt1 ) );
			tangentImpulse1 = MaxW( SubW( NegW( maxFriction1 ), c.tangentImpulse1 ),
									  MinW( tangentImpulse1, SubW( maxFriction1, c.tangentImpulse1 ) ) );
			c.tangentImpulse1 = AddW( c.tangentImpulse1, tangentImpulse1 );

			vt2 = MulAddW( vt2, k43, tangentImpulse1 );

			Vector128<float> maxFriction2 = MulW( c.friction, c.normalImpulse2 );
			Vector128<float> tangentImpulse2 = NegW( MulW( c.tangentMass2, vt2 ) );
			tangentImpulse2 = MaxW( SubW( NegW( maxFriction2 ), c.tangentImpulse2 ),
									  MinW( tangentImpulse2, SubW( maxFriction2, c.tangentImpulse2 ) ) );
			c.tangentImpulse2 = AddW( c.tangentImpulse2, tangentImpulse2 );

			angularA = AddW( angularA, AddW( MulW( tangentImpulse1, rtA1 ), MulW( tangentImpulse2, rtA2 ) ) );
			angularB = AddW( angularB, AddW( MulW( tangentImpulse1, rtB1 ), MulW( tangentImpulse2, rtB2 ) ) );

			Vector128<float> normalImpulse = AddW( normalImpulse1, normalImpulse2 );
			Vector128<float> tangentImpulse = AddW( tangentImpulse1, tangentImpulse2 );
			Vector128<float> Px = AddW( MulW( normalImpulse, normal.X ), MulW( tangentImpulse, tangent.X ) );
			Vector128<float> Py = AddW( MulW( normalImpulse, normal.Y ), MulW( tangentImpulse, tangent.Y ) );

			bA.v.X = MulSubW( bA.v.X, c.invMassA, Px );
			bA.v.Y = MulSubW( bA.v.Y, c.invMassA, Py );
			bA.w = MulSubW( bA.w, c.invIA, angularA );

			bB.v.X = MulAddW( bB.v.X, c.invMassB, Px );
			bB.v.Y = MulAddW( bB.v.Y, c.invMassB, Py );
			bB.w = MulAddW( bB.w, c.invIB, angularB );

			ScatterBodies( states, c.indexA, ref bA );
			ScatterBodies( states, c.indexB, ref bB );
		}
	}

	internal static void ApplyRestitution_WideW4( SolverBlock block, ref StepContext context )
	{
		ReadOnlySpanStates states = new ReadOnlySpanStates( context.states );
		ref GraphColor color = ref context.graph.colors[block.colorIndex];
		Span<ContactConstraintW4> constraints = context.wideContactConstraints.AsSpan( color.wideConstraintStart );
		Vector128<float> inv_h = SplatW( context.inv_h );
		Vector128<float> negRestitutionThreshold = SplatW( -context.world.restitutionThreshold );
		Vector128<float> zeroW = ZeroW();
		bool propagate = context.world.enableRestitutionPropagation;

		for ( int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; ++wideIndex )
		{
			ref ContactConstraintW4 c = ref constraints[wideIndex];
			if ( propagate == false && AllZeroW( c.restitution ) )
			{
				continue;
			}

			BodyStateW bA = GatherBodies( states, c.indexA );
			BodyStateW bB = GatherBodies( states, c.indexB );

			Vector128<float> restitutionMask = GreaterThanW( c.restitution, zeroW );
			Vector128<float> normalMass1 = propagate ? c.normalMass1 : BlendW( zeroW, c.normalMass1, restitutionMask );
			Vector128<float> normalMass2 = propagate ? c.normalMass2 : BlendW( zeroW, c.normalMass2, restitutionMask );
			Vec2W dp = new Vec2W { X = SubW( bB.dp.X, bA.dp.X ), Y = SubW( bB.dp.Y, bA.dp.Y ) };

			{
				Vec2W rA = c.anchorA1;
				Vec2W rB = c.anchorB1;

				Vector128<float> compressionImpulse = SubW( c.totalNormalImpulse1, c.restitutionImpulse1 );
				Vector128<float> armed = AndW( AndW( restitutionMask, LessThanW( c.relativeVelocity1, negRestitutionThreshold ) ),
										 GreaterThanW( compressionImpulse, zeroW ) );

				Vec2W rsA = RotateVectorW( bA.dq, rA );
				Vec2W rsB = RotateVectorW( bB.dq, rB );
				Vec2W ds = new Vec2W { X = AddW( dp.X, SubW( rsB.X, rsA.X ) ), Y = AddW( dp.Y, SubW( rsB.Y, rsA.Y ) ) };
				Vector128<float> s = AddW( DotW( c.normal, ds ), c.baseSeparation1 );

				Vector128<float> specBias = MaxW( zeroW, MulW( s, inv_h ) );
				Vector128<float> velocityBias = BlendW( specBias, MulW( c.restitution, c.relativeVelocity1 ), armed );

				Vector128<float> dvx = SubW( SubW( bB.v.X, MulW( bB.w, rB.Y ) ), SubW( bA.v.X, MulW( bA.w, rA.Y ) ) );
				Vector128<float> dvy = SubW( AddW( bB.v.Y, MulW( bB.w, rB.X ) ), AddW( bA.v.Y, MulW( bA.w, rA.X ) ) );
				Vector128<float> vn = AddW( MulW( dvx, c.normal.X ), MulW( dvy, c.normal.Y ) );

				Vector128<float> negImpulse = MulW( normalMass1, AddW( vn, velocityBias ) );

				Vector128<float> newImpulse = MaxW( SubW( c.normalImpulse1, negImpulse ), zeroW );
				Vector128<float> impulse = SubW( newImpulse, c.normalImpulse1 );

				Vector128<float> approachImpulse = MinW( MaxW( NegW( MulW( normalMass1, vn ) ), zeroW ), MaxW( impulse, zeroW ) );
				Vector128<float> allowance =
					SubW( MulW( c.restitution, AddW( compressionImpulse, approachImpulse ) ), c.restitutionImpulse1 );
				Vector128<float> maxImpulse = AddW( approachImpulse, MaxW( allowance, zeroW ) );
				impulse = BlendW( impulse, MinW( impulse, maxImpulse ), armed );

				c.normalImpulse1 = AddW( c.normalImpulse1, impulse );
				c.restitutionImpulse1 = AddW( c.restitutionImpulse1, SubW( impulse, approachImpulse ) );
				c.totalNormalImpulse1 = AddW( c.totalNormalImpulse1, impulse );

				Vector128<float> Px = MulW( impulse, c.normal.X );
				Vector128<float> Py = MulW( impulse, c.normal.Y );

				bA.v.X = MulSubW( bA.v.X, c.invMassA, Px );
				bA.v.Y = MulSubW( bA.v.Y, c.invMassA, Py );
				bA.w = MulSubW( bA.w, c.invIA, SubW( MulW( rA.X, Py ), MulW( rA.Y, Px ) ) );

				bB.v.X = MulAddW( bB.v.X, c.invMassB, Px );
				bB.v.Y = MulAddW( bB.v.Y, c.invMassB, Py );
				bB.w = MulAddW( bB.w, c.invIB, SubW( MulW( rB.X, Py ), MulW( rB.Y, Px ) ) );
			}

			{
				Vec2W rA = c.anchorA2;
				Vec2W rB = c.anchorB2;

				Vector128<float> compressionImpulse = SubW( c.totalNormalImpulse2, c.restitutionImpulse2 );
				Vector128<float> armed = AndW( AndW( restitutionMask, LessThanW( c.relativeVelocity2, negRestitutionThreshold ) ),
										 GreaterThanW( compressionImpulse, zeroW ) );

				Vec2W rsA = RotateVectorW( bA.dq, rA );
				Vec2W rsB = RotateVectorW( bB.dq, rB );
				Vec2W ds = new Vec2W { X = AddW( dp.X, SubW( rsB.X, rsA.X ) ), Y = AddW( dp.Y, SubW( rsB.Y, rsA.Y ) ) };
				Vector128<float> s = AddW( DotW( c.normal, ds ), c.baseSeparation2 );

				Vector128<float> specBias = MaxW( zeroW, MulW( s, inv_h ) );
				Vector128<float> velocityBias = BlendW( specBias, MulW( c.restitution, c.relativeVelocity2 ), armed );

				Vector128<float> dvx = SubW( SubW( bB.v.X, MulW( bB.w, rB.Y ) ), SubW( bA.v.X, MulW( bA.w, rA.Y ) ) );
				Vector128<float> dvy = SubW( AddW( bB.v.Y, MulW( bB.w, rB.X ) ), AddW( bA.v.Y, MulW( bA.w, rA.X ) ) );
				Vector128<float> vn = AddW( MulW( dvx, c.normal.X ), MulW( dvy, c.normal.Y ) );

				Vector128<float> negImpulse = MulW( normalMass2, AddW( vn, velocityBias ) );

				Vector128<float> newImpulse = MaxW( SubW( c.normalImpulse2, negImpulse ), zeroW );
				Vector128<float> impulse = SubW( newImpulse, c.normalImpulse2 );

				Vector128<float> approachImpulse = MinW( MaxW( NegW( MulW( normalMass2, vn ) ), zeroW ), MaxW( impulse, zeroW ) );
				Vector128<float> allowance =
					SubW( MulW( c.restitution, AddW( compressionImpulse, approachImpulse ) ), c.restitutionImpulse2 );
				Vector128<float> maxImpulse = AddW( approachImpulse, MaxW( allowance, zeroW ) );
				impulse = BlendW( impulse, MinW( impulse, maxImpulse ), armed );

				c.normalImpulse2 = AddW( c.normalImpulse2, impulse );
				c.restitutionImpulse2 = AddW( c.restitutionImpulse2, SubW( impulse, approachImpulse ) );
				c.totalNormalImpulse2 = AddW( c.totalNormalImpulse2, impulse );

				Vector128<float> Px = MulW( impulse, c.normal.X );
				Vector128<float> Py = MulW( impulse, c.normal.Y );

				bA.v.X = MulSubW( bA.v.X, c.invMassA, Px );
				bA.v.Y = MulSubW( bA.v.Y, c.invMassA, Py );
				bA.w = MulSubW( bA.w, c.invIA, SubW( MulW( rA.X, Py ), MulW( rA.Y, Px ) ) );

				bB.v.X = MulAddW( bB.v.X, c.invMassB, Px );
				bB.v.Y = MulAddW( bB.v.Y, c.invMassB, Py );
				bB.w = MulAddW( bB.w, c.invIB, SubW( MulW( rB.X, Py ), MulW( rB.Y, Px ) ) );
			}

			ScatterBodies( states, c.indexA, ref bA );
			ScatterBodies( states, c.indexB, ref bB );
		}
	}

	// I tried adding this to the last relax iterations but it was slower.
	//
	// Runs as a flat parallel-for over the whole wide constraint range. Per-color
	// contact sims are looked up through the prepareSpans cursor rather than the
	// block's colorIndex, matching the layout of PrepareContacts_Wide.
	//
	// I could store the manifold pointer in the ContactConstraintW4 to simplify
	// this. But it is important to keep the constraint small.
	internal static void StoreImpulses_WideW4( SolverBlock block, ref StepContext context, int workerIndex )
	{
		World world = context.world;
		ReadOnlySpan<ContactPrepareSpan> spans = context.contactPrepareSpans;
		ReadOnlySpan<ContactConstraintW4> wideBase = context.wideContactConstraints;
		ref TaskContext taskContext = ref world.taskContexts.data[workerIndex];
		ref BitSet hitEventBitSet = ref taskContext.hitEventBitSet;
		bool hasHitEvents = taskContext.hasHitEvents;
		float negHitThreshold = -world.hitEventThreshold;

		int wideIndex = block.startIndex;
		int endWideIndex = block.startIndex + block.count;

		// Find color for start index
		int colorIndex = 0;
		while ( spans[colorIndex + 1].start <= wideIndex )
		{
			colorIndex += 1;
		}

		while ( wideIndex < endWideIndex )
		{
			int colorWideEndIndex = MinInt( spans[colorIndex + 1].start, endWideIndex );
			int colorWideStart = spans[colorIndex].start;
			int colorContactCount = spans[colorIndex].count;
			Span<ContactSim> contactSims = spans[colorIndex].contacts;
			for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
			{
				ref readonly ContactConstraintW4 c = ref wideBase[wideIndex];
				Vector128<float> rollingImpulse = c.rollingImpulse;
				Vector128<float> normalImpulse1 = c.normalImpulse1;
				Vector128<float> normalImpulse2 = c.normalImpulse2;
				Vector128<float> tangentImpulse1 = c.tangentImpulse1;
				Vector128<float> tangentImpulse2 = c.tangentImpulse2;
				Vector128<float> totalNormalImpulse1 = c.totalNormalImpulse1;
				Vector128<float> totalNormalImpulse2 = c.totalNormalImpulse2;
				int localWideIndex = wideIndex - colorWideStart;
				int baseIndex = 4 * localWideIndex;

				for ( int laneIndex = 0; laneIndex < 4; ++laneIndex )
				{
					int contactIndex = baseIndex + laneIndex;
					if ( contactIndex >= colorContactCount )
					{
						break;
					}

					ref ContactSim contactSim = ref contactSims[contactIndex];
					ref Manifold m = ref contactSim.manifold;
					m.rollingImpulse = rollingImpulse[laneIndex];

					m.points[0].normalImpulse = normalImpulse1[laneIndex];
					m.points[0].tangentImpulse = tangentImpulse1[laneIndex];
					m.points[0].totalNormalImpulse = totalNormalImpulse1[laneIndex];

					m.points[1].normalImpulse = normalImpulse2[laneIndex];
					m.points[1].tangentImpulse = tangentImpulse2[laneIndex];
					m.points[1].totalNormalImpulse = totalNormalImpulse2[laneIndex];

					// Check for hit events to speed up serial processing later in the step
					if ( ( contactSim.simFlags & b2_simEnableHitEvent ) != 0 )
					{
						for ( int k = 0; k < contactSim.manifold.pointCount; ++k )
						{
							ref ManifoldPoint mp = ref m.points[k];
							// Need to check total impulse because the point may be speculative and not colliding
							if ( mp.normalVelocity < negHitThreshold && mp.totalNormalImpulse > 0.0f )
							{
								SetBit( ref hitEventBitSet, contactSim.contactId );
								hasHitEvents = true;
								break;
							}
						}
					}
				}
			}

			// Advance to next color
			colorIndex += 1;
		}

		taskContext.hasHitEvents = hasHitEvents;
	}

}
