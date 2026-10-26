// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.
	// #define B2_PROXY_TYPE( KEY ) ( (BodyType)( ( KEY ) & 3 ) )
	// #define B2_PROXY_ID( KEY ) ( ( KEY ) >> 2 )
	// #define B2_PROXY_KEY( ID, TYPE ) ( ( ( ID ) << 2 ) | ( TYPE ) )

	/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
	/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
	/// It is up to the client to consume the new pairs and to track subsequent overlap.
	internal sealed class BroadPhase
	{
		// One tree for each body type.
		public FixedArray3<DynamicTree> trees;

		public BroadPhase()
		{
			for ( int i = 0; i < 3; ++i )
			{
				trees[i] = new DynamicTree();
			}
		}

		// The moved siblings gathered from the dynamic body tree.
		public /*TODO pointer*/ int[] movedSiblings;

		// Tracks shape pairs that have a Contact
		public HashSet64 pairSet;
	}







	internal static void BroadPhase_MarkProxyMovedSerial( BroadPhase bp, int proxyKey )
	{
		BodyType proxyType = B2_PROXY_TYPE( proxyKey );
		int proxyId = B2_PROXY_ID( proxyKey );
		DynamicTree_MarkProxyMovedSerial( bp.trees[proxyType], proxyId );
	}

	internal static void BroadPhase_MarkProxyMoved( BroadPhase bp, int proxyKey, AABB aabb )
	{
		BodyType proxyType = B2_PROXY_TYPE( proxyKey );
		int proxyId = B2_PROXY_ID( proxyKey );
		DynamicTree_MarkProxyMoved( bp.trees[proxyType], proxyId, aabb );
	}

}
