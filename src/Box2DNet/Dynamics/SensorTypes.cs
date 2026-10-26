// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	// Used to track shapes that hit sensors using time of impact
	internal struct SensorHit
	{
		public int sensorId;
		public int visitorId;
	}

	internal struct Visitor
	{
		public int shapeId;
		public ushort generation;
	}

	internal struct Sensor
	{
		// todo find a way to pool these
		public B2Array<Visitor> hits;
		public B2Array<Visitor> overlaps1;
		public B2Array<Visitor> overlaps2;
		public int shapeId;
	}

	internal struct SensorTaskContext
	{
		public BitSet eventBits;
	}



}
