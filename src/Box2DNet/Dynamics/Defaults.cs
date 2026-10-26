// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	public static WorldDef DefaultWorldDef()
	{
		float lengthUnits = GetLengthUnitsPerMeter();
		WorldDef def = default;
		def.gravity.x = 0.0f;
		def.gravity.y = -10.0f;
		def.hitEventThreshold = 1.0f * lengthUnits;
		def.restitutionThreshold = 1.0f * lengthUnits;
		def.restitutionIterations = 2;
		def.enableRestitutionPropagation = false;
		def.contactSpeed = 3.0f * lengthUnits;
		def.contactHertz = 30.0f;
		def.contactDampingRatio = 10.0f;

		// 400 meters per second, faster than the speed of sound
		def.maximumLinearSpeed = 400.0f * lengthUnits;
		def.enableSleep = true;
		def.enableContinuous = true;
		def.internalValue = SecretCookie;
		return def;
	}

	public static BodyDef DefaultBodyDef()
	{
		BodyDef def = default;
		def.type = BodyType.staticBody;
		def.rotation = Rot_identity;
		def.safetyFactor = 0.5f;
		def.sleepThreshold = 0.05f * GetLengthUnitsPerMeter();
		def.gravityScale = 1.0f;
		def.enableSleep = true;
		def.enableContactRecycling = true;
		def.isAwake = true;
		def.isEnabled = true;
		def.internalValue = SecretCookie;
		return def;
	}

	public static Filter DefaultFilter()
	{
		Filter filter = new Filter { categoryBits = DefaultCategoryBits, maskBits = DefaultMaskBits, groupIndex = 0 };
		return filter;
	}

	public static QueryFilter DefaultQueryFilter()
	{
		QueryFilter filter = new QueryFilter { categoryBits = DefaultCategoryBits, maskBits = DefaultMaskBits };
		return filter;
	}

	public static ShapeDef DefaultShapeDef()
	{
		ShapeDef def = default;
		def.material.friction = 0.6f;
		def.density = 1.0f;
		def.filter = DefaultFilter();
		def.updateBodyMass = true;
		def.invokeContactCreation = true;
		def.internalValue = SecretCookie;
		return def;
	}

	public static SurfaceMaterial DefaultSurfaceMaterial()
	{
		SurfaceMaterial material = new SurfaceMaterial { friction = 0.6f };

		return material;
	}

	public static ChainDef DefaultChainDef()
	{
		ChainDef def = default;
		def.materials = new SurfaceMaterial[] { new SurfaceMaterial { friction = 0.6f } };
		def.materialCount = 1;
		// using infinity as a sentinel
		def.ghost1.x = float.PositiveInfinity;
		def.ghost1.y = float.PositiveInfinity;
		def.ghost2.x = float.PositiveInfinity;
		def.ghost2.y = float.PositiveInfinity;
		def.filter = DefaultFilter();
		def.internalValue = SecretCookie;
		return def;
	}

	private static void EmptyDrawPolygon( WorldTransform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, HexColor color, object? context )
	{
	}

	private static void EmptyDrawSolidPolygon( WorldTransform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, float radius, HexColor color, object? context )
	{
	}

	private static void EmptyDrawCircle( Pos center, float radius, HexColor color, object? context )
	{
	}

	private static void EmptyDrawSolidCircle( WorldTransform transform, Vec2 center, float radius, HexColor color, object? context )
	{
	}

	private static void EmptyDrawSolidCapsule( Pos p1, Pos p2, float radius, HexColor color, object? context )
	{
	}

	private static void EmptyDrawSegment( Pos p1, Pos p2, HexColor color, object? context )
	{
	}

	private static void EmptyDrawTransform( WorldTransform transform, object? context )
	{
	}

	private static void EmptyDrawPoint( Pos p, float size, HexColor color, object? context )
	{
	}

	private static void EmptyDrawString( Pos p, string s, HexColor color, object? context )
	{
	}

	private static void EmptyDrawBounds( AABB aabb, HexColor color, object? context )
	{
	}

	public static DebugDraw DefaultDebugDraw()
	{
		DebugDraw draw = default;

		// These allow the user to skip some implementations and not hit null exceptions.
		draw.DrawPolygonFcn = EmptyDrawPolygon;
		draw.DrawSolidPolygonFcn = EmptyDrawSolidPolygon;
		draw.DrawCircleFcn = EmptyDrawCircle;
		draw.DrawSolidCircleFcn = EmptyDrawSolidCircle;
		draw.DrawSolidCapsuleFcn = EmptyDrawSolidCapsule;
		draw.DrawLineFcn = EmptyDrawSegment;
		draw.DrawTransformFcn = EmptyDrawTransform;
		draw.DrawPointFcn = EmptyDrawPoint;
		draw.DrawStringFcn = EmptyDrawString;
		draw.DrawBoundsFcn = EmptyDrawBounds;

		draw.drawingBounds.lowerBound = new Vec2( -FLT_MAX, -FLT_MAX );
		draw.drawingBounds.upperBound = new Vec2( FLT_MAX, FLT_MAX );
		draw.forceScale = 1.0f;
		draw.jointScale = 1.0f;
		draw.drawShapes = true;

		return draw;
	}

}
