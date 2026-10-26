// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System.Numerics;

namespace Box2DNet.Testbed;

// 2D camera with world units in meters. The y-axis points up in the world and down on the screen.
public sealed class Camera
{
	public Vec2 center = new( 0.0f, 20.0f );
	public float zoom = 1.0f;
	public float width = 1280.0f;
	public float height = 800.0f;

	// world units visible vertically at zoom 1
	private const float k_viewHeight = 50.0f;

	public float PixelsPerMeter => height / ( zoom * k_viewHeight );

	public Vector2 ToScreen( Vec2 p )
	{
		float s = PixelsPerMeter;
		return new Vector2( 0.5f * width + s * ( p.x - center.x ), 0.5f * height - s * ( p.y - center.y ) );
	}

	public Vec2 ToWorld( Vector2 p )
	{
		float s = PixelsPerMeter;
		return new Vec2( center.x + ( p.X - 0.5f * width ) / s, center.y - ( p.Y - 0.5f * height ) / s );
	}

	public AABB GetViewBounds()
	{
		Vec2 lower = ToWorld( new Vector2( 0.0f, height ) );
		Vec2 upper = ToWorld( new Vector2( width, 0.0f ) );
		return new AABB( lower, upper );
	}
}
