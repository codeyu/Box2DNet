﻿/*
  Box2DNet Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using Box2DNet.Common;
using System.Numerics;

using Transform = Box2DNet.Common.Transform;

namespace Box2DNet.Collision
{
	/// <summary>
	/// A circle shape.
	/// </summary>
	public class CircleShape : Shape
	{
		// Position
		internal Vector2 _position;

		public CircleShape()			
		{
			_type = ShapeType.CircleShape;
		}

		public override bool TestPoint(Transform xf, Vector2 p)
		{
			Vector2 center = xf.position + xf.TransformDirection(_position);
			Vector2 d = p - center;
			return Vector2.Dot(d, d) <= _radius * _radius;
		}

		// Collision Detection in Interactive 3D Environments by Gino van den Bergen
		// From Section 3.1.2
		// x = s + a * r
		// norm(x) = radius
		public override SegmentCollide TestSegment(Transform xf, out float lambda, out Vector2 normal, Segment segment, float maxLambda)
		{
			lambda = 0f;
			normal = Vector2.Zero;

			Vector2 position = xf.position + xf.TransformDirection(_position);
			Vector2 s = segment.P1 - position;
			float b = Vector2.Dot(s, s) - _radius * _radius;

			// Does the segment start inside the circle?
			if (b < 0.0f)
			{
				lambda = 0f;
				return SegmentCollide.StartInsideCollide;
			}

			// Solve quadratic equation.
			Vector2 r = segment.P2 - segment.P1;
			float c = Vector2.Dot(s, r);
			float rr = Vector2.Dot(r, r);
			float sigma = c * c - rr * b;

			// Check for negative discriminant and short segment.
			if (sigma < 0.0f || rr < Common.Settings.FLT_EPSILON)
			{
				return SegmentCollide.MissCollide;
			}

			// Find the point of intersection of the line with the circle.
			float a = -(c + Common.Math.Sqrt(sigma));

			// Is the intersection point on the segment?
			if (0.0f <= a && a <= maxLambda * rr)
			{
				a /= rr;
				lambda = a;
				normal = s + a * r;
				normal.Normalize();
				return SegmentCollide.HitCollide;
			}

			return SegmentCollide.MissCollide;
		}

		public override void ComputeAABB(out AABB aabb, Transform xf)
		{
			aabb = new AABB();

			Vector2 p = xf.position + xf.TransformDirection(_position);
			aabb.LowerBound = new Vector2(p.X - _radius, p.Y - _radius);
			aabb.UpperBound = new Vector2(p.X + _radius, p.Y + _radius);
		}

		public override void ComputeMass(out MassData massData, float density)
		{
			massData = new MassData();

			massData.Mass = density * (float)System.Math.PI * _radius * _radius;
			massData.Center = _position;

			// inertia about the local origin
			massData.I = massData.Mass * (0.5f * _radius * _radius + Vector2.Dot(_position, _position));
		}		

		public override float ComputeSubmergedArea(Vector2 normal, float offset, Transform xf, out Vector2 c)
		{
			Vector2 p = xf.TransformPoint(_position);
			float l = -(Vector2.Dot(normal, p) - offset);
			if (l < -_radius + Box2DNet.Common.Settings.FLT_EPSILON)
			{
				//Completely dry
				c = new Vector2();
				return 0;
			}
			if (l > _radius)
			{
				//Completely wet
				c = p;
				return Box2DNet.Common.Settings.Pi * _radius * _radius;
			}

			//Magic
			float r2 = _radius * _radius;
			float l2 = l * l;
			float area = r2 * ((float)System.Math.Asin(l / _radius) + Box2DNet.Common.Settings.Pi / 2) +
				l * Box2DNet.Common.Math.Sqrt(r2 - l2);
			float com = -2.0f / 3.0f * (float)System.Math.Pow(r2 - l2, 1.5f) / area;

			c.X = p.X + normal.X * com;
			c.Y = p.Y + normal.Y * com;

			return area;
		}

		/// <summary>
		/// Get the supporting vertex index in the given direction.
		/// </summary>
		public override int GetSupport(Vector2 d)
		{
			return 0;
		}

		/// <summary>
		/// Get the supporting vertex in the given direction.
		/// </summary>
		public override Vector2 GetSupportVertex(Vector2 d)
		{
			return _position;
		}

		/// <summary>
		/// Get a vertex by index. Used by Distance.
		/// </summary>
		public override Vector2 GetVertex(int index)
		{
			Box2DNetDebug.Assert(index == 0);
			return _position;
		}

		public override float ComputeSweepRadius(Vector2 pivot)
		{
			return Box2DNet.Common.Math.Distance(_position, pivot);
		}

		/// <summary>
		/// Get the vertex count.
		/// </summary>
		public int VertexCount { get { return 1; } }
	}
}