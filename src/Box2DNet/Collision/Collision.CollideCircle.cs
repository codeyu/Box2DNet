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
using System.Numerics;
namespace Box2DNet.Collision
{
	public partial class Collision
	{
		public static void CollideCircles(ref Manifold manifold, CircleShape circle1, Transform xf1, CircleShape circle2, Transform xf2)
		{
			manifold.PointCount = 0;

			Vector2 p1 = xf1.TransformPoint(circle1._position);
			Vector2 p2 = xf2.TransformPoint(circle2._position);

			Vector2 d = p2 - p1;
			float distSqr = Vector2.Dot(d, d);
			float radius = circle1._radius + circle2._radius;
			if (distSqr > radius * radius)
			{
				return;
			}

			manifold.Type = ManifoldType.Circles;
			manifold.LocalPoint = circle1._position;
			manifold.LocalPlaneNormal = Vector2.Zero;
			manifold.PointCount = 1;

			manifold.Points[0].LocalPoint = circle2._position;
			manifold.Points[0].ID.Key = 0;
		}

		public static void CollidePolygonAndCircle(ref Manifold manifold, PolygonShape polygon, Transform xf1, CircleShape circle, Transform xf2)
		{
			manifold.PointCount = 0;

			// Compute circle position in the frame of the polygon.
			Vector2 c = xf2.TransformPoint(circle._position);
			Vector2 cLocal = xf1.InverseTransformPoint(c);

			// Find the min separating edge.
			int normalIndex = 0;
			float separation = -Settings.FLT_MAX;
			float radius = polygon._radius + circle._radius;
			int vertexCount = polygon._vertexCount;
			Vector2[] vertices = polygon._vertices;
			Vector2[] normals = polygon._normals;

			for (int i = 0; i < vertexCount; ++i)
			{
				float s = Vector2.Dot(normals[i], cLocal - vertices[i]);
				if (s > radius)
				{
					// Early out.
					return;
				}

				if (s > separation)
				{
					separation = s;
					normalIndex = i;
				}
			}

			// Vertices that subtend the incident face.
			int vertIndex1 = normalIndex;
			int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
			Vector2 v1 = vertices[vertIndex1];
			Vector2 v2 = vertices[vertIndex2];

			// If the center is inside the polygon ...
			if (separation < Common.Settings.FLT_EPSILON)
			{
				manifold.PointCount = 1;
				manifold.Type = ManifoldType.FaceA;
				manifold.LocalPlaneNormal = normals[normalIndex];
				manifold.LocalPoint = 0.5f * (v1 + v2);
				manifold.Points[0].LocalPoint = circle._position;
				manifold.Points[0].ID.Key = 0;
				return;
			}

			// Compute barycentric coordinates
			float u1 = Vector2.Dot(cLocal - v1, v2 - v1);
			float u2 = Vector2.Dot(cLocal - v2, v1 - v2);
			if (u1 <= 0.0f)
			{
				if ((cLocal - v1).LengthSquared() > radius * radius)
				{
					return;
				}

				manifold.PointCount = 1;
				manifold.Type = ManifoldType.FaceA;
				manifold.LocalPlaneNormal = cLocal - v1;
				manifold.LocalPlaneNormal.Normalize();
				manifold.LocalPoint = v1;
				manifold.Points[0].LocalPoint = circle._position;
				manifold.Points[0].ID.Key = 0;
			}
			else if (u2 <= 0.0f)
			{
				if ((cLocal - v2).LengthSquared() > radius * radius)
				{
					return;
				}

				manifold.PointCount = 1;
				manifold.Type = ManifoldType.FaceA;
				manifold.LocalPlaneNormal = cLocal - v2;
				manifold.LocalPlaneNormal.Normalize();
				manifold.LocalPoint = v2;
				manifold.Points[0].LocalPoint = circle._position;
				manifold.Points[0].ID.Key = 0;
			}
			else
			{
				Vector2 faceCenter = 0.5f * (v1 + v2);
				float separation_ = Vector2.Dot(cLocal - faceCenter, normals[vertIndex1]);
				if (separation_ > radius)
				{
					return;
				}

				manifold.PointCount = 1;
				manifold.Type = ManifoldType.FaceA;
				manifold.LocalPlaneNormal = normals[vertIndex1];
				manifold.LocalPoint = faceCenter;
				manifold.Points[0].LocalPoint = circle._position;
				manifold.Points[0].ID.Key = 0;
			}
		}
	}
}