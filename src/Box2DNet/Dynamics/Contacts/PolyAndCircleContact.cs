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

using Box2DNet.Collision;
using Box2DNet.Common;

using Transform = Box2DNet.Common.Transform;

namespace Box2DNet.Dynamics
{
	public class PolyAndCircleContact : Contact
	{
		public PolyAndCircleContact(Fixture fixtureA, Fixture fixtureB)
			: base(fixtureA, fixtureB)
		{
			Box2DNetDebug.Assert(fixtureA.ShapeType == ShapeType.PolygonShape);
			Box2DNetDebug.Assert(fixtureB.ShapeType == ShapeType.CircleShape);
			CollideShapeFunction = CollidePolygonCircle;
		}

		private static void CollidePolygonCircle(ref Manifold manifold, Shape shape1, Transform xf1, Shape shape2, Transform xf2)
		{
			Collision.Collision.CollidePolygonAndCircle(ref manifold, (PolygonShape)shape1, xf1, (CircleShape)shape2, xf2);
		}

		new public static Contact Create(Fixture fixtureA, Fixture fixtureB)
		{
			return new PolyAndCircleContact(fixtureA, fixtureB);
		}

		new public static void Destroy(ref Contact contact)
		{
			contact = null;
		}
	}
}
