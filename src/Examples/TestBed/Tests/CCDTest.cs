/*
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
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
using Box2DNet.Dynamics;

namespace TestBed
{
	public class CCDTest : Test
	{
		public CCDTest()
		{
			{
				PolygonDef sd = new PolygonDef();
				sd.SetAsBox(10.0f, 0.2f);
				sd.Density = 0.0f;

				BodyDef bd = new BodyDef();
				bd.Position.Set(0.0f, -0.2f);
				Body body = _world.CreateBody(bd);
				body.CreateFixture(sd);

				sd.SetAsBox(0.2f, 1.0f, new Vec2(0.5f, 1.2f), 0.0f);
				body.CreateFixture(sd);
			}
			{
				PolygonDef sd = new PolygonDef();
				sd.SetAsBox(2.0f, 0.1f);
				sd.Density = 1.0f;
				sd.Restitution = 0;

				BodyDef bd = new BodyDef();
				bd.Position.Set(0.0f, 20.0f);
				Body body = _world.CreateBody(bd);
				body.CreateFixture(sd);
				body.SetMassFromShapes();
				body.SetLinearVelocity(new Vec2(0.0f, -100.0f));
				body.SetAngularVelocity(Box2DNet.Common.Math.Random(-50.0f, 50.0f));
			}
		}

		public static Test Create()
		{
			return new CCDTest();
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);
			OpenGLDebugDraw.DrawString(5, _textLine, "Max toi iters = " + Box2DNet.Collision.Collision.MaxToiIters + 
				", max root iters = " + Box2DNet.Collision.Collision.MaxToiRootIters);
		}
	}
}