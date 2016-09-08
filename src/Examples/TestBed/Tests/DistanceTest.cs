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

using System.Text;

using Box2DNet.Common;
using Box2DNet.Collision;
using Box2DNet.Dynamics;
using Math=Box2DNet.Common.Math;

namespace TestBed
{
	public class DistanceTest : Test
	{
		Vec2 _positionB;
		float _angleB;

		XForm _transformA;
		XForm _transformB;
		PolygonShape _polygonA;
		PolygonShape _polygonB;

		Vec2[] _dv = new Vec2[Box2DNet.Common.Settings.MaxPolygonVertices];

		public DistanceTest()
		{
			{
				_polygonA = new PolygonShape();
				_transformA.SetIdentity();
				_transformA.Position.Set(0.0f, -0.2f);
				_polygonA.SetAsBox(10.0f, 0.2f);
			}

			{
				_polygonB = new PolygonShape();
				_positionB.Set(12.017401f, 0.13678508f);
				_angleB = -0.0109265f;
				_transformB.Set(_positionB, _angleB);
				_polygonB.SetAsBox(2.0f, 0.1f);
			}
		}

		public override void Step(Settings settings)
		{
			base.Step(settings);

			DistanceInput input = new DistanceInput();
			input.TransformA = _transformA;
			input.TransformB = _transformB;
			input.UseRadii = true;
			SimplexCache cache = new SimplexCache();
			cache.Count = 0;
			DistanceOutput output;
			Collision.Distance(out output, ref cache, ref input, _polygonA, _polygonB);

			StringBuilder strBld = new StringBuilder();
			strBld.AppendFormat("distance = {0}", new object[] { output.Distance });
			OpenGLDebugDraw.DrawString(5, _textLine, strBld.ToString());
			_textLine += 15;

			strBld = new StringBuilder();
			strBld.AppendFormat("iterations = {0}", new object[] { output.Iterations });
			OpenGLDebugDraw.DrawString(5, _textLine, strBld.ToString());
			_textLine += 15;

			{
				Color color = new Color(0.9f, 0.9f, 0.9f);
				int i;
				for (i = 0; i < _polygonA.VertexCount; ++i)
				{
					_dv[i] = Math.Mul(_transformA, _polygonA.Vertices[i]);
				}
				_debugDraw.DrawPolygon(_dv, _polygonA.VertexCount, color);

				for (i = 0; i < _polygonB.VertexCount; ++i)
				{
					_dv[i] = Math.Mul(_transformB, _polygonB.Vertices[i]);
				}
				_debugDraw.DrawPolygon(_dv, _polygonB.VertexCount, color);
			}

			Vec2 x1 = output.PointA;
			Vec2 x2 = output.PointB;

			OpenGLDebugDraw.DrawPoint(x1, 4.0f, new Color(1, 0, 0));
			OpenGLDebugDraw.DrawSegment(x1, x2, new Color(1, 1, 0));
			OpenGLDebugDraw.DrawPoint(x2, 4.0f, new Color(1, 0, 0));
		}

		public override void Keyboard(System.Windows.Forms.Keys key)
		{
			switch (key)
			{
				case  System.Windows.Forms.Keys.A:
					_positionB.X -= 0.1f;
					break;

				case System.Windows.Forms.Keys.D:
					_positionB.X += 0.1f;
					break;

				case System.Windows.Forms.Keys.S:
					_positionB.Y -= 0.1f;
					break;

				case System.Windows.Forms.Keys.W:
					_positionB.Y += 0.1f;
					break;

				case System.Windows.Forms.Keys.Q:
					_angleB += 0.1f * Box2DNet.Common.Settings.Pi;
					break;

				case System.Windows.Forms.Keys.E:
					_angleB -= 0.1f * Box2DNet.Common.Settings.Pi;
					break;
			}

			_transformB.Set(_positionB, _angleB);
		}

		public static Test Create()
		{
			return new DistanceTest();
		}
	}
}