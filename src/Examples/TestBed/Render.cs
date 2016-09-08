/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

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

using System;
using System.Collections.Generic;
using System.Text;

using ISE;

using Box2DNet.Common;
using Box2DNet.Collision;
using Box2DNet.Dynamics;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;
namespace TestBed
{
    using SharpFont;
    using Box2DXMath = Box2DNet.Common.Math;
    using SysMath = System.Math;

    // This class implements debug drawing callbacks that are invoked
    // inside World.Step.
    public class OpenGLDebugDraw : DebugDraw
	{
		public override void DrawPolygon(Vec2[] vertices, int vertexCount, Color color)
		{
			GL.Color3(color.R, color.G, color.B);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < vertexCount; ++i)
			{
				GL.Vertex2(vertices[i].X, vertices[i].Y);
			}
			GL.End();			
		}

		public override void DrawSolidPolygon(Vec2[] vertices, int vertexCount, Color color)
		{
			GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
			GL.Color4(0.5f * color.R, 0.5f * color.G, 0.5f * color.B, 0.5f);
			GL.Begin(BeginMode.TriangleFan);
			for (int i = 0; i < vertexCount; ++i)
			{
				GL.Vertex2(vertices[i].X, vertices[i].Y);
			}
			GL.End();
			GL.Disable(EnableCap.Blend);

			GL.Color4(color.R, color.G, color.B, 1.0f);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < vertexCount; ++i)
			{
				GL.Vertex2(vertices[i].X, vertices[i].Y);
			}
			GL.End();
		}

		public override void DrawCircle(Vec2 center, float radius, Color color)
		{
			float k_segments = 16.0f;
			float k_increment = 2.0f * Box2DNet.Common.Settings.Pi / k_segments;
			float theta = 0.0f;
			GL.Color3(color.R, color.G, color.B);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < k_segments; ++i)
			{
				Vec2 v = center + radius * new Vec2((float)SysMath.Cos(theta), (float)SysMath.Sin(theta));
				GL.Vertex2(v.X, v.Y);
				theta += k_increment;
			}
			GL.End();
		}

		public override void DrawSolidCircle(Vec2 center, float radius, Vec2 axis, Color color)
		{
			float k_segments = 16.0f;
			float k_increment = 2.0f * Box2DNet.Common.Settings.Pi / k_segments;
			float theta = 0.0f;
			GL.Enable(EnableCap.Blend);
			GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
			GL.Color4(0.5f * color.R, 0.5f * color.G, 0.5f * color.B, 0.5f);
			GL.Begin(BeginMode.TriangleFan);
			for (int i = 0; i < k_segments; ++i)
			{
				Vec2 v = center + radius * new Vec2((float)SysMath.Cos(theta), (float)SysMath.Sin(theta));
				GL.Vertex2(v.X, v.Y);
				theta += k_increment;
			}
			GL.End();
			GL.Disable(EnableCap.Blend);

			theta = 0.0f;
			GL.Color4(color.R, color.G, color.B, 1.0f);
			GL.Begin(BeginMode.LineLoop);
			for (int i = 0; i < k_segments; ++i)
			{
				Vec2 v = center + radius * new Vec2((float)SysMath.Cos(theta), (float)SysMath.Sin(theta));
				GL.Vertex2(v.X, v.Y);
				theta += k_increment;
			}
			GL.End();

			Vec2 p = center + radius * axis;
			GL.Begin(BeginMode.Lines);
			GL.Vertex2(center.X, center.Y);
			GL.Vertex2(p.X, p.Y);
			GL.End();
		}

		public override void DrawSegment(Vec2 p1, Vec2 p2, Color color)
		{
			GL.Color3(color.R, color.G, color.B);
			GL.Begin(BeginMode.Lines);
			GL.Vertex2(p1.X, p1.Y);
			GL.Vertex2(p2.X, p2.Y);
			GL.End();
		}

		public override void DrawXForm(XForm xf)
		{
			Vec2 p1 = xf.Position, p2;
			float k_axisScale = 0.4f;
			GL.Begin(BeginMode.Lines);

			GL.Color3(1.0f, 0.0f, 0.0f);
			GL.Vertex2(p1.X, p1.Y);
			p2 = p1 + k_axisScale * xf.R.Col1;
			GL.Vertex2(p2.X, p2.Y);

			GL.Color3(0.0f, 1.0f, 0.0f);
			GL.Vertex2(p1.X, p1.Y);
			p2 = p1 + k_axisScale * xf.R.Col2;
			GL.Vertex2(p2.X, p2.Y);

			GL.End();
		}

		public static void DrawSegment(Vec2 p1, Vec2 p2, Color color, params object[] p)
		{
			GL.Color3(color.R, color.G, color.B);
			GL.Begin(BeginMode.Lines);
			GL.Vertex2(p1.X, p1.Y);
			GL.Vertex2(p2.X, p2.Y);
			GL.End();
		}

		public static void DrawPoint(Vec2 p, float size, Color color)
		{
			GL.PointSize(size);
			GL.Begin(BeginMode.Points);
			GL.Color3(color.R, color.G, color.B);
			GL.Vertex2(p.X, p.Y);
			GL.End();
			GL.PointSize(1.0f);
		}

		static FTFont sysfont;
		static GLControl openGlControl;
		private static bool sIsTextRendererInitialized = false;
		public static void InitTextRenderer(GLControl openGlCtrl)
		{
			openGlControl = openGlCtrl;

			try
			{
				int Errors = 0;
				// CREATE FONT
				sysfont = new FTFont("FreeSans.ttf", out Errors);
  
                // INITIALISE FONT AS A PER_CHARACTER TEXTURE MAPPED FONT
                sysfont.ftRenderToTexture(12, 196);

                // SET the sample font to align CENTERED
                sysfont.FT_ALIGN = FTFontAlign.FT_ALIGN_LEFT;
				sIsTextRendererInitialized = true;
			}
			catch (Exception)
			{
				sIsTextRendererInitialized = false;
			}
		}

		public static void DrawString(int x, int y, string str)
		{
			if (sIsTextRendererInitialized)
			{
				GL.MatrixMode(MatrixMode.Projection);
				GL.PushMatrix();
				GL.LoadIdentity();

				GL.MatrixMode(MatrixMode.Modelview);
				GL.PushMatrix();
				GL.LoadIdentity();

				float xOffset = -0.95f + (float)x / ((float)openGlControl.Width / 2f);
				float yOffset = 0.95f - (float)y / ((float)openGlControl.Height / 2f);
				// Offset the font on the screen
				GL.Translate(xOffset, yOffset, 0);

				GL.Color3(0.9f, 0.6f, 0.6f);
				// Scale the font
				GL.Scale(0.0035f, 0.0035f, 0.0035f);

				// Begin writing the font
				sysfont.ftBeginFont();
				sysfont.ftWrite(str);
				// Stop writing the font and restore old OpenGL parameters
				sysfont.ftEndFont();

				GL.PopMatrix();
				GL.MatrixMode(MatrixMode.Projection);
				GL.PopMatrix();
				GL.MatrixMode(MatrixMode.Modelview);
			}
		}

		public static void DrawAABB(AABB aabb, Color c)
		{
			GL.Color3(c.R, c.G, c.B);
			GL.Begin(BeginMode.LineLoop);
			GL.Vertex2(aabb.LowerBound.X, aabb.LowerBound.Y);
			GL.Vertex2(aabb.UpperBound.X, aabb.LowerBound.Y);
			GL.Vertex2(aabb.UpperBound.X, aabb.UpperBound.Y);
			GL.Vertex2(aabb.LowerBound.X, aabb.UpperBound.Y);
			GL.End();
		}
	}
}
