/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx

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

#define GLRender

using System;
using System.Windows.Forms;
using OpenTK.Graphics.OpenGL;
using Box2DNet.Common;

namespace TestBed
{
	public partial class MainForm : Form
	{
	    public Test CurrentTest { get; set; }

	    private readonly Settings _settings = new Settings();
		private float _viewZoom = 1f;
		private Vec2 _viewCenter = new Vec2(0.0f, 20.0f);
		private TestEntry _testEntry;
		private bool _rMouseDown = false;
		private Vec2 _lastp;		

		public MainForm()
		{
		    InitializeComponent();
		}		

		private void MainForm_Load(object sender, EventArgs e)
		{
#if GLRender
            openGlControl_Resize(this, EventArgs.Empty);
            OpenGLDebugDraw.InitTextRenderer(openGlControl);
#endif //GLRender

            Init();
            SetView();


			chkbAabbs.Checked = _settings.drawAABBs == 1;
			chkbCF.Checked = _settings.drawContactForces == 1;
			chkbCN.Checked = _settings.drawContactNormals == 1;
			chkbCom.Checked = _settings.drawCOMs == 1;
			chkbCoreShapes.Checked = _settings.drawCoreShapes == 1;
			chkbCP.Checked = _settings.drawContactPoints == 1;
			chkbFF.Checked = _settings.drawFrictionForces == 1;
			chkbJoints.Checked = _settings.drawJoints == 1 ? true : false;
			chkbObbs.Checked = _settings.drawOBBs == 1 ? true : false;
			chkbPairs.Checked = _settings.drawPairs == 1 ? true : false;
			chkbShapes.Checked = _settings.drawShapes == 1 ? true : false;
			chkbStatistics.Checked = _settings.drawStats == 1 ? true : false;

			chkbToi.Checked = _settings.enableTOI == 1 ? true : false;
			chkbWarmStart.Checked = _settings.enableWarmStarting == 1 ? true : false;

			nudVelIters.Value = _settings.velocityIterations;
			nudPosIters.Value = _settings.positionIterations;
			nudHz.Value = (decimal)_settings.hz;

			foreach (TestEntry t in Test.g_testEntries)
			{
			    cmbbTests.Items.Add(t);
			}

		    _testEntry = Test.g_testEntries[0];
			CurrentTest = _testEntry.CreateFcn();
			cmbbTests.SelectedIndex = 0;

			redrawTimer.Interval = 16;
			redrawTimer.Enabled = true;
		}

        //protected override void OnResize(EventArgs e)
        //{
        //    base.OnResize(e);
        //    SetView();
        //}

		#region Timer

		private void redrawTimer_Tick(object sender, EventArgs e)
		{
			SimulationLoop();
		}

		#endregion Timer

		#region Input Handlers

		private void openGlControl_MouseMove(object sender, MouseEventArgs e)
		{
			Vec2 p = ConvertScreenToWorld(e.X, e.Y);
			CurrentTest.MouseMove(p);

			if (_rMouseDown)
			{
				Vec2 diff = p - _lastp;
				_viewCenter.X -= diff.X;
				_viewCenter.Y -= diff.Y;
				SetView();
				_lastp = ConvertScreenToWorld(e.X, e.Y);
			}
		}

		protected override void OnMouseWheel(MouseEventArgs e)
		{
			if (e.Location.X > openGlControl.Location.X && e.Location.X < openGlControl.Location.X + openGlControl.Width &&
				e.Location.Y > openGlControl.Location.Y && e.Location.Y < openGlControl.Location.Y + openGlControl.Height)
			{
				if (e.Delta > 0)
				{
					_viewZoom /= 1.1f;
				}
				else
				{
					_viewZoom *= 1.1f;
				}
				SetView();
			}
		}

		private void openGlControl_MouseUp(object sender, MouseEventArgs e)
		{
			if(e.Button == MouseButtons.Left)
				CurrentTest.MouseUp();
			else if (e.Button == MouseButtons.Right)
				_rMouseDown = false;
		}

		private void openGlControl_MouseDown(object sender, MouseEventArgs e)
		{
			if (e.Button == MouseButtons.Left)
				CurrentTest.MouseDown(ConvertScreenToWorld(e.X, e.Y));
			else if (e.Button == MouseButtons.Right)
			{
				_lastp = ConvertScreenToWorld(e.X, e.Y);
				_rMouseDown = true;
			}
		}

		private void openGlControl_PreviewKeyDown(object sender, PreviewKeyDownEventArgs e)
		{
			switch (e.KeyCode)
			{
				case Keys.Escape:
					this.Close();
					break;
				case Keys.Z:
					_viewZoom = Box2DNet.Common.Math.Min(1.1f * _viewZoom, 20.0f);
					SetView();
					break;
				case Keys.X:
					_viewZoom = Box2DNet.Common.Math.Max(0.9f * _viewZoom, 0.02f);
					SetView();
					break;
				case Keys.R:
					CurrentTest = _testEntry.CreateFcn();
					break;
				case Keys.Space:
					CurrentTest.LaunchBomb();
					break;
				case Keys.Left:
					_viewCenter.X -= 0.5f;
					SetView();
					break;
				case Keys.Right:
					_viewCenter.X += 0.5f;
					SetView();
					break;
				case Keys.Down:
					_viewCenter.Y -= 0.5f;
					SetView();
					break;
				case Keys.Up:
					_viewCenter.Y += 0.5f;
					SetView();
					break;
				case Keys.Home:
					_viewZoom = 1.0f;
					_viewCenter.Set(0.0f, 20.0f);
					SetView();
					break;
				default:
					CurrentTest.Keyboard(e.KeyCode);
					break;
			}
		}

		#endregion Input Handlers

		#region Controls Events Handlers

		private void nudHz_ValueChanged(object sender, EventArgs e)
		{
			_settings.hz = (float)nudHz.Value;
		}

		private void btnQuit_Click(object sender, EventArgs e)
		{
			this.Close();
		}

		private void btnSingleStep_Click(object sender, EventArgs e)
		{
			_settings.singleStep = 1;
			_settings.pause = 1;
		}

		private void btnPause_Click(object sender, EventArgs e)
		{
			_settings.pause = _settings.pause == 0 ? 1 : 0;
		}

		private void chkbWarmStart_CheckedChanged(object sender, EventArgs e)
		{
			_settings.enableWarmStarting = chkbWarmStart.Checked ? 1 : 0;
		}

		private void chkbToi_CheckedChanged(object sender, EventArgs e)
		{
			_settings.enableTOI = chkbToi.Checked ? 1 : 0;
		}

		private void chkbShapes_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawShapes = chkbShapes.Checked ? 1 : 0;
		}

		private void chkbJoints_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawJoints = chkbJoints.Checked ? 1 : 0;
		}

		private void chkbCoreShapes_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawCoreShapes = chkbCoreShapes.Checked ? 1 : 0;
		}

		private void chkbAabbs_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawAABBs = chkbAabbs.Checked ? 1 : 0;
		}

		private void chkbObbs_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawOBBs = chkbObbs.Checked ? 1 : 0;
		}

		private void chkbPairs_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawPairs = chkbPairs.Checked ? 1 : 0;
		}

		private void chkbCN_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawContactNormals = chkbCN.Checked ? 1 : 0;
		}

		private void chkbCF_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawContactForces = chkbCF.Checked ? 1 : 0;
		}

		private void chkbFF_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawFrictionForces = chkbFF.Checked ? 1 : 0;
		}

		private void chkbCom_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawCOMs = chkbCom.Checked ? 1 : 0;
		}

		private void chkbStatistics_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawStats = chkbStatistics.Checked ? 1 : 0;
		}

		private void chkbCP_CheckedChanged(object sender, EventArgs e)
		{
			_settings.drawContactPoints = chkbCP.Checked ? 1 : 0;
		}

		private void cmbbTests_SelectedIndexChanged(object sender, EventArgs e)
		{
			_testEntry = cmbbTests.SelectedItem as TestEntry;
			CurrentTest = _testEntry.CreateFcn();
			this.Text = "Box2DX " + Application.ProductVersion + " - " + _testEntry.ToString();
		}

		private void nudVelIters_ValueChanged(object sender, EventArgs e)
		{
			_settings.velocityIterations = (int)nudVelIters.Value;
		}

		private void nudPosIters_ValueChanged(object sender, EventArgs e)
		{
			_settings.positionIterations = (int)nudPosIters.Value;
		}

		#endregion Controls Events Handlers

		#region Render

		private void Init()
		{

#if GLRender
			GL.ShadeModel(ShadingModel.Smooth);
			GL.ClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			GL.ClearDepth(1.0f);
			GL.Enable(EnableCap.ColorMaterial);
			GL.Enable(EnableCap.Light0);
			GL.Hint(HintTarget.PerspectiveCorrectionHint, HintMode.Nicest);
#endif
		}

		private void SetView()
		{
#if GLRender
			int width = openGlControl.Width;
			int height = openGlControl.Height;

            GL.Viewport(0, 0, width, height);
			GL.MatrixMode(MatrixMode.Projection);
			GL.LoadIdentity();

			float ratio = (float)width / (float)height;

			Vec2 extents = new Vec2(ratio * 25.0f, 25.0f);
			extents *= _viewZoom;

			Vec2 lower = _viewCenter - extents;
			Vec2 upper = _viewCenter + extents;

            // L/R/B/T
            GL.Ortho(lower.X, upper.X, lower.Y, upper.Y, -1, 1);

			GL.MatrixMode(MatrixMode.Modelview);
			GL.LoadIdentity();

            
#endif
		}

		private Vec2 ConvertScreenToWorld(float x, float y)
		{
			float tw = openGlControl.Width;
			float th = openGlControl.Height;
			float u = x / tw;
			float v = (th - y) / th;

			float ratio = tw / th;
			Vec2 extents = new Vec2(ratio * 25.0f, 25.0f);
			extents *= _viewZoom;

			Vec2 lower = _viewCenter - extents;
			Vec2 upper = _viewCenter + extents;

		    Vec2 p = new Vec2
		    {
		        X = (1.0f - u)*lower.X + u*upper.X,
		        Y = (1.0f - v)*lower.Y + v*upper.Y
		    };
		    return p;
		}

		private void SimulationLoop()
		{
#if GLRender
			GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

			CurrentTest.SetTextLine(30);
			CurrentTest.Step(_settings);
			OpenGLDebugDraw.DrawString(5, 15, _testEntry.Name);
            openGlControl.Invalidate();//codeyu
			ErrorCode errorCode = GL.GetError();
			if (errorCode != ErrorCode.NoError)
			{
				redrawTimer.Stop();
			}
#endif
		}

		#endregion Render		
        
        private void openGlControl_Resize(object sender, EventArgs e)
        {
            if (openGlControl.ClientSize.Height == 0)
                openGlControl.ClientSize = new System.Drawing.Size(openGlControl.ClientSize.Width, 1);

            GL.Viewport(0, 0, openGlControl.ClientSize.Width, openGlControl.ClientSize.Height);
        }

        private void openGlControl_Paint(object sender, PaintEventArgs e)
        {
            openGlControl.MakeCurrent();
            SimulationLoop();
            //GL.Clear(ClearBufferMask.ColorBufferBit);
            openGlControl.SwapBuffers();
        }
	}
}
