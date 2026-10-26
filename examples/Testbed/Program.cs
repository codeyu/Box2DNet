// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Numerics;
using Hexa.NET.GLFW;
using Hexa.NET.ImGui;
using Hexa.NET.ImGui.Backends.OpenGL3;
using static Box2DNet.B2;
using BackendGLFW = Hexa.NET.ImGui.Backends.GLFW;

namespace Box2DNet.Testbed;

// Testbed application: a GLFW window with an OpenGL 3.3 context, Dear ImGui (cimgui via
// Hexa.NET.ImGui) for the user interface and the ImGui background draw list for debug drawing.
public static unsafe class Program
{
	private const int GLFW_CONTEXT_VERSION_MAJOR = 0x00022002;
	private const int GLFW_CONTEXT_VERSION_MINOR = 0x00022003;
	private const int GLFW_OPENGL_FORWARD_COMPAT = 0x00022006;
	private const int GLFW_OPENGL_PROFILE = 0x00022008;
	private const int GLFW_OPENGL_CORE_PROFILE = 0x00032001;
	private const uint GL_COLOR_BUFFER_BIT = 0x00004000;

	private static delegate* unmanaged<float, float, float, float, void> s_glClearColor;
	private static delegate* unmanaged<uint, void> s_glClear;
	private static delegate* unmanaged<int, int, int, int, void> s_glViewport;

	private static readonly Settings s_settings = new();
	private static readonly Camera s_camera = new();
	private static Sample? s_sample;
	private static int s_sampleIndex;
	private static bool s_rightDown;
	private static Vector2 s_lastMouse;

	public static int Main( string[] args )
	{
		if ( GLFW.Init() == 0 )
		{
			Console.Error.WriteLine( "Failed to initialize GLFW" );
			return 1;
		}

		GLFW.WindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
		GLFW.WindowHint( GLFW_CONTEXT_VERSION_MINOR, 3 );
		GLFW.WindowHint( GLFW_OPENGL_FORWARD_COMPAT, 1 );
		GLFW.WindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );

		string title = $"Box2DNet Testbed - Box2D {GetVersion().major}.{GetVersion().minor}.{GetVersion().revision}";
		GLFWwindowPtr window = GLFW.CreateWindow( 1600, 1000, title, default( GLFWmonitorPtr ), default( GLFWwindowPtr ) );
		if ( window.IsNull )
		{
			Console.Error.WriteLine( "Failed to open GLFW window" );
			GLFW.Terminate();
			return 1;
		}

		GLFW.MakeContextCurrent( window );
		GLFW.SwapInterval( 1 );
		LoadGL();

		ImGuiContextPtr context = ImGui.CreateContext();
		ImGui.SetCurrentContext( context );
		ImGuiIOPtr io = ImGui.GetIO();
		io.ConfigFlags |= ImGuiConfigFlags.NavEnableKeyboard;
		ImGui.StyleColorsDark();

		BackendGLFW.ImGuiImplGLFW.SetCurrentContext( context );
		BackendGLFW.ImGuiImplGLFW.InitForOpenGL( new BackendGLFW.GLFWwindowPtr( (BackendGLFW.GLFWwindow*)window.Handle ), true );
		ImGuiImplOpenGL3.SetCurrentContext( context );
		ImGuiImplOpenGL3.Init( "#version 330" );

		ImGuiDraw draw = new( s_camera );
		RestartSample();

		// --smoke runs every sample for a few frames and exits, for automated checks
		bool smoke = Array.IndexOf( args, "--smoke" ) >= 0;
		int smokeFrames = 0;

		while ( GLFW.WindowShouldClose( window ) == 0 )
		{
			GLFW.PollEvents();

			int fbWidth, fbHeight;
			GLFW.GetFramebufferSize( window, &fbWidth, &fbHeight );
			s_glViewport( 0, 0, fbWidth, fbHeight );
			s_glClearColor( 0.2f, 0.2f, 0.2f, 1.0f );
			s_glClear( GL_COLOR_BUFFER_BIT );

			ImGuiImplOpenGL3.NewFrame();
			BackendGLFW.ImGuiImplGLFW.NewFrame();
			ImGui.NewFrame();

			s_camera.width = MathF.Max( io.DisplaySize.X, 1.0f );
			s_camera.height = MathF.Max( io.DisplaySize.Y, 1.0f );

			HandleInput( io );
			UpdateUI();

			s_sample!.Step( s_settings, ref draw.debugDraw );
			DrawProfile();

			ImGui.Render();
			ImGuiImplOpenGL3.RenderDrawData( ImGui.GetDrawData() );
			GLFW.SwapBuffers( window );

			if ( smoke && ++smokeFrames == 180 )
			{
				Console.WriteLine( $"{SampleRegistry.entries[s_sampleIndex].name}: {s_sample.stepCount} steps, {World_GetCounters( s_sample.WorldId ).bodyCount} bodies" );
				smokeFrames = 0;
				if ( ++s_sampleIndex == SampleRegistry.entries.Length )
				{
					break;
				}

				RestartSample();
			}
		}

		s_sample?.Dispose();
		// the backends keep their own copy of the context pointer, clear it before destroying the context
		ImGuiImplOpenGL3.Shutdown();
		ImGuiImplOpenGL3.SetCurrentContext( default( ImGuiContextPtr ) );
		BackendGLFW.ImGuiImplGLFW.Shutdown();
		BackendGLFW.ImGuiImplGLFW.SetCurrentContext( default( ImGuiContextPtr ) );
		ImGui.DestroyContext( context );
		GLFW.DestroyWindow( window );
		GLFW.Terminate();
		return 0;
	}

	private static void LoadGL()
	{
		s_glClearColor = (delegate* unmanaged<float, float, float, float, void>)GetProc( "glClearColor"u8 );
		s_glClear = (delegate* unmanaged<uint, void>)GetProc( "glClear"u8 );
		s_glViewport = (delegate* unmanaged<int, int, int, int, void>)GetProc( "glViewport"u8 );
	}

	// the u8 literal is null terminated
	private static void* GetProc( ReadOnlySpan<byte> name )
	{
		fixed ( byte* p = name )
		{
			return (void*)GLFW.GetProcAddress( p );
		}
	}

	private static void RestartSample()
	{
		s_sample?.Dispose();
		s_sample = SampleRegistry.entries[s_sampleIndex].create( s_settings, s_camera );
	}

	private static void HandleInput( ImGuiIOPtr io )
	{
		Vector2 mouse = io.MousePos;
		Vec2 worldPoint = s_camera.ToWorld( mouse );

		if ( io.WantCaptureMouse == false )
		{
			if ( io.MouseWheel != 0.0f )
			{
				// zoom about the cursor
				float factor = io.MouseWheel > 0.0f ? 1.0f / 1.1f : 1.1f;
				s_camera.zoom = Math.Clamp( s_camera.zoom * factor, 0.005f, 20.0f );
				Vec2 after = s_camera.ToWorld( mouse );
				s_camera.center = Add( s_camera.center, Sub( worldPoint, after ) );
			}

			if ( ImGui.IsMouseClicked( ImGuiMouseButton.Left ) )
			{
				s_sample!.MouseDown( worldPoint );
			}

			if ( ImGui.IsMouseClicked( ImGuiMouseButton.Right ) )
			{
				s_rightDown = true;
			}
		}

		if ( ImGui.IsMouseReleased( ImGuiMouseButton.Left ) )
		{
			s_sample!.MouseUp();
		}

		if ( ImGui.IsMouseReleased( ImGuiMouseButton.Right ) )
		{
			s_rightDown = false;
		}

		if ( s_rightDown )
		{
			// pan
			Vec2 last = s_camera.ToWorld( s_lastMouse );
			s_camera.center = Add( s_camera.center, Sub( last, worldPoint ) );
		}

		s_sample!.MouseMove( s_camera.ToWorld( mouse ) );
		s_lastMouse = mouse;

		if ( io.WantCaptureKeyboard == false )
		{
			if ( ImGui.IsKeyPressed( ImGuiKey.P, false ) )
			{
				s_settings.pause = !s_settings.pause;
			}

			if ( ImGui.IsKeyPressed( ImGuiKey.O ) )
			{
				s_settings.pause = true;
				s_settings.singleStep = true;
			}

			if ( ImGui.IsKeyPressed( ImGuiKey.R, false ) )
			{
				RestartSample();
			}
		}
	}

	private static void UpdateUI()
	{
		ImGui.SetNextWindowPos( new Vector2( s_camera.width - 330.0f, 10.0f ), ImGuiCond.Always );
		ImGui.SetNextWindowSize( new Vector2( 320.0f, s_camera.height - 20.0f ), ImGuiCond.Always );
		ImGui.Begin( "Tools", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse );

		if ( ImGui.BeginTabBar( "ControlTabs" ) )
		{
			if ( ImGui.BeginTabItem( "Controls" ) )
			{
				int newIndex = s_sampleIndex;
				SampleEntry current = SampleRegistry.entries[s_sampleIndex];
				if ( ImGui.BeginCombo( "Sample", current.category + " / " + current.name ) )
				{
					for ( int i = 0; i < SampleRegistry.entries.Length; ++i )
					{
						SampleEntry e = SampleRegistry.entries[i];
						if ( ImGui.Selectable( e.category + " / " + e.name, i == s_sampleIndex ) )
						{
							newIndex = i;
						}
					}

					ImGui.EndCombo();
				}

				ImGui.SliderInt( "Sub-steps", ref s_settings.subStepCount, 1, 32 );
				ImGui.SliderFloat( "Hertz", ref s_settings.hertz, 5.0f, 240.0f, "%.0f hz" );
				if ( ImGui.SliderInt( "Workers", ref s_settings.workerCount, 1, Math.Min( MaxWorkers, Environment.ProcessorCount ) ) )
				{
					RestartSample();
				}

				ImGui.Separator();
				ImGui.Checkbox( "Sleep", ref s_settings.enableSleep );
				ImGui.Checkbox( "Continuous", ref s_settings.enableContinuous );
				ImGui.Separator();
				ImGui.Checkbox( "Shapes", ref s_settings.drawShapes );
				ImGui.Checkbox( "Joints", ref s_settings.drawJoints );
				ImGui.Checkbox( "Joint Extras", ref s_settings.drawJointExtras );
				ImGui.Checkbox( "Bounds", ref s_settings.drawBounds );
				ImGui.Checkbox( "Contact Points", ref s_settings.drawContacts );
				ImGui.Checkbox( "Contact Normals", ref s_settings.drawContactNormals );
				ImGui.Checkbox( "Contact Forces", ref s_settings.drawContactForces );
				ImGui.Checkbox( "Friction Forces", ref s_settings.drawFrictionForces );
				ImGui.Checkbox( "Mass", ref s_settings.drawMass );
				ImGui.Checkbox( "Islands", ref s_settings.drawIslands );
				ImGui.Checkbox( "Profile", ref s_settings.drawProfile );
				ImGui.Separator();

				if ( ImGui.Button( s_settings.pause ? "Resume (P)" : "Pause (P)" ) )
				{
					s_settings.pause = !s_settings.pause;
				}

				ImGui.SameLine();
				if ( ImGui.Button( "Single Step (O)" ) )
				{
					s_settings.pause = true;
					s_settings.singleStep = true;
				}

				ImGui.SameLine();
				if ( ImGui.Button( "Restart (R)" ) || newIndex != s_sampleIndex )
				{
					s_sampleIndex = newIndex;
					RestartSample();
				}

				ImGui.Separator();
				ImGui.TextDisabled( "Left drag: move bodies" );
				ImGui.TextDisabled( "Right drag: pan, wheel: zoom" );
				ImGui.EndTabItem();
			}

			if ( ImGui.BeginTabItem( "Sample" ) )
			{
				s_sample!.UpdateGui();
				ImGui.EndTabItem();
			}

			ImGui.EndTabBar();
		}

		ImGui.End();
	}

	private static void DrawProfile()
	{
		if ( s_settings.drawProfile == false )
		{
			return;
		}

		WorldId worldId = s_sample!.WorldId;
		Counters c = World_GetCounters( worldId );
		Profile p = World_GetProfile( worldId );

		ImGui.SetNextWindowPos( new Vector2( 10.0f, 10.0f ), ImGuiCond.Always );
		ImGui.SetNextWindowBgAlpha( 0.35f );
		ImGui.Begin( "Profile",
					 ImGuiWindowFlags.NoDecoration | ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.NoSavedSettings );
		SampleEntry entry = SampleRegistry.entries[s_sampleIndex];
		ImGui.Text( $"{entry.category} : {entry.name}" );
		ImGui.Text( $"bodies/shapes/contacts/joints = {c.bodyCount}/{c.shapeCount}/{c.contactCount}/{c.jointCount}" );
		ImGui.Text( $"islands/tasks = {c.islandCount}/{c.taskCount}" );
		ImGui.Text( $"tree height static/movable = {c.staticTreeHeight}/{c.treeHeight}" );
		ImGui.Text( $"step [ms] = {p.step:F2} (collide {p.collide:F2}, solve {p.solve:F2})" );
		ImGui.Text( $"fps = {ImGui.GetIO().Framerate:F0}" );
		ImGui.End();
	}
}
