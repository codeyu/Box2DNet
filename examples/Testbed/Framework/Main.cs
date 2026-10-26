// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics;
using System.Numerics;
using Hexa.NET.GLFW;
using Hexa.NET.ImGui;
using Hexa.NET.ImGui.Backends.OpenGL3;
using static Box2DNet.B2;
using static Box2DNet.Shared.SharedLib;
using static Box2DNet.Testbed.DrawFunctions;
using static Box2DNet.Testbed.Glfw;
using BackendGLFW = Hexa.NET.ImGui.Backends.GLFW;

namespace Box2DNet.Testbed;

// From main.cpp: a GLFW window with an OpenGL 3.3 context, Dear ImGui (cimgui via Hexa.NET.ImGui) for
// the user interface and the ImGui background draw list for the debug drawing.
public static unsafe partial class SampleApp
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

	private static readonly SampleContext s_context = new();
	private static bool s_rightMouseDown;
	private static Pos s_clickPointWS;

	// The native callbacks must stay alive
	private static GLFWkeyfun? s_keyCallback;
	private static GLFWcharfun? s_charCallback;
	private static GLFWmousebuttonfun? s_mouseButtonCallback;
	private static GLFWcursorposfun? s_cursorPosCallback;
	private static GLFWscrollfun? s_scrollCallback;

	private static void KeyCallback( GLFWwindow* window, int key, int scancode, int action, int mods )
	{
		if ( ImGui.GetIO().WantCaptureKeyboard )
		{
			return;
		}

		if ( action != GLFW_PRESS )
		{
			return;
		}

		Sample? sample = s_context.sample;
		switch ( key )
		{
			case GLFW_KEY_ESCAPE:
				// Layered cancel. Quit lives on Ctrl+Q so a stray Esc can't close the app.
				if ( s_context.showControls )
				{
					s_context.showControls = false;
				}
				else
				{
					sample?.Keyboard( key, action, mods );
				}

				break;

			case GLFW_KEY_Q:
				if ( mods == GLFW_MOD_CONTROL )
				{
					GLFW.SetWindowShouldClose( s_context.window, 1 );
				}
				else
				{
					sample?.Keyboard( key, action, mods );
				}

				break;

			case GLFW_KEY_LEFT:
			case GLFW_KEY_RIGHT:
			case GLFW_KEY_DOWN:
			case GLFW_KEY_UP:
			case GLFW_KEY_Z:
			case GLFW_KEY_X:
				// Held camera keys, polled each frame
				break;

			case GLFW_KEY_HOME:
				sample?.FocusHome();
				break;

			case GLFW_KEY_R:
				SelectSample( s_context, s_context.sampleIndex, true );
				break;

			case GLFW_KEY_O:
				if ( mods == GLFW_MOD_CONTROL )
				{
					s_context.showUI = true;
					s_context.openSamplePicker = true;
				}
				else
				{
					sample?.Keyboard( key, action, mods );
				}

				break;

			// Step forward
			case GLFW_KEY_PERIOD:
				s_context.singleStep += ( mods & GLFW_MOD_SHIFT ) != 0 ? 5 : 1;
				break;

			case GLFW_KEY_P:
			case GLFW_KEY_PAUSE:
				s_context.pause = !s_context.pause;
				break;

			case GLFW_KEY_LEFT_BRACKET:
			{
				// Switch to previous test
				int selection = s_context.sampleIndex - 1;
				if ( selection < 0 )
				{
					selection = g_sampleCount - 1;
				}

				SelectSample( s_context, selection, false );
				break;
			}

			case GLFW_KEY_RIGHT_BRACKET:
			{
				// Switch to next test
				int selection = s_context.sampleIndex + 1;
				if ( selection == g_sampleCount )
				{
					selection = 0;
				}

				SelectSample( s_context, selection, false );
				break;
			}

			case GLFW_KEY_TAB:
				s_context.showUI = !s_context.showUI;
				break;

			case GLFW_KEY_M:
				s_context.showMetrics = !s_context.showMetrics;
				break;

			case GLFW_KEY_I:
				s_context.showProfile = !s_context.showProfile;
				break;

			default:
				sample?.Keyboard( key, action, mods );
				break;
		}
	}

	// Held keys move the camera. Rates are per second so a slow frame doesn't slow the camera, and
	// the pan scales with zoom so it covers the same share of the view at any zoom.
	private static void UpdateCameraKeys( float frameTime )
	{
		if ( ImGui.GetIO().WantCaptureKeyboard )
		{
			return;
		}

		GLFWwindowPtr window = s_context.window;
		Camera camera = s_context.camera;

		// A stall such as a sample switch would otherwise jump the camera.
		float dt = MinFloat( frameTime, 0.1f );

		// About 35% per second.
		float zoomRate = 0.3f;
		if ( glfwGetKey( window, GLFW_KEY_Z ) == GLFW_PRESS )
		{
			// Samples may start beyond the limit, and zooming out must never pull the view in
			camera.zoom = MinFloat( MathF.Exp( zoomRate * dt ) * camera.zoom, MaxFloat( camera.zoom, 100.0f ) );
		}
		else if ( glfwGetKey( window, GLFW_KEY_X ) == GLFW_PRESS )
		{
			camera.zoom = MaxFloat( MathF.Exp( -zoomRate * dt ) * camera.zoom, 0.5f );
		}

		// One view half height per second
		float panStep = camera.zoom * dt;
		Vec2 pan = Vec2_zero;
		if ( glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS )
		{
			pan.x -= panStep;
		}

		if ( glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS )
		{
			pan.x += panStep;
		}

		if ( glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS )
		{
			pan.y -= panStep;
		}

		if ( glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS )
		{
			pan.y += panStep;
		}

		camera.center = OffsetPos( camera.center, pan );
	}

	private static void CharCallback( GLFWwindow* window, uint c )
	{
		if ( ImGui.GetIO().WantCaptureKeyboard )
		{
			return;
		}

		// Read the typed character so ? works on any keyboard layout
		if ( c == '?' )
		{
			s_context.showControls = !s_context.showControls;
		}
	}

	private static Vec2 GetCursor()
	{
		double xd, yd;
		GLFW.GetCursorPos( s_context.window, &xd, &yd );
		return new Vec2( (float)xd, (float)yd );
	}

	private static void MouseButtonCallback( GLFWwindow* window, int button, int action, int modifiers )
	{
		if ( ImGui.GetIO().WantCaptureMouse )
		{
			return;
		}

		Vec2 ps = GetCursor();

		// Use the mouse to move things around.
		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			Pos pw = ConvertScreenToWorld( s_context.camera, ps );
			if ( action == GLFW_PRESS )
			{
				s_context.sample?.MouseDown( pw, button, modifiers );
			}

			if ( action == GLFW_RELEASE )
			{
				s_context.sample?.MouseUp( pw, button );
			}
		}
		else if ( button == GLFW_MOUSE_BUTTON_2 )
		{
			if ( action == GLFW_PRESS )
			{
				s_clickPointWS = ConvertScreenToWorld( s_context.camera, ps );
				s_rightMouseDown = true;
			}

			if ( action == GLFW_RELEASE )
			{
				s_rightMouseDown = false;
			}
		}
	}

	private static void MouseMotionCallback( GLFWwindow* window, double xd, double yd )
	{
		Vec2 ps = new( (float)xd, (float)yd );

		Pos pw = ConvertScreenToWorld( s_context.camera, ps );
		s_context.sample?.MouseMove( pw );

		if ( s_rightMouseDown )
		{
			Vec2 diff = Sub( pw, s_clickPointWS );
			s_context.camera.center.x -= diff.x;
			s_context.camera.center.y -= diff.y;
			s_clickPointWS = ConvertScreenToWorld( s_context.camera, ps );
		}
	}

	private static void ScrollCallback( GLFWwindow* window, double dx, double dy )
	{
		if ( ImGui.GetIO().WantCaptureMouse )
		{
			return;
		}

		Vec2 ps = GetCursor();
		Pos pw1 = ConvertScreenToWorld( s_context.camera, ps );

		if ( dy > 0 )
		{
			s_context.camera.zoom /= 1.1f;
		}
		else
		{
			s_context.camera.zoom *= 1.1f;
		}

		Pos pw2 = ConvertScreenToWorld( s_context.camera, ps );

		// Keep the world point under the cursor fixed across the zoom.
		Vec2 pan = Sub( pw2, pw1 );
		s_context.camera.center.x -= pan.x;
		s_context.camera.center.y -= pan.y;
	}

	private static void ApplyUIStyle()
	{
		ImGuiStylePtr style = ImGui.GetStyle();

		style.WindowPadding = new Vector2( 10.0f, 10.0f );
		style.FramePadding = new Vector2( 8.0f, 4.0f );
		style.CellPadding = new Vector2( 6.0f, 4.0f );
		style.ItemSpacing = new Vector2( 8.0f, 7.0f );
		style.ItemInnerSpacing = new Vector2( 7.0f, 4.0f );
		style.IndentSpacing = 18.0f;
		style.ScrollbarSize = 12.0f;
		style.GrabMinSize = 10.0f;

		style.WindowBorderSize = 1.0f;
		style.FrameBorderSize = 0.0f;
		style.PopupBorderSize = 1.0f;

		style.WindowRounding = 4.0f;
		style.ChildRounding = 4.0f;
		style.PopupRounding = 4.0f;
		style.FrameRounding = 3.0f;
		style.GrabRounding = 3.0f;
		style.ScrollbarRounding = 3.0f;
		style.TabRounding = 3.0f;

		style.WindowTitleAlign = new Vector2( 0.0f, 0.5f );

		// Palette: neutral charcoal surfaces, one steel-blue accent at three brightnesses.
		Vector4 accent = new( 0.28f, 0.48f, 0.66f, 1.00f );
		Vector4 accentHi = new( 0.38f, 0.60f, 0.80f, 1.00f );
		Vector4 accentLo = new( 0.22f, 0.36f, 0.50f, 1.00f );

		Span<Vector4> c = style.Colors;
		c[(int)ImGuiCol.Text] = new Vector4( 0.90f, 0.91f, 0.93f, 1.00f );
		c[(int)ImGuiCol.TextDisabled] = new Vector4( 0.49f, 0.51f, 0.55f, 1.00f );
		c[(int)ImGuiCol.WindowBg] = new Vector4( 0.110f, 0.115f, 0.125f, 0.97f );
		c[(int)ImGuiCol.ChildBg] = new Vector4( 0.00f, 0.00f, 0.00f, 0.00f );
		c[(int)ImGuiCol.PopupBg] = new Vector4( 0.100f, 0.105f, 0.115f, 0.98f );
		c[(int)ImGuiCol.Border] = new Vector4( 0.00f, 0.00f, 0.00f, 0.45f );
		c[(int)ImGuiCol.FrameBg] = new Vector4( 0.18f, 0.19f, 0.21f, 1.00f );
		c[(int)ImGuiCol.FrameBgHovered] = new Vector4( 0.24f, 0.26f, 0.29f, 1.00f );
		c[(int)ImGuiCol.FrameBgActive] = new Vector4( 0.29f, 0.32f, 0.36f, 1.00f );
		c[(int)ImGuiCol.TitleBg] = new Vector4( 0.090f, 0.095f, 0.105f, 1.00f );
		c[(int)ImGuiCol.TitleBgActive] = new Vector4( 0.14f, 0.16f, 0.19f, 1.00f );
		c[(int)ImGuiCol.MenuBarBg] = new Vector4( 0.13f, 0.14f, 0.16f, 1.00f );
		c[(int)ImGuiCol.CheckMark] = accentHi;
		c[(int)ImGuiCol.SliderGrab] = accent;
		c[(int)ImGuiCol.SliderGrabActive] = accentHi;
		c[(int)ImGuiCol.Button] = new Vector4( 0.22f, 0.24f, 0.27f, 1.00f );
		c[(int)ImGuiCol.ButtonHovered] = accentLo;
		c[(int)ImGuiCol.ButtonActive] = accent;
		c[(int)ImGuiCol.Header] = new Vector4( 0.19f, 0.21f, 0.24f, 1.00f );
		c[(int)ImGuiCol.HeaderHovered] = accentLo;
		c[(int)ImGuiCol.HeaderActive] = accent;
		c[(int)ImGuiCol.Separator] = new Vector4( 1.00f, 1.00f, 1.00f, 0.09f );
		c[(int)ImGuiCol.Tab] = new Vector4( 0.15f, 0.16f, 0.18f, 1.00f );
		c[(int)ImGuiCol.TabHovered] = accentLo;
		c[(int)ImGuiCol.TabSelected] = accent;
		c[(int)ImGuiCol.PlotHistogram] = accent;
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

	public static int Main( string[] args )
	{
		SetAssertCallback( ( condition, fileName, lineNumber ) =>
		{
			Console.WriteLine( $"SAMPLE ASSERTION: {condition}, {fileName}, line {lineNumber}" );
			return 1;
		} );

		RegisterSamples();

		// Sort first so every sample index read below is a sorted index
		SortSamples();

		// --smoke runs every sample for a few frames and exits, for automated checks. An optional
		// filter limits it to the samples whose category or name contains the text.
		int smokeIndex = Array.IndexOf( args, "--smoke" );
		bool smoke = smokeIndex >= 0;
		string? smokeFilter = smoke && smokeIndex + 1 < args.Length ? args[smokeIndex + 1] : null;

		s_context.Load();

		// First run with no settings opens with the controls window up
		s_context.showControls = s_context.newUser && smoke == false;

		s_context.workerCount = MinInt( 8, GetNumberOfCores() / 2 );

		if ( GLFW.Init() == 0 )
		{
			Console.Error.WriteLine( "Failed to initialize GLFW" );
			return -1;
		}

		GLFW.WindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
		GLFW.WindowHint( GLFW_CONTEXT_VERSION_MINOR, 3 );
		GLFW.WindowHint( GLFW_OPENGL_FORWARD_COMPAT, 1 );
		GLFW.WindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );

		Version version = GetVersion();
		string title = $"Box2DNet Samples - Box2D Version {version.major}.{version.minor}.{version.revision} - single precision";

		s_context.window = GLFW.CreateWindow( (int)s_context.camera.width, (int)s_context.camera.height, title, default( GLFWmonitorPtr ),
											  default( GLFWwindowPtr ) );
		if ( s_context.window.IsNull )
		{
			Console.Error.WriteLine( "Failed to open GLFW window." );
			GLFW.Terminate();
			return -1;
		}

		GLFW.MakeContextCurrent( s_context.window );
		GLFW.SwapInterval( 1 );
		LoadGL();

		// Installed before the ImGui backend so it chains to these after updating ImGui
		s_keyCallback = KeyCallback;
		s_charCallback = CharCallback;
		s_mouseButtonCallback = MouseButtonCallback;
		s_cursorPosCallback = MouseMotionCallback;
		s_scrollCallback = ScrollCallback;
		GLFW.SetKeyCallback( s_context.window, s_keyCallback );
		GLFW.SetCharCallback( s_context.window, s_charCallback );
		GLFW.SetMouseButtonCallback( s_context.window, s_mouseButtonCallback );
		GLFW.SetCursorPosCallback( s_context.window, s_cursorPosCallback );
		GLFW.SetScrollCallback( s_context.window, s_scrollCallback );

		ImGuiContextPtr imguiContext = ImGui.CreateContext();
		ImGui.SetCurrentContext( imguiContext );
		ImGuiIOPtr io = ImGui.GetIO();
		ApplyUIStyle();

		BackendGLFW.ImGuiImplGLFW.SetCurrentContext( imguiContext );
		BackendGLFW.ImGuiImplGLFW.InitForOpenGL( new BackendGLFW.GLFWwindowPtr( (BackendGLFW.GLFWwindow*)s_context.window.Handle ), true );
		ImGuiImplOpenGL3.SetCurrentContext( imguiContext );
		ImGuiImplOpenGL3.Init( "#version 330" );

		s_context.draw = CreateDraw();

		s_context.sampleIndex = ClampInt( s_context.sampleIndex, 0, g_sampleCount - 1 );

		// --sample text opens the first sample whose category or name contains the text
		int sampleArg = Array.IndexOf( args, "--sample" );
		if ( sampleArg >= 0 && sampleArg + 1 < args.Length )
		{
			int found = NextSmokeSample( -1, args[sampleArg + 1] );
			if ( found >= 0 )
			{
				s_context.sampleIndex = found;
			}
		}

		if ( smoke )
		{
			s_context.sampleIndex = NextSmokeSample( -1, smokeFilter );
		}

		float frameTime = 0.0f;
		int smokeFrames = 0;
		int smokeFailures = 0;

		if ( smoke )
		{
			if ( SmokeSelect( s_context.sampleIndex, smokeFilter, ref smokeFailures ) == false )
			{
				GLFW.SetWindowShouldClose( s_context.window, 1 );
			}
		}
		else
		{
			SelectSample( s_context, s_context.sampleIndex, false );
		}
		Stopwatch clock = Stopwatch.StartNew();

		while ( GLFW.WindowShouldClose( s_context.window ) == 0 )
		{
			double time1 = clock.Elapsed.TotalSeconds;

			UpdateCameraKeys( frameTime );

			int width, height;
			GLFW.GetWindowSize( s_context.window, &width, &height );
			s_context.camera.width = width;
			s_context.camera.height = height;

			int bufferWidth, bufferHeight;
			GLFW.GetFramebufferSize( s_context.window, &bufferWidth, &bufferHeight );
			s_glViewport( 0, 0, bufferWidth, bufferHeight );
			s_glClearColor( 0.2f, 0.2f, 0.2f, 1.0f );
			s_glClear( GL_COLOR_BUFFER_BIT );

			ImGuiImplOpenGL3.NewFrame();
			BackendGLFW.ImGuiImplGLFW.NewFrame();
			ImGui.NewFrame();

			Sample sample = s_context.sample!;
			try
			{
				sample.ResetText();

				if ( s_context.showUI == false )
				{
					// Minimal hud
					sample.DrawHud( frameTime );
				}

				SetDrawOrigin( s_context.draw, s_context.camera.center );

				sample.Step();

				FlushDraw( s_context.draw, s_context.camera );

				if ( s_context.showUI )
				{
					DrawUI( s_context, frameTime );
				}
			}
			catch ( Exception e ) when ( smoke )
			{
				SampleEntry failed = g_sampleEntries[s_context.sampleIndex];
				Console.WriteLine( $"FAILED {failed.category} / {failed.name}: {e.GetType().Name}: {e.Message}" );
				Console.WriteLine( e.StackTrace );
				smokeFailures += 1;
				smokeFrames = int.MaxValue - 1;
			}

			ImGui.Render();
			ImGuiImplOpenGL3.RenderDrawData( ImGui.GetDrawData() );
			GLFW.SwapBuffers( s_context.window );
			GLFW.PollEvents();

			if ( smoke && ++smokeFrames >= 60 )
			{
				SampleEntry entry = g_sampleEntries[s_context.sampleIndex];
				if ( smokeFrames < int.MaxValue )
				{
					Console.WriteLine( $"ok {entry.category} / {entry.name}: {s_context.sample!.m_stepCount} steps" );
				}

				smokeFrames = 0;
				if ( SmokeSelect( NextSmokeSample( s_context.sampleIndex, smokeFilter ), smokeFilter, ref smokeFailures ) == false )
				{
					break;
				}
			}

			frameTime = (float)( clock.Elapsed.TotalSeconds - time1 );
		}

		s_context.sample?.Dispose();
		s_context.sample = null;

		// the backends keep their own copy of the context pointer, clear it before destroying the context
		ImGuiImplOpenGL3.Shutdown();
		ImGuiImplOpenGL3.SetCurrentContext( default( ImGuiContextPtr ) );
		BackendGLFW.ImGuiImplGLFW.Shutdown();
		BackendGLFW.ImGuiImplGLFW.SetCurrentContext( default( ImGuiContextPtr ) );
		ImGui.DestroyContext( imguiContext );
		GLFW.DestroyWindow( s_context.window );
		GLFW.Terminate();

		if ( smoke )
		{
			Console.WriteLine( smokeFailures == 0 ? "smoke ok" : $"smoke failures: {smokeFailures}" );
			return smokeFailures == 0 ? 0 : 1;
		}

		s_context.Save();
		return 0;
	}

	// Selects the sample at index, or the next one that can be created. False when none is left.
	private static bool SmokeSelect( int index, string? filter, ref int failures )
	{
		while ( index >= 0 )
		{
			try
			{
				SelectSample( s_context, index, false );
				return true;
			}
			catch ( Exception e )
			{
				SampleEntry failed = g_sampleEntries[index];
				Console.WriteLine( $"FAILED {failed.category} / {failed.name}: {e.GetType().Name}: {e.Message}" );
				Console.WriteLine( e.StackTrace );
				failures += 1;
				s_context.sample = null;
				index = NextSmokeSample( index, filter );
			}
		}

		return false;
	}

	private static int NextSmokeSample( int index, string? filter )
	{
		for ( int i = index + 1; i < g_sampleCount; ++i )
		{
			SampleEntry e = g_sampleEntries[i];
			if ( filter == null || ( e.category + " / " + e.name ).Contains( filter, StringComparison.OrdinalIgnoreCase ) )
			{
				return i;
			}
		}

		return -1;
	}
}
