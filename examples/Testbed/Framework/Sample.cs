// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using System.Text.Json;
using Hexa.NET.GLFW;
using Hexa.NET.ImGui;
using static Box2DNet.B2;
using static Box2DNet.Shared.SharedLib;
using static Box2DNet.Testbed.DrawFunctions;
using static Box2DNet.Testbed.Glfw;

namespace Box2DNet.Testbed;

// From sample.h. Recording and replay are not ported.
public sealed class SampleContext
{
	private const string fileName = "settings.json";

	public GLFWwindowPtr window;
	public Camera camera = GetDefaultCamera();
	public Draw draw = null!;
	public Sample? sample;
	public Capacity capacity;
	public DebugDraw debugDraw;
	public float uiScale = 1.0f;
	public float hertz = 60.0f;
	public float recycleDistance = 0.05f;
	public int subStepCount = 4;
	public int restitutionIterations = 2;
	public int workerCount = 1;
	public bool restart;
	public bool pause;
	public int singleStep;
	public bool enableWarmStarting = true;
	public bool enableContinuous = true;
	public bool enableRestitutionPropagation;
	public bool enableSSE2Fallback;
	public bool enableSleep = true;
	public bool showUI = true;

	// Metrics drawer visibility. M toggles.
	public bool showMetrics;

	// Profile panel visibility. I toggles.
	public bool showProfile;

	// Controls help window visibility. ? toggles, Esc closes.
	public bool showControls;

	// Camera the sample set up on creation. Home returns to it.
	public Pos homeCenter = new( 0.0f, 20.0f );
	public float homeZoom = 1.0f;

	// Set by Ctrl+O; consumed by the UI to open the sample picker.
	public bool openSamplePicker;

	// These are persisted
	public int sampleIndex;
	public bool newUser = true;

	private sealed class Settings
	{
		public int sampleIndex { get; set; }
		public bool drawShapes { get; set; } = true;
		public bool showMetrics { get; set; }
		public bool showProfile { get; set; }
	}

	public void Save()
	{
		Settings settings = new()
		{
			sampleIndex = sampleIndex, drawShapes = debugDraw.drawShapes, showMetrics = showMetrics, showProfile = showProfile,
		};

		try
		{
			File.WriteAllText( fileName, JsonSerializer.Serialize( settings, new JsonSerializerOptions { WriteIndented = true } ) );
		}
		catch ( IOException )
		{
		}
	}

	public void Load()
	{
		camera = GetDefaultCamera();
		debugDraw = DefaultDebugDraw();
		debugDraw.DrawPolygonFcn = ( transform, vertices, vertexCount, color, context ) =>
			DrawPolygon( draw, transform, vertices, vertexCount, color );
		debugDraw.DrawSolidPolygonFcn = ( transform, vertices, vertexCount, radius, color, context ) =>
			DrawSolidPolygon( draw, transform, vertices, vertexCount, radius, color );
		debugDraw.DrawCircleFcn = ( center, radius, color, context ) => DrawCircle( draw, center, radius, color );
		debugDraw.DrawSolidCircleFcn = ( transform, center, radius, color, context ) =>
			DrawSolidCircle( draw, transform, center, radius, color );
		debugDraw.DrawSolidCapsuleFcn = ( p1, p2, radius, color, context ) => DrawCapsule( draw, p1, p2, radius, color );
		debugDraw.DrawLineFcn = ( p1, p2, color, context ) => DrawLine( draw, p1, p2, color );
		debugDraw.DrawTransformFcn = ( transform, context ) => DrawTransform( draw, transform, 1.0f );
		debugDraw.DrawPointFcn = ( p, size, color, context ) => DrawPoint( draw, p, size, color );
		debugDraw.DrawStringFcn = ( p, s, color, context ) => DrawString( draw, camera, p, color, "%s", s );
		debugDraw.DrawBoundsFcn = ( aabb, color, context ) => DrawBounds( draw, aabb, color );

		debugDraw.context = this;

		recycleDistance = ContactRecycleDistance;

		try
		{
			if ( File.Exists( fileName ) == false )
			{
				return;
			}

			Settings? settings = JsonSerializer.Deserialize<Settings>( File.ReadAllText( fileName ) );
			if ( settings == null )
			{
				return;
			}

			newUser = false;
			sampleIndex = settings.sampleIndex;
			debugDraw.drawShapes = settings.drawShapes;
			showMetrics = settings.showMetrics;
			showProfile = settings.showProfile;
		}
		catch ( Exception e ) when ( e is IOException or JsonException )
		{
		}
	}
}

public delegate Sample SampleCreateFcn( SampleContext context );

public delegate Capacity SampleCapacityFcn();

public sealed class SampleEntry
{
	public string category = "";
	public string name = "";
	public SampleCreateFcn createFcn = null!;
	public SampleCapacityFcn? capacityFcn;
}

public class Sample : IDisposable
{
	public const int m_maxTasks = 512;
	public const int m_maxThreads = 64;
	public const int m_profileCapacity = 512;

#if DEBUG
	public const bool m_isDebug = true;
#else
	public const bool m_isDebug = false;
#endif

	public SampleContext m_context;
	public Camera m_camera;
	public Draw m_draw;

	public BodyId m_mouseBodyId;

	public WorldId m_worldId;
	public JointId m_mouseJointId;
	public Pos m_mousePoint;
	public float m_mouseForceScale;
	public int m_stepCount;
	public float m_screenTextX;
	public float m_screenTextY;

	public Profile[] m_profiles = new Profile[m_profileCapacity];
	public int m_currentProfileIndex;
	public ulong m_profileReadIndex;
	public ulong m_profileWriteIndex;

	public bool m_didStep;
	public bool m_stepWhilePaused;

	// createWorld false lets a subclass that supplies its own world skip the throwaway world
	public Sample( SampleContext context, bool createWorld = true )
	{
		m_context = context;
		m_camera = context.camera;
		m_draw = context.draw;

		m_worldId = nullWorldId;

		m_mouseJointId = nullJointId;

		m_stepCount = 0;
		m_didStep = false;
		m_stepWhilePaused = true;
		m_screenTextX = 5.0f;
		m_screenTextY = 0.0f;

		m_mouseBodyId = nullBodyId;
		m_mousePoint = default;
		m_mouseForceScale = 100.0f;

		m_currentProfileIndex = 0;
		m_profileReadIndex = 0;
		m_profileWriteIndex = 0;

		g_randomSeed = RAND_SEED;

		if ( createWorld )
		{
			CreateWorld();
		}
	}

	public virtual void Dispose()
	{
		if ( m_worldId.IsNonNull )
		{
			DestroyWorld( m_worldId );
			m_worldId = nullWorldId;
		}

		GC.SuppressFinalize( this );
	}

	public void CreateWorld()
	{
		if ( m_worldId.IsNonNull )
		{
			DestroyWorld( m_worldId );
			m_worldId = nullWorldId;
		}

		WorldDef worldDef = DefaultWorldDef();
		worldDef.workerCount = m_context.workerCount;
		worldDef.userTaskContext = this;
		worldDef.enableSleep = m_context.enableSleep;
		worldDef.capacity = m_context.capacity;
		m_worldId = B2.CreateWorld( worldDef );

		World_SetContactRecycleDistance( m_worldId, m_context.recycleDistance );
	}

	public void ResetText()
	{
		float fontSize = ImGui.GetFontSize();
		if ( m_context.showUI )
		{
			m_screenTextY = ImGui.GetFrameHeight() + 1.5f * fontSize;
		}
		else
		{
			m_screenTextY = 3.0f * fontSize;
		}

		m_screenTextX = 5.0f;
		if ( IsProfileVisible() )
		{
			m_screenTextX += GetProfilePanelWidth() + 0.5f * fontSize;
		}
	}

	public virtual bool DrawControls()
	{
		return false;
	}

	// Allow solver controls to be hidden by a sample.
	public virtual bool HasSolverControls()
	{
		return true;
	}

	// Allow a sample without a world step to hide the profile panel.
	public virtual bool HasProfile()
	{
		return true;
	}

	// Width of the right info panel in font units. A sample with a wide control area can widen it.
	public virtual float InfoPanelWidthEm()
	{
		return SampleApp.INFO_PANEL_WIDTH;
	}

	// Allow a sample to add extra tabs to the metrics window.
	public virtual void DrawMetricsTab()
	{
	}

	public virtual void Keyboard( int key, int action, int mods )
	{
	}

	// Home. Defaults to the camera the sample set up on creation.
	public virtual void FocusHome()
	{
		m_context.camera.center = m_context.homeCenter;
		m_context.camera.zoom = m_context.homeZoom;
	}

	private sealed class QueryContext
	{
		public Pos point;
		public BodyId bodyId = nullBodyId;
	}

	private static bool QueryCallback( ShapeId shapeId, object? context )
	{
		QueryContext queryContext = (QueryContext)context!;

		BodyId bodyId = Shape_GetBody( shapeId );
		BodyType bodyType = Body_GetType( bodyId );
		if ( bodyType != BodyType.dynamicBody )
		{
			// continue query
			return true;
		}

		bool overlap = Shape_TestPoint( shapeId, queryContext.point );
		if ( overlap )
		{
			// found shape
			queryContext.bodyId = bodyId;
			return false;
		}

		return true;
	}

	public virtual void MouseDown( Pos p, int button, int mod )
	{
		if ( m_mouseJointId.IsNonNull )
		{
			return;
		}

		if ( button == GLFW_MOUSE_BUTTON_1 )
		{
			// A tiny box around the click point, exact at any distance with the click as the origin
			Vec2 d = new( 0.001f, 0.001f );
			AABB box = new( Neg( d ), d );

			m_mousePoint = p;

			// Query the world for overlapping shapes.
			QueryContext queryContext = new() { point = p, bodyId = nullBodyId };
			World_OverlapAABB( m_worldId, p, box, DefaultQueryFilter(), QueryCallback, queryContext );

			if ( queryContext.bodyId.IsNonNull )
			{
				BodyDef bodyDef = DefaultBodyDef();
				bodyDef.type = BodyType.kinematicBody;
				bodyDef.position = m_mousePoint;
				bodyDef.enableSleep = false;
				m_mouseBodyId = CreateBody( m_worldId, bodyDef );

				MotorJointDef jointDef = DefaultMotorJointDef();
				jointDef.@base.bodyIdA = m_mouseBodyId;
				jointDef.@base.bodyIdB = queryContext.bodyId;
				jointDef.@base.localFrameB.p = Body_GetLocalPoint( queryContext.bodyId, p );
				jointDef.linearHertz = 7.5f;
				jointDef.linearDampingRatio = 1.0f;

				MassData massData = Body_GetMassData( queryContext.bodyId );
				float g = Length( World_GetGravity( m_worldId ) );
				float mg = massData.mass * g;

				jointDef.maxSpringForce = m_mouseForceScale * mg;

				if ( massData.mass > 0.0f )
				{
					// This acts like angular friction
					float lever = MathF.Sqrt( massData.rotationalInertia / massData.mass );
					jointDef.maxVelocityTorque = 0.25f * lever * mg;
				}

				m_mouseJointId = CreateMotorJoint( m_worldId, jointDef );
			}
		}
	}

	public virtual void MouseUp( Pos p, int button )
	{
		if ( m_mouseJointId.IsNonNull && button == GLFW_MOUSE_BUTTON_1 )
		{
			DestroyJoint( m_mouseJointId );
			m_mouseJointId = nullJointId;

			DestroyBody( m_mouseBodyId );
			m_mouseBodyId = nullBodyId;
		}
	}

	public virtual void MouseMove( Pos p )
	{
		if ( Joint_IsValid( m_mouseJointId ) == false )
		{
			// The world or attached body was destroyed.
			m_mouseJointId = nullJointId;
		}

		m_mousePoint = p;
	}

	public void DrawScreenTextLine( string text, params object?[] args )
	{
		DrawScreenString( m_draw, m_screenTextX, m_screenTextY, HexColor.colorWhite, "%s", Printf.Fmt( text, args ) );
		m_screenTextY += 1.5f * ImGui.GetFontSize();
	}

	public void ResetProfile()
	{
		m_stepCount = 0;
		Array.Clear( m_profiles );
		m_currentProfileIndex = 0;
		m_profileReadIndex = 0;
		m_profileWriteIndex = 0;
	}

	public virtual void Step()
	{
		m_didStep = false;

		float timeStep = 0.0f;
		if ( m_context.pause == false || m_context.singleStep > 0 )
		{
			timeStep = m_context.hertz > 0.0f ? 1.0f / m_context.hertz : 0.0f;
			m_context.singleStep = MaxInt( 0, m_context.singleStep - 1 );
		}

		if ( m_mouseJointId.IsNonNull && Joint_IsValid( m_mouseJointId ) == false )
		{
			// The world or attached body was destroyed.
			m_mouseJointId = nullJointId;

			if ( m_mouseBodyId.IsNonNull )
			{
				DestroyBody( m_mouseBodyId );
				m_mouseBodyId = nullBodyId;
			}
		}

		if ( m_mouseBodyId.IsNonNull && timeStep > 0.0f )
		{
			bool wake = true;
			Body_SetTargetTransform( m_mouseBodyId, new Transform( m_mousePoint, Rot_identity ), timeStep, wake );
		}

		m_context.debugDraw.drawingBounds = GetViewBounds( m_context.camera );

		World_EnableSleeping( m_worldId, m_context.enableSleep );
		World_EnableWarmStarting( m_worldId, m_context.enableWarmStarting );
		World_EnableContinuous( m_worldId, m_context.enableContinuous );
		World_SetRestitutionIterations( m_worldId, m_context.restitutionIterations );
		World_EnableRestitutionPropagation( m_worldId, m_context.enableRestitutionPropagation );
		World_EnableSSE2Fallback( m_worldId, m_context.enableSSE2Fallback );

		if ( timeStep > 0.0f || m_stepWhilePaused )
		{
			World_Step( m_worldId, timeStep, m_context.subStepCount );
		}

		World_Draw( m_worldId, ref m_context.debugDraw );

		if ( timeStep > 0.0f )
		{
			m_stepCount += 1;
			m_didStep = true;

			if ( m_profileWriteIndex == m_profileCapacity + m_profileReadIndex )
			{
				m_profileReadIndex += 1;
			}

			m_currentProfileIndex = (int)( m_profileWriteIndex & ( m_profileCapacity - 1 ) );
			m_profiles[m_currentProfileIndex] = World_GetProfile( m_worldId );

			m_profileWriteIndex += 1;
		}
	}

	public bool IsProfileVisible()
	{
		return m_context.showUI && m_context.showProfile && HasProfile();
	}

	public float GetProfilePanelWidth()
	{
		if ( IsProfileVisible() == false )
		{
			return 0.0f;
		}

		return 22.0f * ImGui.GetFontSize();
	}

	private static readonly string[] s_profileNames =
	[
		"step", "pairs", "collide", "solve", "  solver setup", "  constraints", "    prepare", "    integrate vel",
		"    warm start", "    solve impulses", "    integrate pos", "    relax", "    restitution", "    store", "  split islands",
		"  transforms", "  sensor hits", "  joint events", "  hit events", "  refit", "  bullets", "  sleep islands", "sensors",
	];

	private static float GetProfileValue( in Profile p, int row )
	{
		return row switch
		{
			0 => p.step,
			1 => p.pairs,
			2 => p.collide,
			3 => p.solve,
			4 => p.solverSetup,
			5 => p.constraints,
			6 => p.prepareConstraints,
			7 => p.integrateVelocities,
			8 => p.warmStart,
			9 => p.solveImpulses,
			10 => p.integratePositions,
			11 => p.relaxImpulses,
			12 => p.restitution,
			13 => p.storeImpulses,
			14 => p.splitIslands,
			15 => p.transforms,
			16 => p.sensorHits,
			17 => p.jointEvents,
			18 => p.hitEvents,
			19 => p.refit,
			20 => p.bullets,
			21 => p.sleepIslands,
			_ => p.sensors,
		};
	}

	// Current, average and max for each profile entry over the history
	public void DrawProfile()
	{
		if ( IsProfileVisible() == false )
		{
			return;
		}

		float fontSize = ImGui.GetFontSize();
		float menuBarHeight = ImGui.GetFrameHeight();
		float panelWidth = GetProfilePanelWidth();

		ImGui.SetNextWindowPos( new Vector2( 0.5f * fontSize, menuBarHeight + 0.5f * fontSize ) );
		ImGui.SetNextWindowSize( new Vector2( panelWidth, m_camera.height - menuBarHeight - fontSize ) );

		ImGui.Begin( "Profile", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse | ImGuiWindowFlags.NoTitleBar );

		int count = (int)( m_profileWriteIndex - m_profileReadIndex );
		if ( ImGui.BeginTable( "profile", 4, ImGuiTableFlags.RowBg | ImGuiTableFlags.SizingStretchProp ) )
		{
			ImGui.TableSetupColumn( "ms" );
			ImGui.TableSetupColumn( "now" );
			ImGui.TableSetupColumn( "avg" );
			ImGui.TableSetupColumn( "max" );
			ImGui.TableHeadersRow();

			for ( int row = 0; row < s_profileNames.Length; ++row )
			{
				float sum = 0.0f, max = 0.0f;
				for ( int i = 0; i < count; ++i )
				{
					int idx = (int)( ( m_profileReadIndex + (ulong)i ) & ( m_profileCapacity - 1 ) );
					float v = GetProfileValue( m_profiles[idx], row );
					sum += v;
					max = MathF.Max( max, v );
				}

				float now = GetProfileValue( m_profiles[m_currentProfileIndex], row );
				ImGui.TableNextRow();
				ImGui.TableSetColumnIndex( 0 );
				ImGui.TextUnformatted( s_profileNames[row] );
				ImGui.TableSetColumnIndex( 1 );
				ImGui.TextUnformatted( now.ToString( "F2", CultureInfo.InvariantCulture ) );
				ImGui.TableSetColumnIndex( 2 );
				ImGui.TextUnformatted( ( count > 0 ? sum / count : 0.0f ).ToString( "F2", CultureInfo.InvariantCulture ) );
				ImGui.TableSetColumnIndex( 3 );
				ImGui.TextUnformatted( max.ToString( "F2", CultureInfo.InvariantCulture ) );
			}

			ImGui.EndTable();
		}

		if ( ImGui.Button( "Reset" ) )
		{
			ResetProfile();
		}

		ImGui.End();
	}

	public void DrawMetrics()
	{
		if ( m_context.showUI == false || m_context.showMetrics == false || m_worldId.IsNull )
		{
			return;
		}

		float fontSize = ImGui.GetFontSize();
		ImGui.SetNextWindowPos( new Vector2( m_screenTextX, m_camera.height - 16.0f * fontSize ), ImGuiCond.FirstUseEver );
		ImGui.SetNextWindowSize( new Vector2( 24.0f * fontSize, 15.0f * fontSize ), ImGuiCond.FirstUseEver );
		bool open = true;
		if ( ImGui.Begin( "Metrics", ref open ) )
		{
			if ( ImGui.BeginTabBar( "MetricsTabs" ) )
			{
				if ( ImGui.BeginTabItem( "World" ) )
				{
					Counters s = World_GetCounters( m_worldId );
					ImGui.Text( Printf.Fmt( "bodies/shapes/contacts/joints = %d/%d/%d/%d", s.bodyCount, s.shapeCount, s.contactCount, s.jointCount ) );
					ImGui.Text( Printf.Fmt( "awake contacts = %d, recycled = %d", s.awakeContactCount, s.recycledContactCount ) );
					ImGui.Text( Printf.Fmt( "islands/tasks = %d/%d", s.islandCount, s.taskCount ) );
					ImGui.Text( Printf.Fmt( "tree height static/movable = %d/%d", s.staticTreeHeight, s.treeHeight ) );
					ImGui.Text( Printf.Fmt( "stack allocator size = %d K", s.stackUsed / 1024 ) );
					ImGui.Text( Printf.Fmt( "total allocation = %d K", s.byteCount / 1024 ) );

					string colors = "";
					for ( int i = 0; i < GraphColorCount; ++i )
					{
						colors += s.colorCounts[i].ToString( CultureInfo.InvariantCulture ) + ( i < GraphColorCount - 1 ? " " : "" );
					}

					ImGui.TextWrapped( "graph colors: " + colors );
					ImGui.EndTabItem();
				}

				DrawMetricsTab();
				ImGui.EndTabBar();
			}
		}

		ImGui.End();
		m_context.showMetrics = open;
	}

	public void DrawHud( float frameTime )
	{
		SampleEntry entry = SampleApp.g_sampleEntries[m_context.sampleIndex];
		float fontSize = ImGui.GetFontSize();

		DrawScreenString( m_context.draw, 5.0f, 1.5f * fontSize, HexColor.colorYellow, "%s : %s", entry.category, entry.name );
		DrawScreenString( m_context.draw, 5.0f, 3.0f * fontSize, HexColor.colorForestGreen, "Press TAB to show UI" );
		m_screenTextY += 1.5f * fontSize;

		if ( m_context.pause )
		{
			DrawScreenString( m_context.draw, 5.0f, 4.5f * fontSize, HexColor.colorRed, "****PAUSED****" );
			m_screenTextY += 1.5f * fontSize;
		}

		DrawScreenString( m_context.draw, 5.0f, m_camera.height - 1.5f * fontSize, HexColor.colorSeaGreen, "%.1f ms  step %d",
						  1000.0f * frameTime, m_stepCount );
	}

	// Parse an SVG path element with only straight lines. Removes the loop point.
	public static int ParsePath( string svgPath, Vec2 offset, Span<Vec2> points, int capacity, float scale, bool reverseOrder )
	{
		int pointCount = 0;
		Vec2 currentPoint = default;
		int ptr = 0;
		char command = svgPath.Length > 0 ? svgPath[0] : '\0';

		float ReadFloat()
		{
			int start = ptr;
			while ( ptr < svgPath.Length && ( char.IsDigit( svgPath[ptr] ) || svgPath[ptr] is '-' or '.' or 'e' or '+' ) )
			{
				ptr++;
			}

			return float.Parse( svgPath.AsSpan( start, ptr - start ), CultureInfo.InvariantCulture );
		}

		while ( ptr < svgPath.Length )
		{
			if ( char.IsDigit( svgPath[ptr] ) == false && svgPath[ptr] != '-' )
			{
				// note: command can be implicitly repeated
				command = svgPath[ptr];

				if ( command is 'M' or 'L' or 'H' or 'V' or 'm' or 'l' or 'h' or 'v' )
				{
					ptr += 2; // Skip the command character and space
				}

				if ( command == 'z' )
				{
					break;
				}
			}

			int save = ptr;
			float x, y;
			switch ( command )
			{
				case 'M':
				case 'L':
					x = ReadFloat();
					ptr++;
					y = ReadFloat();
					currentPoint = new Vec2( x, y );
					break;
				case 'H':
					currentPoint.x = ReadFloat();
					break;
				case 'V':
					currentPoint.y = ReadFloat();
					break;
				case 'm':
				case 'l':
					x = ReadFloat();
					ptr++;
					y = ReadFloat();
					currentPoint.x += x;
					currentPoint.y += y;
					break;
				case 'h':
					currentPoint.x += ReadFloat();
					break;
				case 'v':
					currentPoint.y += ReadFloat();
					break;
				default:
					throw new FormatException( "unsupported svg path command" );
			}

			ptr = save;

			points[pointCount] = new Vec2( scale * ( currentPoint.x + offset.x ), -scale * ( currentPoint.y + offset.y ) );
			pointCount += 1;
			if ( pointCount == capacity )
			{
				break;
			}

			// Move to the next space or end of string
			while ( ptr < svgPath.Length && char.IsWhiteSpace( svgPath[ptr] ) == false )
			{
				ptr++;
			}

			// Skip contiguous spaces
			while ( ptr < svgPath.Length && char.IsWhiteSpace( svgPath[ptr] ) )
			{
				ptr++;
			}
		}

		if ( pointCount == 0 )
		{
			return 0;
		}

		// Remove the loop point for chain shapes.
		if ( pointCount > 2 && Distance( points[0], points[pointCount - 1] ) <= LinearSlop )
		{
			pointCount -= 1;
		}

		return pointCount;
	}
}

// The sample registry and the user interface from sample.cpp
public static partial class SampleApp
{
	public const float INFO_PANEL_WIDTH = 19.0f;

	public static readonly List<SampleEntry> g_sampleEntries = new();

	public static int g_sampleCount => g_sampleEntries.Count;

	// Arrays of the sample's class typed records, which are value types in C++
	public static T[] NewArray<T>( int count ) where T : new()
	{
		T[] array = new T[count];
		for ( int i = 0; i < count; ++i )
		{
			array[i] = new T();
		}

		return array;
	}

	public static uint IM_COL32( int r, int g, int b, int a ) => (uint)( r | ( g << 8 ) | ( b << 16 ) | ( a << 24 ) );

	public static int RegisterSample( string category, string name, SampleCreateFcn fcn )
	{
		g_sampleEntries.Add( new SampleEntry { category = category, name = name, createFcn = fcn } );
		return g_sampleEntries.Count - 1;
	}

	public static int RegisterSampleWithCapacity( string category, string name, SampleCreateFcn fcn, SampleCapacityFcn capacityFcn )
	{
		g_sampleEntries.Add( new SampleEntry { category = category, name = name, createFcn = fcn, capacityFcn = capacityFcn } );
		return g_sampleEntries.Count - 1;
	}

	public static void SortSamples()
	{
		g_sampleEntries.Sort( ( a, b ) =>
		{
			int result = string.CompareOrdinal( a.category, b.category );
			return result != 0 ? result : string.CompareOrdinal( a.name, b.name );
		} );
	}

	public static void SelectSample( SampleContext context, int selection, bool restart )
	{
		if ( restart == false )
		{
			ResetView( context.camera );
			context.sampleIndex = selection;
			context.pause = false;
			context.subStepCount = 4;
			context.restitutionIterations = 2;
			context.enableRestitutionPropagation = false;
			context.debugDraw.drawJoints = true;
		}

		// Steps queued in a sample that never consumes them must not play out in the next one
		context.singleStep = 0;

		context.sample?.Dispose();
		context.sample = null;
		SampleEntry entry = g_sampleEntries[context.sampleIndex];
		context.capacity = entry.capacityFcn != null ? entry.capacityFcn() : DefaultWorldDef().capacity;
		context.restart = restart;
		context.sample = entry.createFcn( context );
		context.restart = false;

		// A restart keeps the camera where it was, so the original starting view stays home
		if ( restart == false )
		{
			context.homeCenter = context.camera.center;
			context.homeZoom = context.camera.zoom;
		}
	}

	private static void DrawRow( string key, string desc )
	{
		ImGui.TableNextRow();
		ImGui.TableSetColumnIndex( 0 );
		ImGui.TextUnformatted( key );
		ImGui.TableSetColumnIndex( 1 );
		ImGui.TextUnformatted( desc );
	}

	private static void DrawMenuBar( SampleContext context )
	{
		if ( ImGui.BeginMainMenuBar() == false )
		{
			return;
		}

		if ( ImGui.BeginMenu( "Samples" ) )
		{
			if ( ImGui.MenuItem( "Find...", "Ctrl+O" ) )
			{
				context.openSamplePicker = true;
			}

			ImGui.Separator();

			string? category = null;
			bool open = false;
			for ( int i = 0; i < g_sampleCount; ++i )
			{
				SampleEntry entry = g_sampleEntries[i];
				if ( entry.category != category )
				{
					if ( open )
					{
						ImGui.EndMenu();
					}

					category = entry.category;
					open = ImGui.BeginMenu( category );
				}

				if ( open && ImGui.MenuItem( entry.name, "", i == context.sampleIndex ) )
				{
					SelectSample( context, i, false );
				}
			}

			if ( open )
			{
				ImGui.EndMenu();
			}

			ImGui.EndMenu();
		}

		if ( ImGui.BeginMenu( "View" ) )
		{
			ImGui.MenuItem( "Profile", "I", ref context.showProfile );
			ImGui.MenuItem( "Metrics", "M", ref context.showMetrics );
			ImGui.MenuItem( "Controls", "?", ref context.showControls );
			ImGui.Separator();
			if ( ImGui.MenuItem( "Home", "Home" ) )
			{
				context.sample?.FocusHome();
			}

			ImGui.EndMenu();
		}

		if ( ImGui.BeginMenu( "Draw" ) )
		{
			ref DebugDraw d = ref context.debugDraw;
			ImGui.MenuItem( "Shapes", "", ref d.drawShapes );
			ImGui.MenuItem( "Joints", "", ref d.drawJoints );
			ImGui.MenuItem( "Joint Extras", "", ref d.drawJointExtras );
			ImGui.MenuItem( "Bounds", "", ref d.drawBounds );
			ImGui.MenuItem( "Contact Points", "", ref d.drawContacts );
			ImGui.MenuItem( "Contact Normals", "", ref d.drawContactNormals );
			ImGui.MenuItem( "Contact Features", "", ref d.drawContactFeatures );
			ImGui.MenuItem( "Contact Forces", "", ref d.drawContactForces );
			ImGui.MenuItem( "Friction Forces", "", ref d.drawFrictionForces );
			ImGui.MenuItem( "Mass", "", ref d.drawMass );
			ImGui.MenuItem( "Body Names", "", ref d.drawBodyNames );
			ImGui.MenuItem( "Graph Colors", "", ref d.drawGraphColors );
			ImGui.MenuItem( "Islands", "", ref d.drawIslands );
			ImGui.MenuItem( "Chain Normals", "", ref d.drawChainNormals );
			ImGui.EndMenu();
		}

		ImGui.EndMainMenuBar();

		if ( context.showControls )
		{
			bool open = true;
			ImGui.SetNextWindowSize( new Vector2( 22.0f * ImGui.GetFontSize(), 0.0f ), ImGuiCond.FirstUseEver );
			if ( ImGui.Begin( "Controls", ref open, ImGuiWindowFlags.AlwaysAutoResize ) )
			{
				if ( ImGui.BeginTable( "keys", 2 ) )
				{
					DrawRow( "Left drag", "Move a body" );
					DrawRow( "Right drag", "Pan" );
					DrawRow( "Wheel / Z X", "Zoom" );
					DrawRow( "Arrows", "Pan" );
					DrawRow( "Home", "Return to the start view" );
					DrawRow( "P", "Pause" );
					DrawRow( ". (Shift)", "Step forward 1 (5)" );
					DrawRow( "R", "Restart" );
					DrawRow( "[ ]", "Previous / next sample" );
					DrawRow( "Ctrl+O", "Find a sample" );
					DrawRow( "Tab", "Show / hide the UI" );
					DrawRow( "I", "Profile" );
					DrawRow( "M", "Metrics" );
					DrawRow( "Ctrl+Q", "Quit" );
					ImGui.EndTable();
				}
			}

			ImGui.End();
			context.showControls = open;
		}
	}

	private static string s_pickerQuery = "";
	private static int s_pickerSelection;

	// Case-insensitive subsequence match. Returns >=0 score on match, -1 on no match.
	private static int FuzzyScore( string needle, string haystack )
	{
		if ( needle.Length == 0 )
		{
			return 0;
		}

		int score = 0, h = 0, last = -2;
		foreach ( char nc in needle )
		{
			char n = char.ToLowerInvariant( nc );
			while ( h < haystack.Length && char.ToLowerInvariant( haystack[h] ) != n )
			{
				h++;
			}

			if ( h == haystack.Length )
			{
				return -1;
			}

			// contiguous matches and word starts score higher
			score += h == last + 1 ? 3 : 1;
			if ( h == 0 || haystack[h - 1] == ' ' )
			{
				score += 2;
			}

			last = h;
			h++;
		}

		return score;
	}

	private static void DrawSamplePicker( SampleContext context )
	{
		if ( context.openSamplePicker )
		{
			ImGui.OpenPopup( "Find Sample" );
			context.openSamplePicker = false;
			s_pickerQuery = "";
			s_pickerSelection = 0;
		}

		ImGui.SetNextWindowSize( new Vector2( 28.0f * ImGui.GetFontSize(), 24.0f * ImGui.GetFontSize() ), ImGuiCond.Appearing );
		if ( ImGui.BeginPopupModal( "Find Sample" ) == false )
		{
			return;
		}

		if ( ImGui.IsWindowAppearing() )
		{
			ImGui.SetKeyboardFocusHere();
		}

		ImGui.InputText( "##query", ref s_pickerQuery, 128 );

		List<(int score, int index)> matches = new();
		for ( int i = 0; i < g_sampleCount; ++i )
		{
			SampleEntry entry = g_sampleEntries[i];
			int score = FuzzyScore( s_pickerQuery, entry.category + " " + entry.name );
			if ( score >= 0 )
			{
				matches.Add( ( score, i ) );
			}
		}

		matches.Sort( ( a, b ) => b.score != a.score ? b.score.CompareTo( a.score ) : a.index.CompareTo( b.index ) );
		s_pickerSelection = Math.Clamp( s_pickerSelection, 0, Math.Max( 0, matches.Count - 1 ) );

		if ( ImGui.IsKeyPressed( ImGuiKey.DownArrow ) )
		{
			s_pickerSelection = Math.Min( s_pickerSelection + 1, matches.Count - 1 );
		}

		if ( ImGui.IsKeyPressed( ImGuiKey.UpArrow ) )
		{
			s_pickerSelection = Math.Max( s_pickerSelection - 1, 0 );
		}

		int chosen = -1;
		if ( ImGui.BeginChild( "matches" ) )
		{
			for ( int m = 0; m < matches.Count; ++m )
			{
				SampleEntry entry = g_sampleEntries[matches[m].index];
				if ( ImGui.Selectable( entry.category + " / " + entry.name, m == s_pickerSelection ) )
				{
					chosen = matches[m].index;
				}
			}
		}

		ImGui.EndChild();

		if ( ImGui.IsKeyPressed( ImGuiKey.Enter ) && matches.Count > 0 )
		{
			chosen = matches[s_pickerSelection].index;
		}

		if ( chosen >= 0 )
		{
			SelectSample( context, chosen, false );
			ImGui.CloseCurrentPopup();
		}
		else if ( ImGui.IsKeyPressed( ImGuiKey.Escape ) )
		{
			ImGui.CloseCurrentPopup();
		}

		ImGui.EndPopup();
	}

	private static void HelpMarker( string text )
	{
		ImGui.SameLine( 0.0f, ImGui.GetStyle().ItemInnerSpacing.X );
		ImGui.TextDisabled( "(?)" );
		ImGui.SetItemTooltip( text );
	}

	private static void DrawInfoPanel( SampleContext context, float frameTime )
	{
		SampleEntry entry = g_sampleEntries[context.sampleIndex];
		Sample sample = context.sample!;
		float fontSize = ImGui.GetFontSize();
		float menuWidth = sample.InfoPanelWidthEm() * fontSize;
		float menuBarHeight = ImGui.GetFrameHeight();

		ImGui.SetNextWindowPos( new Vector2( context.camera.width - menuWidth - 0.5f * fontSize, menuBarHeight + 0.5f * fontSize ) );
		ImGui.SetNextWindowSize( new Vector2( menuWidth, context.camera.height - menuBarHeight - fontSize ) );

		ImGui.Begin( "Info", ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse | ImGuiWindowFlags.NoTitleBar );

		ImGui.TextColored( MakeColor( HexColor.colorGoldenRod ), entry.name );
		ImGui.TextColored( MakeColor( HexColor.colorLightGray ), entry.category );
		ImGui.Separator();

		if ( context.pause )
		{
			ImGui.TextColored( MakeColor( HexColor.colorRed ), "PAUSED" );
			ImGui.SameLine();
			ImGui.TextDisabled( "(P)" );
			ImGui.Separator();
		}

		ImGui.TextColored( MakeColor( HexColor.colorSeaGreen ), Printf.Fmt( "frame %.1f ms", 1000.0f * frameTime ) );
		ImGui.TextColored( MakeColor( HexColor.colorSeaGreen ), Printf.Fmt( "step %d", sample.m_stepCount ) );
		ImGui.Separator();
		ImGui.TextColored( MakeColor( HexColor.colorSeaGreen ), Printf.Fmt( "cam (%.1f, %.1f)", context.camera.center.x, context.camera.center.y ) );
		ImGui.TextColored( MakeColor( HexColor.colorSeaGreen ), Printf.Fmt( "zoom %.2f", context.camera.zoom ) );

		ImGui.Separator();

		ImGui.PushItemWidth( 6.0f * fontSize );
		if ( sample.DrawControls() )
		{
			ImGui.Separator();
		}

		ImGui.PopItemWidth();

		// DrawControls may have restarted the sample
		sample = context.sample!;

		if ( sample.HasSolverControls() && ImGui.CollapsingHeader( "Solver", ImGuiTreeNodeFlags.DefaultOpen ) )
		{
			ImGui.PushItemWidth( 6.0f * fontSize );

			ImGui.SliderInt( "Sub-steps##Solver", ref context.subStepCount, 1, 32 );
			HelpMarker( "The solver breaks the full step into several sub-steps.\nMore sub-steps usually lead to more accurate results." );

			ImGui.SliderInt( "Bounce Iters##Solver", ref context.restitutionIterations, 0, 8 );
			HelpMarker( "Iterations for the restitution solver." );

			ImGui.SliderFloat( "Hertz##Solver", ref context.hertz, 5.0f, 240.0f, "%.0f Hz" );
			HelpMarker( "The number of world steps per second." );

			if ( ImGui.SliderInt( "Workers##Solver", ref context.workerCount, 1, MaxWorkers ) )
			{
				context.workerCount = ClampInt( context.workerCount, 1, MaxWorkers );
				SelectSample( context, context.sampleIndex, true );
				sample = context.sample!;
			}

			HelpMarker( "The number worker threads used by the world step." );

			float recyclingCentimeters = 100.0f * context.recycleDistance;
			if ( ImGui.SliderFloat( "Recycle##Solver", ref recyclingCentimeters, 0.0f, 10.0f, "%.1f cm" ) )
			{
				context.recycleDistance = 0.01f * recyclingCentimeters;
				World_SetContactRecycleDistance( sample.m_worldId, context.recycleDistance );
			}

			HelpMarker( "The contact recycling distance tolerance.\nSet to zero to disable recycling." );

			ImGui.PopItemWidth();

			ImGui.Checkbox( "Sleep##Solver", ref context.enableSleep );
			HelpMarker( "Allow bodies to sleep, reducing simulation CPU cost." );

			ImGui.Checkbox( "Warm Starting##Solver", ref context.enableWarmStarting );
			HelpMarker( "Enable solver warm starting which usually improves stacking stability." );

			ImGui.Checkbox( "Continuous##Solver", ref context.enableContinuous );
			HelpMarker( "Enable continuous collision detection." );

			ImGui.Checkbox( "Bounce Propagation##Solver", ref context.enableRestitutionPropagation );
			HelpMarker( "Enable restitution solver propagation across all touching contacts points" );
		}

		ImGui.End();
	}

	// Entry point for all UI drawing. This can delete the current sample.
	public static void DrawUI( SampleContext context, float frameTime )
	{
		DrawMenuBar( context );
		DrawSamplePicker( context );
		DrawInfoPanel( context, frameTime );
		context.sample!.DrawProfile();
		context.sample!.DrawMetrics();
	}
}
