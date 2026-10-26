// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using Hexa.NET.GLFW;

namespace Box2DNet.Testbed;

// The GLFW constants and functions the samples use, with their C names.
public static class Glfw
{
	public const int GLFW_RELEASE = 0;
	public const int GLFW_PRESS = 1;
	public const int GLFW_REPEAT = 2;

	public const int GLFW_MOD_SHIFT = 0x0001;
	public const int GLFW_MOD_CONTROL = 0x0002;
	public const int GLFW_MOD_ALT = 0x0004;
	public const int GLFW_MOD_SUPER = 0x0008;

	public const int GLFW_MOUSE_BUTTON_1 = 0;
	public const int GLFW_MOUSE_BUTTON_2 = 1;
	public const int GLFW_MOUSE_BUTTON_3 = 2;
	public const int GLFW_MOUSE_BUTTON_LEFT = GLFW_MOUSE_BUTTON_1;
	public const int GLFW_MOUSE_BUTTON_RIGHT = GLFW_MOUSE_BUTTON_2;
	public const int GLFW_MOUSE_BUTTON_MIDDLE = GLFW_MOUSE_BUTTON_3;

	public const int GLFW_KEY_SPACE = 32;
	public const int GLFW_KEY_APOSTROPHE = 39;
	public const int GLFW_KEY_COMMA = 44;
	public const int GLFW_KEY_MINUS = 45;
	public const int GLFW_KEY_PERIOD = 46;
	public const int GLFW_KEY_SLASH = 47;
	public const int GLFW_KEY_0 = 48;
	public const int GLFW_KEY_1 = 49;
	public const int GLFW_KEY_2 = 50;
	public const int GLFW_KEY_3 = 51;
	public const int GLFW_KEY_4 = 52;
	public const int GLFW_KEY_5 = 53;
	public const int GLFW_KEY_6 = 54;
	public const int GLFW_KEY_7 = 55;
	public const int GLFW_KEY_8 = 56;
	public const int GLFW_KEY_9 = 57;
	public const int GLFW_KEY_SEMICOLON = 59;
	public const int GLFW_KEY_EQUAL = 61;
	public const int GLFW_KEY_A = 65;
	public const int GLFW_KEY_B = 66;
	public const int GLFW_KEY_C = 67;
	public const int GLFW_KEY_D = 68;
	public const int GLFW_KEY_E = 69;
	public const int GLFW_KEY_F = 70;
	public const int GLFW_KEY_G = 71;
	public const int GLFW_KEY_H = 72;
	public const int GLFW_KEY_I = 73;
	public const int GLFW_KEY_J = 74;
	public const int GLFW_KEY_K = 75;
	public const int GLFW_KEY_L = 76;
	public const int GLFW_KEY_M = 77;
	public const int GLFW_KEY_N = 78;
	public const int GLFW_KEY_O = 79;
	public const int GLFW_KEY_P = 80;
	public const int GLFW_KEY_Q = 81;
	public const int GLFW_KEY_R = 82;
	public const int GLFW_KEY_S = 83;
	public const int GLFW_KEY_T = 84;
	public const int GLFW_KEY_U = 85;
	public const int GLFW_KEY_V = 86;
	public const int GLFW_KEY_W = 87;
	public const int GLFW_KEY_X = 88;
	public const int GLFW_KEY_Y = 89;
	public const int GLFW_KEY_Z = 90;
	public const int GLFW_KEY_LEFT_BRACKET = 91;
	public const int GLFW_KEY_BACKSLASH = 92;
	public const int GLFW_KEY_RIGHT_BRACKET = 93;
	public const int GLFW_KEY_GRAVE_ACCENT = 96;
	public const int GLFW_KEY_ESCAPE = 256;
	public const int GLFW_KEY_ENTER = 257;
	public const int GLFW_KEY_TAB = 258;
	public const int GLFW_KEY_BACKSPACE = 259;
	public const int GLFW_KEY_INSERT = 260;
	public const int GLFW_KEY_DELETE = 261;
	public const int GLFW_KEY_RIGHT = 262;
	public const int GLFW_KEY_LEFT = 263;
	public const int GLFW_KEY_DOWN = 264;
	public const int GLFW_KEY_UP = 265;
	public const int GLFW_KEY_PAGE_UP = 266;
	public const int GLFW_KEY_PAGE_DOWN = 267;
	public const int GLFW_KEY_HOME = 268;
	public const int GLFW_KEY_END = 269;
	public const int GLFW_KEY_PAUSE = 284;
	public const int GLFW_KEY_F1 = 290;
	public const int GLFW_KEY_LEFT_SHIFT = 340;
	public const int GLFW_KEY_LEFT_CONTROL = 341;
	public const int GLFW_KEY_LEFT_ALT = 342;
	public const int GLFW_KEY_RIGHT_SHIFT = 344;
	public const int GLFW_KEY_RIGHT_CONTROL = 345;
	public const int GLFW_KEY_RIGHT_ALT = 346;

	// C code tests the key state both as a number and as a condition
	public readonly struct KeyState
	{
		private readonly int m_value;

		public KeyState( int value )
		{
			m_value = value;
		}

		public static implicit operator int( KeyState state ) => state.m_value;
		public static implicit operator bool( KeyState state ) => state.m_value != 0;
	}

	public static KeyState glfwGetKey( GLFWwindowPtr window, int key ) => new( GLFW.GetKey( window, key ) );

	public static int glfwGetMouseButton( GLFWwindowPtr window, int button ) => GLFW.GetMouseButton( window, button );

	public static double glfwGetTime() => GLFW.GetTime();
}
