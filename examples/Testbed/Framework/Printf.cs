// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Globalization;
using System.Text;

namespace Box2DNet.Testbed;

// A small printf so the sample text keeps its C format strings.
public static class Printf
{
	public static string Fmt( string format, params object?[] args )
	{
		if ( args.Length == 0 )
		{
			return format.Replace( "%%", "%" );
		}

		StringBuilder sb = new();
		int argIndex = 0;
		int i = 0;
		while ( i < format.Length )
		{
			char c = format[i];
			if ( c != '%' )
			{
				sb.Append( c );
				i += 1;
				continue;
			}

			i += 1;
			if ( i < format.Length && format[i] == '%' )
			{
				sb.Append( '%' );
				i += 1;
				continue;
			}

			// flags
			bool leftAlign = false, plus = false, zeroPad = false, space = false;
			while ( i < format.Length && "-+0 #".IndexOf( format[i] ) >= 0 )
			{
				switch ( format[i] )
				{
					case '-': leftAlign = true; break;
					case '+': plus = true; break;
					case '0': zeroPad = true; break;
					case ' ': space = true; break;
				}

				i += 1;
			}

			int width = 0;
			if ( i < format.Length && format[i] == '*' )
			{
				width = Convert.ToInt32( args[argIndex++], CultureInfo.InvariantCulture );
				i += 1;
			}
			else
			{
				while ( i < format.Length && char.IsDigit( format[i] ) )
				{
					width = 10 * width + ( format[i] - '0' );
					i += 1;
				}
			}

			int precision = -1;
			if ( i < format.Length && format[i] == '.' )
			{
				i += 1;
				precision = 0;
				if ( i < format.Length && format[i] == '*' )
				{
					precision = Convert.ToInt32( args[argIndex++], CultureInfo.InvariantCulture );
					i += 1;
				}
				else
				{
					while ( i < format.Length && char.IsDigit( format[i] ) )
					{
						precision = 10 * precision + ( format[i] - '0' );
						i += 1;
					}
				}
			}

			// length modifiers
			while ( i < format.Length && "hlLqjzt".IndexOf( format[i] ) >= 0 )
			{
				i += 1;
			}

			if ( i >= format.Length )
			{
				break;
			}

			char conversion = format[i];
			i += 1;
			object? arg = argIndex < args.Length ? args[argIndex++] : null;
			string text = FormatOne( conversion, precision, arg );

			if ( ( plus || space ) && "dif".IndexOf( conversion ) >= 0 && text.StartsWith( '-' ) == false )
			{
				text = ( plus ? "+" : " " ) + text;
			}

			if ( text.Length < width )
			{
				if ( leftAlign )
				{
					text = text.PadRight( width );
				}
				else if ( zeroPad && "diuxXfFeEgG".IndexOf( conversion ) >= 0 )
				{
					bool negative = text.StartsWith( '-' ) || text.StartsWith( '+' );
					string sign = negative ? text[..1] : "";
					text = sign + text[sign.Length..].PadLeft( width - sign.Length, '0' );
				}
				else
				{
					text = text.PadLeft( width );
				}
			}

			sb.Append( text );
		}

		return sb.ToString();
	}

	private static string FormatOne( char conversion, int precision, object? arg )
	{
		CultureInfo ci = CultureInfo.InvariantCulture;
		switch ( conversion )
		{
			case 'd':
			case 'i':
			case 'u':
				return Convert.ToInt64( arg is Enum ? Convert.ChangeType( arg, Enum.GetUnderlyingType( arg.GetType() ), ci ) : arg, ci )
					.ToString( ci );
			case 'x':
				return ToUInt64( arg ).ToString( "x", ci );
			case 'X':
				return ToUInt64( arg ).ToString( "X", ci );
			case 'f':
			case 'F':
				return Convert.ToDouble( arg, ci ).ToString( "F" + ( precision < 0 ? 6 : precision ), ci );
			case 'e':
			case 'E':
				return Convert.ToDouble( arg, ci ).ToString( ( conversion == 'e' ? "0." : "0." ) + new string( '0', precision < 0 ? 6 : precision ) + "e+00", ci );
			case 'g':
			case 'G':
				return Convert.ToDouble( arg, ci ).ToString( "G" + ( precision < 0 ? 6 : Math.Max( precision, 1 ) ), ci );
			case 'c':
				return arg is char ch ? ch.ToString() : ( (char)Convert.ToInt32( arg, ci ) ).ToString();
			case 's':
			{
				string s = arg?.ToString() ?? "(null)";
				return precision >= 0 && s.Length > precision ? s[..precision] : s;
			}
			case 'p':
				return "0x" + ToUInt64( arg ).ToString( "x16", ci );
			default:
				return "%" + conversion;
		}
	}

	private static ulong ToUInt64( object? arg )
	{
		return arg switch
		{
			int v => unchecked( (uint)v ),
			long v => unchecked( (ulong)v ),
			short v => unchecked( (ushort)v ),
			sbyte v => unchecked( (byte)v ),
			null => 0,
			_ => Convert.ToUInt64( arg, CultureInfo.InvariantCulture ),
		};
	}
}
