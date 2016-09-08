#region Header
// --------------------------------------------------------------------------
//    Icarus Scene Engine
//    Copyright (C) 2005-2007  Euan D. MacInnes. All Rights Reserved.

//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
// --------------------------------------------------------------------------
//
//     UNIT               : FTclass.cs
//     SUMMARY            : Tao.FreeType OpenGL wrapper for uploading FreeType fonts to
//                        : OpenGL
//                        : 
//                        : 
//
//     PRINCIPLE AUTHOR   : Euan D. MacInnes
//     
#endregion Header
#region Revisions
// --------------------------------------------------------------------------
//     REVISIONS/NOTES
//        dd-mm-yyyy    By          Revision Summary
//
// --------------------------------------------------------------------------
#endregion Revisions


using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Runtime.InteropServices;
using SharpFont;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;
namespace ISE
{
	/// <summary>
	/// Glyph offset information for advanced rendering and/or conversions.
	/// </summary>
	public struct FTGlyphOffset
	{
		/// <summary>
		/// Width of the Glyph, in pixels.
		/// </summary>
		public int width;
		/// <summary>
		/// height of the Glyph, in pixels. Represents the number of scanlines
		/// </summary>
		public int height;
		/// <summary>
		/// For Bitmap-generated fonts, this is the top-bearing expressed in integer pixels.
		/// This is the distance from the baseline to the topmost Glyph scanline, upwards Y being positive.
		/// </summary>
		public int top;
		/// <summary>
		/// For Bitmap-generated fonts, this is the left-bearing expressed in integer pixels
		/// </summary>
		public int left;
		/// <summary>
		/// This is the transformed advance width for the glyph.
		/// </summary>
		public FTVector26Dot6 advance;
		/// <summary>
		/// The difference between hinted and unhinted left side bearing while autohinting is active. 0 otherwise.
		/// </summary>
		public long lsb_delta;
		/// <summary>
		/// The difference between hinted and unhinted right side bearing while autohinting is active. 0 otherwise.
		/// </summary>
		public long rsb_delta;
		/// <summary>
		/// The advance width of the unhinted glyph. Its value is expressed in 16.16 fractional pixels, unless FT_LOAD_LINEAR_DESIGN is set when loading the glyph. This field can be important to perform correct WYSIWYG layout. Only relevant for outline glyphs.
		/// </summary>
		public Fixed16Dot16 linearHoriAdvance;
		/// <summary>
		/// The advance height of the unhinted glyph. Its value is expressed in 16.16 fractional pixels, unless FT_LOAD_LINEAR_DESIGN is set when loading the glyph. This field can be important to perform correct WYSIWYG layout. Only relevant for outline glyphs.
		/// </summary>
		public Fixed16Dot16 linearVertAdvance;
	}

	/// <summary>
	/// For internal use, to represent the type of conversion to apply to the font
	/// </summary>
	public enum FTFontType
	{
		/// <summary>
		/// Font has not been initialised yet
		/// </summary>
		FT_NotInitialised,
		/// <summary>
		/// Font was converted to a series of Textures
		/// </summary>
		FT_Texture,
		/// <summary>
		/// Font was converted to a big texture map, representing a collection of glyphs
		/// </summary>
		FT_TextureMap,
		/// <summary>
		/// Font was converted to outlines and stored as display lists
		/// </summary>
		FT_Outline,
		/// <summary>
		/// Font was convered to Outliens and stored as Vertex Buffer Objects
		/// </summary>
		FT_OutlineVBO
	}

	/// <summary>
	/// Alignment of output text
	/// </summary> 
	public enum FTFontAlign
	{
		/// <summary>
		/// Left-align the text when it is drawn
		/// </summary>
		FT_ALIGN_LEFT,
		/// <summary>
		/// Center-align the text when it is drawn
		/// </summary>
		FT_ALIGN_CENTERED,
		/// <summary>
		/// Right-align the text when it is drawn
		/// </summary>
		FT_ALIGN_RIGHT
	}

	/// <summary>
	/// Font class wraper for displaying FreeType fonts in OpenGL.
	/// </summary>
	public class FTFont
	{
		//Public members        
		private int list_base;
		private int font_size = 48;
		private int[] textures;
		private int[] extent_x;
		private FTGlyphOffset[] offsets;

		// Global FreeType library pointer
		private static System.IntPtr libptr;
        private System.IntPtr faceptr;

        static Face sharpFace;
        static Library sharpLib;

        // debug variable used to list the state of all characters rendered
        private string sChars = "";

		/// <summary>
		/// Initialise the FreeType library
		/// </summary>
		/// <returns></returns>
		public static int ftInit()
		{
			// We begin by creating a library pointer            
			if (libptr == IntPtr.Zero)
			{
                try
                {
                    sharpLib = new Library();
                    libptr = IntPtr.Add(libptr, 1);
                    Console.WriteLine("FreeType loaded. Version " + ftVersionString());
                }
                catch(Exception ex)
                {
                    Console.WriteLine("Failed to start FreeType\n" + ex.Message);
                    return -1;
                }
				//int ret = FT.FT_Init_FreeType(out libptr);

				//if (ret != 0)
				//{
				//	Console.WriteLine("Failed to start FreeType");
				//}
				//else
				//	Console.WriteLine("FreeType loaded. Version " + ftVersionString());
				return 0;
			}

			return 0;
		}

		/// <summary>
		/// Font alignment public parameter
		/// </summary>		
		public FTFontAlign FT_ALIGN = FTFontAlign.FT_ALIGN_LEFT;

		/// <summary>
		/// Initialise the Font. Will Initialise the freetype library if not already done so
		/// </summary>
		/// <param name="resourcefilename">Path to the external font file</param>
		/// <param name="Success">Returns 0 if successful</param>
		public FTFont(string resourcefilename, out int Success)
		{
			Success = ftInit();

			if (libptr == IntPtr.Zero) { Console.WriteLine("Couldn't start FreeType"); Success = -1; return; }

			string fontfile = resourcefilename;
            try
            {
                sharpFace = new Face(sharpLib, fontfile);
            }
            catch(Exception)
            {
                Console.WriteLine("Failed to find font " + fontfile);
                Success = -1;
                return;
            }
            
            //Once we have the library we create and load the font face                       
   //         int retb = sharpFace(libptr, fontfile, 0, out faceptr);
			//if (retb != 0)
			//{
			//	Console.WriteLine("Failed to find font " + fontfile);
			//	Success = retb;
			//	return;
			//}

			Success = 0;
		}

		/// <summary>
		/// Return the version information for FreeType.
		/// </summary>
		/// <param name="Major">Major Version</param>
		/// <param name="Minor">Minor Version</param>
		/// <param name="Patch">Patch Number</param>
		public static void ftVersion(ref int Major, ref int Minor, ref int Patch)
		{
			ftInit();
            Version ver = sharpLib.Version;
            Major = ver.Major;
            Minor = ver.Minor;
            Patch = ver.Revision;
        }

		/// <summary>
		/// Return the entire version information for FreeType as a String.
		/// </summary>
		/// <returns></returns>
		public static string ftVersionString()
		{
			int major = 0;
			int minor = 0;
			int patch = 0;
			ftVersion(ref major, ref minor, ref patch);
			return major.ToString() + "." + minor.ToString() + "." + patch.ToString();
		}

		/// <summary>
		/// Render the font to a series of OpenGL textures (one per letter)
		/// </summary>
		/// <param name="fontsize">size of the font</param>
		/// <param name="DPI">dots-per-inch setting</param>
		public void ftRenderToTexture(int fontsize, uint DPI)
		{
			font_size = fontsize;

			//face = (FT_FaceRec)Marshal.PtrToStructure(faceptr, typeof(FT_FaceRec));
			Console.WriteLine("Num Faces:" + sharpFace.FaceCount);
			Console.WriteLine("Num Glyphs:" + sharpFace.GlyphCount);
			Console.WriteLine("Num Char Maps:" + sharpFace.CharmapsCount);
			Console.WriteLine("Font Family:" + sharpFace.FamilyName);
			Console.WriteLine("Style Name:" + sharpFace.StyleName);
			Console.WriteLine("Generic:" + sharpFace.Tag);
			Console.WriteLine("Bbox:" + sharpFace.BBox);
			Console.WriteLine("Glyph:" + sharpFace.Glyph);
			//   IConsole.Write("Num Glyphs:", );

			//Freetype measures the font size in 1/64th of pixels for accuracy 
			//so we need to request characters in size*64
			//FT.FT_Set_Char_Size(faceptr, font_size << 6, font_size << 6, DPI, DPI);
            sharpFace.SetCharSize(font_size << 6, font_size << 6, DPI, DPI);
            //Provide a reasonably accurate estimate for expected pixel sizes
            //when we later on create the bitmaps for the font
            //FT.FT_Set_Pixel_Sizes(faceptr, (uint)font_size, (uint)font_size);
            sharpFace.SetPixelSizes((uint)font_size, (uint)font_size);

            // Once we have the face loaded and sized we generate opengl textures 
            // from the glyphs  for each printable character
            Console.WriteLine("Compiling Font Characters 0..127");
			textures = new int[128];
			extent_x = new int[128];
			offsets = new FTGlyphOffset[128];
			list_base = GL.GenLists(128);
			GL.GenTextures(128, textures);
			for (int c = 0; c < 128; c++)
			{
				CompileCharacterToTexture(sharpFace, c);

				if (c < 127)
					sChars += ",";
			}
			//Console.WriteLine("Font Compiled:" + sChars);            
		}

		internal int next_po2(int a)
		{
			int rval = 1;
			while (rval < a) rval <<= 1;
			return rval;
		}


		private void CompileCharacterToTexture(Face face, int c)
		{
			FTGlyphOffset offset = new FTGlyphOffset();

			//We first convert the number index to a character index
			//uint index = FT.FT_Get_Char_Index(faceptr, (uint)c);
            uint index = sharpFace.GetCharIndex((uint)c);
            string sError = "";
			if (index == 0) sError = "No Glyph";

            //Here we load the actual glyph for the character
            //int ret = FT.FT_Load_Glyph(faceptr, index, FT.FT_LOAD_DEFAULT);
            //if (ret != 0)
            //{
            //    Console.Write("Load_Glyph failed for character " + c.ToString());
            //}
            try
            {
                sharpFace.LoadGlyph(index, LoadFlags.Default, LoadTarget.Normal);
            }
            catch
            {
                Console.Write("Load_Glyph failed for character " + c.ToString());
            }


            //         FT_GlyphSlotRec glyphrec = (FT_GlyphSlotRec)Marshal.PtrToStructure(face.glyph, typeof(FT_GlyphSlotRec));

            //ret = FT.FT_Render_Glyph(ref glyphrec, FT_Render_Mode.FT_RENDER_MODE_NORMAL);

            //         if (ret != 0)
            //{
            //	Console.Write("Render failed for character " + c.ToString());
            //}
            GlyphSlot glyphrec = sharpFace.Glyph;

            int size = (glyphrec.Bitmap.Width * glyphrec.Bitmap.Rows);
			if (size <= 0)
			{
				//Console.Write("Blank Character: " + c.ToString());
				//space is a special `blank` character
				extent_x[c] = 0;
				if (c == 32)
				{
					GL.NewList((list_base + c), ListMode.Compile);
					GL.Translate(font_size >> 1, 0, 0);
					extent_x[c] = font_size >> 1;
					GL.EndList();
					offset.left = 0;
					offset.top = 0;
					offset.height = 0;
					offset.width = extent_x[c];
					offsets[c] = offset;
				}
				return;

			}

			byte[] bmp = new byte[size];
			Marshal.Copy(glyphrec.Bitmap.Buffer, bmp, 0, bmp.Length);

			//Next we expand the bitmap into an opengl texture 	    	
			int width = next_po2(glyphrec.Bitmap.Width);
			int height = next_po2(glyphrec.Bitmap.Rows);

			byte[] expanded = new byte[2 * width * height];
			for (int j = 0; j < height; j++)
			{
				for (int i = 0; i < width; i++)
				{
					//Luminance
					expanded[2 * (i + j * width)] = (byte)255;
					//expanded[4 * (i + j * width) + 1] = (byte)255;
					//expanded[4 * (i + j * width) + 2] = (byte)255;

					// Alpha
					expanded[2 * (i + j * width) + 1] =
						(i >= glyphrec.Bitmap.Width || j >= glyphrec.Bitmap.Rows) ?
						(byte)0 : (byte)(bmp[i + glyphrec.Bitmap.Width * j]);
				}
			}

			//Set up some texture parameters for opengl
			GL.BindTexture(TextureTarget.Texture2D, textures[c]);
			GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, 0x812F); //const Int32 GL_CLAMP_TO_EDGE = 0x812F;

            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, 0x812F); //const Int32 GL_CLAMP_TO_EDGE = 0x812F;
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, 0x2601); //const Int32 GL_LINEAR = 0x2601;
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, 0x2601); //const Int32 GL_LINEAR = 0x2601;

            //Create the texture
            GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba, width, height,
				0, PixelFormat.LuminanceAlpha, PixelType.UnsignedByte, expanded);
			expanded = null;
			bmp = null;

			//Create a display list and bind a texture to it
			GL.NewList((list_base + c), ListMode.Compile);
			GL.BindTexture(TextureTarget.Texture2D, textures[c]);

			//Account for freetype spacing rules
			GL.Translate(glyphrec.BitmapLeft, 0, 0);
			GL.PushMatrix();
			GL.Translate(0, glyphrec.BitmapTop - glyphrec.Bitmap.Rows, 0);
			float x = (float)glyphrec.Bitmap.Width / (float)width;
			float y = (float)glyphrec.Bitmap.Rows / (float)height;

			offset.left = glyphrec.BitmapLeft;
			offset.top = glyphrec.BitmapTop;
			offset.height = glyphrec.Bitmap.Rows;
			offset.width = glyphrec.Bitmap.Width;
			offset.advance = glyphrec.Advance;
			offset.lsb_delta = glyphrec.DeltaLsb;
			offset.rsb_delta = glyphrec.DeltaRsb;
			offset.linearHoriAdvance = glyphrec.LinearHorizontalAdvance;
			offset.linearVertAdvance = glyphrec.LinearVerticalAdvance;
			offsets[c] = offset;

			//Draw the quad
			GL.Begin(BeginMode.Quads);
			GL.TexCoord2(0, 0); GL.Vertex2(0, glyphrec.Bitmap.Rows);
			GL.TexCoord2(0, y); GL.Vertex2(0, 0);
			GL.TexCoord2(x, y); GL.Vertex2(glyphrec.Bitmap.Width, 0);
			GL.TexCoord2(x, 0); GL.Vertex2(glyphrec.Bitmap.Width, glyphrec.Bitmap.Rows);
			GL.End();
			GL.PopMatrix();

			//Advance for the next character			
			GL.Translate(glyphrec.Bitmap.Width, 0, 0);
			extent_x[c] = glyphrec.BitmapLeft + glyphrec.Bitmap.Width;
			GL.EndList();
			sChars += "f:" + c.ToString() + "[w:" + glyphrec.Bitmap.Width.ToString() + "][h:" + glyphrec.Bitmap.Rows.ToString() + "]" + sError;
		}

		/// <summary>
		/// Dispose of the font
		/// </summary>
		public void Dispose()
		{
			ftClearFont();
			// Dispose of these as we don't need
			if (faceptr != IntPtr.Zero)
			{
                //FT.FT_Done_Face(faceptr);
                sharpFace.Dispose();
                sharpLib.Dispose();
				faceptr = IntPtr.Zero;
                libptr = IntPtr.Zero;
			}

		}

		/// <summary>
		/// Dispose of the FreeType library
		/// </summary>
		public static void DisposeFreeType()
		{
			//FT.FT_Done_FreeType(libptr); codeyu
		}

		/// <summary>
		/// Clear all OpenGL-related structures.
		/// </summary>
		public void ftClearFont()
		{

			if (list_base > 0)
				GL.DeleteLists(list_base, 128);

			if (textures != null)
				GL.DeleteTextures(128, textures);

			textures = null;
			extent_x = null;
		}

		/// <summary>
		/// Return the horizontal extent (width),in pixels, of a given font string
		/// </summary>
		/// <param name="text"></param>
		/// <returns></returns>
		public virtual float ftExtent(ref string text)
		{
			int ret = 0;
			for (int c = 0; c < text.Length; c++)
				ret += extent_x[text[c]];
			return ret;
		}

		/// <summary>
		/// Return the Glyph offsets for the first character in "text"
		/// </summary>
		public FTGlyphOffset ftGetGlyphOffset(Char glyphchar)
		{
			return offsets[glyphchar];
		}

		/// <summary>
		/// Initialise the OpenGL state necessary fo rendering the font properly
		/// </summary>
		public void ftBeginFont()
		{
			int font = list_base;

			//Prepare openGL for rendering the font characters      
			GL.PushAttrib(AttribMask.ListBit | AttribMask.CurrentBit | AttribMask.EnableBit | AttribMask.TransformBit);
			GL.Disable(EnableCap.Lighting);
			GL.Enable(EnableCap.Texture2D);
			GL.Disable(EnableCap.DepthTest);
			GL.Enable(EnableCap.Blend);
			GL.BlendFunc(BlendingFactorSrc.Src1Alpha, BlendingFactorDest.OneMinusSrcAlpha);
			GL.ListBase(font);
		}


		#region ftWrite(string text)
		/// <summary>
		///     Custom GL "write" routine.
		/// </summary>
		/// <param name="text">
		///     The text to print.
		/// </param>
		public void ftWrite(string text)
		{
			// Scale to fit on Projection rendering screen with default coordinates                        
			GL.PushMatrix();

			switch (FT_ALIGN)
			{
				case FTFontAlign.FT_ALIGN_CENTERED:
					GL.Translate(-ftExtent(ref text) / 2, 0, 0);
					break;

				case FTFontAlign.FT_ALIGN_RIGHT:
					GL.Translate(-ftExtent(ref text), 0, 0);
					break;

				//By default it is left-aligned, so there's no need to bother translating by 0.
			}


			//Render
			byte[] textbytes = new byte[text.Length];
			for (int i = 0; i < text.Length; i++)
				textbytes[i] = (byte)text[i];
			GL.CallLists(text.Length, ListNameType.UnsignedByte, textbytes);
			textbytes = null;
			GL.PopMatrix();
		}
		#endregion

		/// <summary>
		/// Restore the OpenGL state to what it was prior
		/// to starting to draw the font
		/// </summary>
		public void ftEndFont()
		{
			//Restore openGL state
			//GL.PopMatrix();
			GL.PopAttrib();
		}
	}
}