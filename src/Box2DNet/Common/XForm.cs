/*
  Box2DNet Copyright (c) 2018 codeyu https://github.com/codeyu/Box2DNet
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

using System; using System.Numerics;
using System.Collections.Generic;
using System.Text;
 

namespace Box2DNet.Common
{
	/// <summary>
	/// A transform contains translation and rotation.
	/// It is used to represent the position and orientation of rigid frames.
	/// </summary>
	public struct XForm
	{
		public Vector2 	position;
		public Mat22 	R;

		/// <summary>
		/// Initialize using a position vector and a rotation matrix.
		/// </summary>
		/// <param name="position"></param>
		/// <param name="R"></param>
		public XForm(Vector2 p, Mat22 rotation)
		{
			position = p;
			R = rotation;
		}

		/// <summary>
		/// Set this to the identity transform.
		/// </summary>
		public void SetIdentity()
		{
			position = Vector2.Zero;
			R = Mat22.Identity;
		}

		/// Set this based on the position and angle.
		public void Set(Vector2 p, float angle)
		{
			position = p;
			R = new Mat22(angle);
		}

		/// Calculate the angle that the rotation matrix represents.
		public float GetAngle()
		{
			return Math.Atan2(R.Col1.Y, R.Col1.X);
		}
		
		public Vector2 TransformDirection(Vector2 vector) 
		{
			return Math.Mul(R, vector);
		}
		
		public Vector2 InverseTransformDirection(Vector2 vector)
		{
			return Math.MulT(R, vector);
		}
		
		public Vector2 TransformPoint(Vector2 vector)
		{	
			return position + Math.Mul(R, vector);
		}
		
		public Vector2 InverseTransformPoint(Vector2 vector)
		{
			return Math.MulT(R, vector - position);
		}

		public static XForm Identity { get { return new XForm(Vector2.Zero, Mat22.Identity); } }
	}
}