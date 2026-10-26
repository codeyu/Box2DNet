using System;
using System.Numerics;

namespace Box2DNet.Common
{
    public static class Utils
    {
        public static Vector3 Xyz(this Quaternion q)
        {
            return new Vector3(q.X,q.Y,q.Z);
        }
        
        public static  void Normalize(this Vector2 v2)
        {
            var scale = 1.0f / v2.Length();
            v2.X *= scale;
            v2.Y *= scale;
        }
        
        public static  Vector2 Normalized(this Vector2 v2)
        {
            
            var scale = 1.0f / v2.Length();
            v2.X *= scale;
            v2.Y *= scale;
            return v2;
        }
        /// <summary>
        /// Gets or sets the value at the index of the Vector.
        /// </summary>
        public static float GetByIndex(this Vector2 v2, int index) 
        {
            
                if (index == 0)
                {
                    return v2.X;
                }
                else if (index == 1)
                {
                    return v2.Y;
                }
                throw new IndexOutOfRangeException("You tried to access this vector at index: " + index);
            
        }

        public static void SetByIndex(this Vector2 v2, int index, float value)
        {
            switch (index)
            {
                case 0:
                    v2.X = value;
                    break;
                case 1:
                    v2.Y = value;
                    break;
                default:
                    throw new IndexOutOfRangeException("You tried to set this vector at index: " + index);
            }
        }
        
    }
}