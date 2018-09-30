﻿/*
  Box2DNet Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

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

#define DEBUG
#undef ALLOWSAFE

using System; using System.Numerics;
using Box2DNet.Common;
 

using Transform = Box2DNet.Common.Transform;

namespace Box2DNet.Collision
{
	/// <summary>
	/// Used to warm start Distance.
	/// Set count to zero on first call.
	/// </summary>
	public struct SimplexCache
	{
		/// <summary>
		/// Length or area.
		/// </summary>
		public Single Metric;
		public UInt16 Count;
		/// <summary>
		/// Vertices on shape A.
		/// </summary>
		//public Byte[/*3*/] IndexA;
		public IndexArray IndexA;
		/// <summary>
		/// Vertices on shape B.
		/// </summary>
		//public Byte[/*3*/] IndexB;
		public IndexArray IndexB;

		//public SimplexCache(byte init)
		//{
		//	Metric = 0;
		//	Count = 0;
		//	IndexA = new Byte[3];
		//	IndexB = new Byte[3];
		//}
	}

	public struct IndexArray
	{
		private Byte I0, I1, I2;

		public Byte this[int index]
		{
			get
			{
#if DEBUG
				Box2DNetDebug.Assert(index >= 0 && index < 3);
#endif
				if (index == 0) return I0;
				else if (index == 1) return I1;
				else return I2;
			}
			set
			{
#if DEBUG
				Box2DNetDebug.Assert(index >= 0 && index < 3);
#endif
				if (index == 0) I0 = value;
				else if (index == 1) I1 = value;
				else I2 = value;
			}
		}
	}

	/// <summary>
	/// Input for Distance.
	/// You have to option to use the shape radii
	/// in the computation.
	/// </summary>
	public struct DistanceInput
	{
		public Transform TransformA;
		public Transform TransformB;
		public bool UseRadii;
	}

	/// <summary>
	/// Output for Distance.
	/// </summary>
	public struct DistanceOutput
	{
		/// <summary>
		/// Closest point on shapeA.
		/// </summary>
		public Vector2 PointA;
		/// <summary>
		/// Closest point on shapeB.
		/// </summary>
		public Vector2 PointB;
		public float Distance;
		/// <summary>
		/// Number of GJK iterations used.
		/// </summary>
		public int Iterations;
	}

	// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.

	internal struct SimplexVertex
	{
		internal Vector2 wA;		// support point in shapeA
		internal Vector2 wB;		// support point in shapeB
		internal Vector2 w;		// wB - wA
		internal float a;		// barycentric coordinate for closest point
		internal int indexA;	// wA index
		internal int indexB;	// wB index
	}

	internal struct Simplex
	{
		internal SimplexVertex _v1, _v2, _v3;
		internal int _count;
		
#if ALLOWUNSAFE
		internal unsafe void ReadCache(SimplexCache* cache, Shape shapeA, Transform TransformA, Shape shapeB, Transform TransformB)
		{
			Box2DNetDebug.Assert(0 <= cache->Count && cache->Count <= 3);

			// Copy data from cache.
			_count = cache->Count;
			SimplexVertex** vertices = stackalloc SimplexVertex*[3];
			fixed (SimplexVertex* v1Ptr = &_v1, v2Ptr = &_v2, v3Ptr = &_v3)
			{
				vertices[0] = v1Ptr;
				vertices[1] = v2Ptr;
				vertices[2] = v3Ptr;
				for (int i = 0; i < _count; ++i)
				{
					SimplexVertex* v = vertices[i];
					v->indexA = cache->IndexA[i];
					v->indexB = cache->IndexB[i];
					Vector2 wALocal = shapeA.GetVertex(v->indexA);
					Vector2 wBLocal = shapeB.GetVertex(v->indexB);
					v->wA = TransformA.TransformPoint(wALocal);
					v->wB = TransformB.TransformPoint(wBLocal);
					v->w = v->wB - v->wA;
					v->a = 0.0f;
				}

				// Compute the new simplex metric, if it is substantially different than
				// old metric then flush the simplex.
				if (_count > 1)
				{
					float metric1 = cache->Metric;
					float metric2 = GetMetric();
					if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Common.Settings.FLT_EPSILON)
					{
						// Reset the simplex.
						_count = 0;
					}
				}

				// If the cache is empty or invalid ...
				if (_count == 0)
				{
					SimplexVertex* v = vertices[0];
					v->indexA = 0;
					v->indexB = 0;
					Vector2 wALocal = shapeA.GetVertex(0);
					Vector2 wBLocal = shapeB.GetVertex(0);
					v->wA = TransformA.TransformPoint(wALocal);
					v->wB = TransformB.TransformPoint(wBLocal);
					v->w = v->wB - v->wA;
					_count = 1;
				}
			}
		}

		internal unsafe void WriteCache(SimplexCache* cache)
		{
			cache->Metric = GetMetric();
			cache->Count = (UInt16)_count;
			SimplexVertex** vertices = stackalloc SimplexVertex*[3];
			fixed (SimplexVertex* v1Ptr = &_v1, v2Ptr = &_v2, v3Ptr = &_v3)
			{
				vertices[0] = v1Ptr;
				vertices[1] = v2Ptr;
				vertices[2] = v3Ptr;
				for (int i = 0; i < _count; ++i)
				{
					cache->IndexA[i] = (Byte)(vertices[i]->indexA);
					cache->IndexB[i] = (Byte)(vertices[i]->indexB);
				}
			}
		}
#else // ALLOWUNSAFE
		
		internal void ReadCache(SimplexCache cache, Shape shapeA, Transform transformA, Shape shapeB, Transform transformB)
		{
			Box2DNetDebug.Assert(0 <= cache.Count && cache.Count <= 3);

			// Copy data from cache.
			_count = cache.Count;
			SimplexVertex[] vertices = new SimplexVertex[] { _v1, _v2, _v3 };
			for (int i = 0; i < _count; ++i)
			{
				SimplexVertex v = vertices[i];
				v.indexA = cache.IndexA[i];
				v.indexB = cache.IndexB[i];
				Vector2 wALocal = shapeA.GetVertex(v.indexA);
				Vector2 wBLocal = shapeB.GetVertex(v.indexB);
				v.wA = transformA.TransformPoint(wALocal);
				v.wB = transformB.TransformPoint(wBLocal);
				v.w = v.wB - v.wA;
				v.a = 0.0f;
			}

			// Compute the new simplex metric, if it is substantially different than
			// old metric then flush the simplex.
			if (_count > 1)
			{
				float metric1 = cache.Metric;
				float metric2 = GetMetric();
				if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Common.Settings.FLT_EPSILON)
				{
					// Reset the simplex.
					_count = 0;
				}
			}

			// If the cache is empty or invalid ...
			if (_count == 0)
			{
				SimplexVertex v = vertices[0];
				v.indexA = 0;
				v.indexB = 0;
				Vector2 wALocal = shapeA.GetVertex(0);
				Vector2 wBLocal = shapeB.GetVertex(0);
				v.wA = transformA.TransformPoint(wALocal);
				v.wB = transformB.TransformPoint(wBLocal);
				v.w = v.wB - v.wA;
				_count = 1;
			}
		}

		internal void WriteCache(SimplexCache cache)
		{
			cache.Metric = GetMetric();
			cache.Count = (UInt16)_count;
			SimplexVertex[] vertices = new SimplexVertex[] { _v1, _v2, _v3 };
			for (int i = 0; i < _count; ++i)
			{
				cache.IndexA[i] = (Byte)(vertices[i].indexA);
				cache.IndexB[i] = (Byte)(vertices[i].indexB);
			}
		}
#endif // ALLOWUNSAFE

		internal Vector2 GetClosestPoint()
		{
			switch (_count)
			{
				case 0:
#if DEBUG
					Box2DNetDebug.Assert(false);
#endif
					return Vector2.Zero;
				case 1:
					return _v1.w;
				case 2:
					return _v1.a * _v1.w + _v2.a * _v2.w;
				case 3:
					return Vector2.Zero;
				default:
#if DEBUG
					Box2DNetDebug.Assert(false);
#endif
					return Vector2.Zero;
			}
		}
		
#if ALLOWUNSAFE
		internal unsafe void GetWitnessPoints(Vector2* pA, Vector2* pB)
		{
			switch (_count)
			{
				case 0:
					Box2DNetDebug.Assert(false);
					break;

				case 1:
					*pA = _v1.wA;
					*pB = _v1.wB;
					break;

				case 2:
					*pA = _v1.a * _v1.wA + _v2.a * _v2.wA;
					*pB = _v1.a * _v1.wB + _v2.a * _v2.wB;
					break;

				case 3:
					*pA = _v1.a * _v1.wA + _v2.a * _v2.wA + _v3.a * _v3.wA;
					*pB = *pA;
					break;

				default:
					Box2DNetDebug.Assert(false);
					break;
			}
		}
#else
		internal void GetWitnessPoints(ref Vector2 pA, ref Vector2 pB)
		{
			switch (_count)
			{
				case 0:
					Box2DNetDebug.Assert(false);
					break;

				case 1:
					pA = _v1.wA;
					pB = _v1.wB;
					break;

				case 2:
					pA = _v1.a * _v1.wA + _v2.a * _v2.wA;
					pB = _v1.a * _v1.wB + _v2.a * _v2.wB;
					break;

				case 3:
					pA = _v1.a * _v1.wA + _v2.a * _v2.wA + _v3.a * _v3.wA;
					pB = pA;
					break;

				default:
					Box2DNetDebug.Assert(false);
					break;
			}
		}
		
#endif // ALLOWUNSAFE

		internal float GetMetric()
		{
			switch (_count)
			{
				case 0:
#if DEBUG
					Box2DNetDebug.Assert(false);
#endif
					return 0.0f;

				case 1:
					return 0.0f;

				case 2:
					return (_v1.w - _v2.w).Length();

				case 3:
					return (_v2.w - _v1.w).Cross(_v3.w - _v1.w);

				default:
#if DEBUG
					Box2DNetDebug.Assert(false);
#endif
					return 0.0f;
			}
		}

		// Solve a line segment using barycentric coordinates.
		//
		// p = a1 * w1 + a2 * w2
		// a1 + a2 = 1
		//
		// The vector from the origin to the closest point on the line is
		// perpendicular to the line.
		// e12 = w2 - w1
		// dot(p, e) = 0
		// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
		//
		// 2-by-2 linear system
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		//
		// Define
		// d12_1 =  dot(w2, e12)
		// d12_2 = -dot(w1, e12)
		// d12 = d12_1 + d12_2
		//
		// Solution
		// a1 = d12_1 / d12
		// a2 = d12_2 / d12
		internal void Solve2()
		{
			Vector2 w1 = _v1.w;
			Vector2 w2 = _v2.w;
			Vector2 e12 = w2 - w1;

			// w1 region
			float d12_2 = -Vector2.Dot(w1, e12);
			if (d12_2 <= 0.0f)
			{
				// a2 <= 0, so we clamp it to 0
				_v1.a = 1.0f;
				_count = 1;
				return;
			}

			// w2 region
			float d12_1 = Vector2.Dot(w2, e12);
			if (d12_1 <= 0.0f)
			{
				// a1 <= 0, so we clamp it to 0
				_v2.a = 1.0f;
				_count = 1;
				_v1 = _v2;
				return;
			}

			// Must be in e12 region.
			float inv_d12 = 1.0f / (d12_1 + d12_2);
			_v1.a = d12_1 * inv_d12;
			_v2.a = d12_2 * inv_d12;
			_count = 2;
		}

		// Possible regions:
		// - points[2]
		// - edge points[0]-points[2]
		// - edge points[1]-points[2]
		// - inside the triangle
		internal void Solve3()
		{
			Vector2 w1 = _v1.w;
			Vector2 w2 = _v2.w;
			Vector2 w3 = _v3.w;

			// Edge12
			// [1      1     ][a1] = [1]
			// [w1.e12 w2.e12][a2] = [0]
			// a3 = 0
			Vector2 e12 = w2 - w1;
			float w1e12 = Vector2.Dot(w1, e12);
			float w2e12 = Vector2.Dot(w2, e12);
			float d12_1 = w2e12;
			float d12_2 = -w1e12;

			// Edge13
			// [1      1     ][a1] = [1]
			// [w1.e13 w3.e13][a3] = [0]
			// a2 = 0
			Vector2 e13 = w3 - w1;
			float w1e13 = Vector2.Dot(w1, e13);
			float w3e13 = Vector2.Dot(w3, e13);
			float d13_1 = w3e13;
			float d13_2 = -w1e13;

			// Edge23
			// [1      1     ][a2] = [1]
			// [w2.e23 w3.e23][a3] = [0]
			// a1 = 0
			Vector2 e23 = w3 - w2;
			float w2e23 = Vector2.Dot(w2, e23);
			float w3e23 = Vector2.Dot(w3, e23);
			float d23_1 = w3e23;
			float d23_2 = -w2e23;

			// Triangle123
			float n123 = e12.Cross(e13); 

			float d123_1 = n123 * w2.Cross(w3);
			float d123_2 = n123 * w3.Cross(w1);
			float d123_3 = n123 * w1.Cross(w2);

			// w1 region
			if (d12_2 <= 0.0f && d13_2 <= 0.0f)
			{
				_v1.a = 1.0f;
				_count = 1;
				return;
			}

			// e12
			if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
			{
				float inv_d12 = 1.0f / (d12_1 + d12_2);
				_v1.a = d12_1 * inv_d12;
				_v2.a = d12_1 * inv_d12;
				_count = 2;
				return;
			}

			// e13
			if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
			{
				float inv_d13 = 1.0f / (d13_1 + d13_2);
				_v1.a = d13_1 * inv_d13;
				_v3.a = d13_2 * inv_d13;
				_count = 2;
				_v2 = _v3;
				return;
			}

			// w2 region
			if (d12_1 <= 0.0f && d23_2 <= 0.0f)
			{
				_v2.a = 1.0f;
				_count = 1;
				_v1 = _v2;
				return;
			}

			// w3 region
			if (d13_1 <= 0.0f && d23_1 <= 0.0f)
			{
				_v3.a = 1.0f;
				_count = 1;
				_v1 = _v3;
				return;
			}

			// e23
			if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
			{
				float inv_d23 = 1.0f / (d23_1 + d23_2);
				_v2.a = d23_1 * inv_d23;
				_v3.a = d23_2 * inv_d23;
				_count = 2;
				_v1 = _v3;
				return;
			}

			// Must be in triangle123
			float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
			_v1.a = d123_1 * inv_d123;
			_v2.a = d123_2 * inv_d123;
			_v3.a = d123_3 * inv_d123;
			_count = 3;
		}
	}

	public partial class Collision
	{
		/// <summary>
		/// Compute the closest points between two shapes. Supports any combination of:
		/// CircleShape, PolygonShape, EdgeShape. The simplex cache is input/output.
		/// On the first call set SimplexCache.Count to zero.
		/// </summary>		
		public 
#if ALLOWUNSAFE
		unsafe 
#endif // ALLOWUNSAFE
		static void Distance(out DistanceOutput output, ref SimplexCache cache, ref DistanceInput input, Shape shapeA, Shape shapeB)
		{
			output = new DistanceOutput();

			Transform transformA = input.TransformA;
			Transform transformB = input.TransformB;

			// Initialize the simplex.
			Simplex simplex = new Simplex();
#if ALLOWUNSAFE
			fixed (SimplexCache* sPtr = &cache)
			{
				simplex.ReadCache(sPtr, shapeA, transformA, shapeB, transformB);
			}
#else
			simplex.ReadCache(cache, shapeA, transformA, shapeB, transformB);
#endif

			// Get simplex vertices as an array.
#if ALLOWUNSAFE
			SimplexVertex* vertices = &simplex._v1;
#else
			SimplexVertex[] vertices = new SimplexVertex[] { simplex._v1, simplex._v2, simplex._v3 };
#endif 

			// These store the vertices of the last simplex so that we
			// can check for duplicates and prevent cycling.
#if ALLOWUNSAFE
			int* lastA = stackalloc int[4], lastB = stackalloc int[4];
#else
			int[] lastA = new int[4];
			int[] lastB = new int[4];
#endif // ALLOWUNSAFE
			int lastCount;

			// Main iteration loop.
			int iter = 0;
			const int k_maxIterationCount = 20;
			while (iter < k_maxIterationCount)
			{
				// Copy simplex so we can identify duplicates.
				lastCount = simplex._count;
				int i;
				for (i = 0; i < lastCount; ++i)
				{
					lastA[i] = vertices[i].indexA;
					lastB[i] = vertices[i].indexB;
				}

				switch (simplex._count)
				{
					case 1:
						break;

					case 2:
						simplex.Solve2();
						break;

					case 3:
						simplex.Solve3();
						break;

					default:
#if DEBUG
						Box2DNetDebug.Assert(false);
#endif
						break;
				}

				// If we have 3 points, then the origin is in the corresponding triangle.
				if (simplex._count == 3)
				{
					break;
				}

				// Compute closest point.
				Vector2 p = simplex.GetClosestPoint();
				float distanceSqr = p.LengthSquared();

				// Ensure the search direction is numerically fit.
				if (distanceSqr < Common.Settings.FLT_EPSILON_SQUARED)
				{
					// The origin is probably contained by a line segment
					// or triangle. Thus the shapes are overlapped.

					// We can't return zero here even though there may be overlap.
					// In case the simplex is a point, segment, or triangle it is difficult
					// to determine if the origin is contained in the CSO or very close to it.
					break;
				}

				// Compute a tentative new simplex vertex using support points.
#if ALLOWUNSAFE
				SimplexVertex* vertex = vertices + simplex._count;
				vertex->indexA = shapeA.GetSupport(transformA.InverseTransformDirection(p));
				vertex->wA = transformA.TransformPoint(shapeA.GetVertex(vertex->indexA));
				//Vec2 wBLocal;
				vertex->indexB = shapeB.GetSupport(transformB.InverseTransformDirection(-p));
				vertex->wB = transformB.TransformPoint(shapeB.GetVertex(vertex->indexB));
				vertex->w = vertex->wB - vertex->wA;
#else
				SimplexVertex vertex = vertices[simplex._count - 1];
				vertex.indexA = shapeA.GetSupport(transformA.InverseTransformDirection(p));
				vertex.wA = transformA.TransformPoint(shapeA.GetVertex(vertex.indexA));
				//Vec2 wBLocal;
				vertex.indexB = shapeB.GetSupport(transformB.InverseTransformDirection(-p));
				vertex.wB = transformB.TransformPoint(shapeB.GetVertex(vertex.indexB));
				vertex.w = vertex.wB - vertex.wA;	
#endif // ALLOWUNSAFE

				// Iteration count is equated to the number of support point calls.
				++iter;

				// Check for convergence.
#if ALLOWUNSAFE
				float lowerBound = Vector2.Dot(p, vertex->w);
#else
				float lowerBound = Vector2.Dot(p, vertex.w);
#endif
				float upperBound = distanceSqr;
				const float k_relativeTolSqr = 0.01f * 0.01f;	// 1:100
				if (upperBound - lowerBound <= k_relativeTolSqr * upperBound)
				{
					// Converged!
					break;
				}

				// Check for duplicate support points.
				bool duplicate = false;
				for (i = 0; i < lastCount; ++i)
				{
#if ALLOWUNSAFE
					if (vertex->indexA == lastA[i] && vertex->indexB == lastB[i])
#else
					if (vertex.indexA == lastA[i] && vertex.indexB == lastB[i])
#endif
					{
						duplicate = true;
						break;
					}
				}

				// If we found a duplicate support point we must exit to avoid cycling.
				if (duplicate)
				{
					break;
				}

				// New vertex is ok and needed.
				++simplex._count;
			}

			
#if ALLOWUNSAFE
			fixed (DistanceOutput* doPtr = &output)
			{
				// Prepare output.
				simplex.GetWitnessPoints(&doPtr->PointA, &doPtr->PointB);
				doPtr->Distance = Vector2.Distance(doPtr->PointA, doPtr->PointB);
				doPtr->Iterations = iter;
			}

			fixed (SimplexCache* sPtr = &cache)
			{
				// Cache the simplex.
				simplex.WriteCache(sPtr);
			}
#else
			// Prepare output.
			simplex.GetWitnessPoints(ref output.PointA, ref output.PointB);
			output.Distance = Box2DNet.Common.Math.Distance(output.PointA, output.PointB);
			output.Iterations = iter;
			
			// Cache the simplex.
			simplex.WriteCache(cache);
#endif

			// Apply radii if requested.
			if (input.UseRadii)
			{
				float rA = shapeA._radius;
				float rB = shapeB._radius;

				if (output.Distance > rA + rB && output.Distance > Common.Settings.FLT_EPSILON)
				{
					// Shapes are still no overlapped.
					// Move the witness points to the outer surface.
					output.Distance -= rA + rB;
					Vector2 normal = output.PointB - output.PointA;
					normal.Normalize();
					output.PointA += rA * normal;
					output.PointB -= rB * normal;
				}
				else
				{
					// Shapes are overlapped when radii are considered.
					// Move the witness points to the middle.
					Vector2 p = 0.5f * (output.PointA + output.PointB);
					output.PointA = p;
					output.PointB = p;
					output.Distance = 0.0f;
				}
			}
		}
	}
}