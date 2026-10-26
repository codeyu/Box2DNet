// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2DNet;

public static partial class B2
{
	/// The maximum number of vertices on a convex polygon. Changing this affects performance even if
	/// you don't use more vertices.
	public const int MaxPolygonVertices = 8;

	public static readonly SimplexCache emptySimplexCache = default;
}

/// Low level ray cast input data
public struct RayCastInput
{
	/// Start point of the ray cast
	public Vec2 origin;

	/// Translation of the ray cast
	public Vec2 translation;

	/// The maximum fraction of the translation to consider, typically 1
	public float maxFraction;
}

/// A distance proxy is used by the GJK algorithm. It encapsulates any shape. You can provide
/// between 1 and B2_MAX_POLYGON_VERTICES points and a radius.
public struct ShapeProxy
{
	/// The point cloud
	public FixedArray8<Vec2> points;

	/// The number of points. Must be greater than 0.
	public int count;

	/// The external radius of the point cloud. May be zero.
	public float radius;
}

/// Low level shape cast input in generic form. This allows casting an arbitrary point cloud wrapped
/// with a radius. For example, a circle is a single point with a non-zero radius. A capsule is two
/// points with a non-zero radius. A box is four points with a zero radius.
public struct ShapeCastInput
{
	/// A generic shape
	public ShapeProxy proxy;

	/// The translation of the shape cast
	public Vec2 translation;

	/// The maximum fraction of the translation to consider, typically 1
	public float maxFraction;

	/// Allow shape cast to encroach when initially touching. This only works if the radius is
	/// greater than zero.
	public bool canEncroach;
}

/// Low level ray cast or shape cast output data. The hit point is in the local or relative frame of
/// the input. Returns a zero fraction and normal in the case of initial overlap.
/// In single precision this is also the world cast output (b2WorldCastOutput).
public struct CastOutput
{
	/// The surface normal at the hit point
	public Vec2 normal;

	/// The surface hit point
	public Vec2 point;

	/// The fraction of the input translation at collision
	public float fraction;

	/// The number of iterations used
	public int iterations;

	/// Did the cast hit?
	public bool hit;
}

/// This holds the mass data computed for a shape.
public struct MassData
{
	/// The mass of the shape, usually in kilograms.
	public float mass;

	/// The position of the shape's centroid relative to the shape's origin.
	public Vec2 center;

	/// The rotational inertia of the shape about the shape center.
	public float rotationalInertia;
}

/// A solid circle
public struct Circle
{
	/// The local center
	public Vec2 center;

	/// The radius
	public float radius;

	public Circle( Vec2 center, float radius )
	{
		this.center = center;
		this.radius = radius;
	}
}

/// A solid capsule can be viewed as two semicircles connected by a rectangle.
public struct Capsule
{
	/// Local center of the first semicircle
	public Vec2 center1;

	/// Local center of the second semicircle
	public Vec2 center2;

	/// The radius of the semicircles
	public float radius;

	public Capsule( Vec2 center1, Vec2 center2, float radius )
	{
		this.center1 = center1;
		this.center2 = center2;
		this.radius = radius;
	}
}

/// A solid convex polygon. It is assumed that the interior of the polygon is to the left of each
/// edge. Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES. In most cases
/// you should not need many vertices for a convex polygon.
/// @warning DO NOT fill this out manually, instead use a helper function like b2MakePolygon or
/// b2MakeBox.
public struct Polygon
{
	/// The polygon vertices
	public FixedArray8<Vec2> vertices;

	/// The outward normal vectors of the polygon sides
	public FixedArray8<Vec2> normals;

	/// The centroid of the polygon
	public Vec2 centroid;

	/// The external radius for rounded polygons
	public float radius;

	/// The number of polygon vertices
	public int count;
}

/// A line segment with two-sided collision.
public struct Segment
{
	/// The first point
	public Vec2 point1;

	/// The second point
	public Vec2 point2;

	public Segment( Vec2 point1, Vec2 point2 )
	{
		this.point1 = point1;
		this.point2 = point2;
	}
}

/// A line segment with one-sided collision. Only collides on the right side. Several of these are
/// generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
public struct ChainSegment
{
	/// The tail ghost vertex
	public Vec2 ghost1;

	/// The line segment
	public Segment segment;

	/// The head ghost vertex
	public Vec2 ghost2;

	/// The owning chain shape index (internal usage only)
	public int chainId;
}

/// A convex hull. Used to create convex polygons.
/// @warning Do not modify these values directly, instead use b2ComputeHull()
public struct Hull
{
	/// The final points of the hull
	public FixedArray8<Vec2> points;

	/// The number of points
	public int count;
}

/// Result of computing the distance between two line segments
public struct SegmentDistanceResult
{
	/// The closest point on the first segment
	public Vec2 closest1;

	/// The closest point on the second segment
	public Vec2 closest2;

	/// The barycentric coordinate on the first segment
	public float fraction1;

	/// The barycentric coordinate on the second segment
	public float fraction2;

	/// The squared distance between the closest points
	public float distanceSquared;
}

/// Used to warm start the GJK simplex. If you call b2ComputeShapeDistance multiple times with
/// nearby transforms this might improve performance. The distance cache must be initialized to zero
/// on the first call.
public struct SimplexCache
{
	/// The number of stored simplex points
	public ushort count;

	/// The cached simplex indices on shape A
	public FixedArray3<byte> indexA;

	/// The cached simplex indices on shape B
	public FixedArray3<byte> indexB;
}

/// Input for b2ComputeShapeDistance
public struct DistanceInput
{
	/// The proxy for shape A
	public ShapeProxy proxyA;

	/// The proxy for shape B
	public ShapeProxy proxyB;

	/// Transform of shape B in shape A's frame, the relative pose B in A (b2InvMulTransforms(
	/// worldA, worldB )). The query is origin independent and runs in frame A.
	public Transform transform;

	/// Should the proxy radius be considered?
	public bool useRadii;
}

/// Output for b2ComputeShapeDistance
public struct DistanceOutput
{
	public Vec2 pointA; ///< Closest point on shapeA, in shape A's frame
	public Vec2 pointB; ///< Closest point on shapeB, in shape A's frame
	public Vec2 normal; ///< A to B normal in shape A's frame. Invalid if distance is zero.
	public float distance; ///< The final distance, zero if overlapped
	public int iterations; ///< Number of GJK iterations used
	public int simplexCount; ///< The number of simplexes stored in the simplex array
}

/// Simplex vertex for debugging the GJK algorithm
public struct SimplexVertex
{
	public Vec2 wA; ///< support point in proxyA
	public Vec2 wB; ///< support point in proxyB
	public Vec2 w; ///< wB - wA
	public float a; ///< barycentric coordinate for closest point
	public int indexA; ///< wA index
	public int indexB; ///< wB index
}

/// Simplex from the GJK algorithm
public struct Simplex
{
	public SimplexVertex v1, v2, v3; ///< vertices
	public int count; ///< number of valid vertices
}

/// Input parameters for b2CastShape
public struct ShapeCastPairInput
{
	public ShapeProxy proxyA; ///< The proxy for shape A
	public ShapeProxy proxyB; ///< The proxy for shape B
	public Transform transform; ///< Transform of shape B in shape A's frame, the relative pose B in A
	public Vec2 translationB; ///< The translation of shape B, in A's frame
	public float maxFraction; ///< The fraction of the translation to consider, typically 1
	public bool canEncroach; ///< Allows shapes with a radius to move slightly closer if already touching
}

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect
/// to the body origin, which may not coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
public struct Sweep
{
	public Vec2 localCenter; ///< Local center of mass position
	public Vec2 c1; ///< Starting center of mass world position
	public Vec2 c2; ///< Ending center of mass world position
	public Rot q1; ///< Starting world rotation
	public Rot q2; ///< Ending world rotation
}

/// Time of impact input
public struct TOIInput
{
	public ShapeProxy proxyA; ///< The proxy for shape A
	public ShapeProxy proxyB; ///< The proxy for shape B
	public Sweep sweepA; ///< The movement of shape A
	public Sweep sweepB; ///< The movement of shape B
	public float maxFraction; ///< Defines the sweep interval [0, maxFraction]
}

/// Describes the TOI output
public enum TOIState
{
	toiStateUnknown,
	toiStateFailed,
	toiStateOverlapped,
	toiStateHit,
	toiStateSeparated
}

/// Time of impact output
public struct TOIOutput
{
	/// The type of result
	public TOIState state;

	/// The hit point
	public Vec2 point;

	/// The hit normal
	public Vec2 normal;

	/// The sweep time of the collision
	public float fraction;
}

/// A manifold point is a contact point belonging to a contact manifold. It holds details related to
/// the geometry and dynamics of the contact points. Box2D uses speculative collision so some
/// contact points may be separated. You may use the totalNormalImpulse to determine if there was an
/// interaction during the time step.
public struct ManifoldPoint
{
	/// Location of the contact point relative to bodyA's center of mass in world space. This can be
	/// converted to a world point using:
	/// b2Pos worldPointA = b2OffsetPos(b2Body_GetWorldCenter(myBodyIdA), anchorA);
	public Vec2 anchorA;

	/// Similar for body B.
	public Vec2 anchorB;

	/// The separation of the contact point, negative if penetrating
	public float separation;

	/// The impulse along the manifold normal vector.
	public float normalImpulse;

	/// The friction impulse
	public float tangentImpulse;

	/// The total normal impulse applied across sub-stepping and restitution. This is important to
	/// identify speculative contact points that had an interaction in the time step. This includes
	/// the warm starting impulse, the sub-step delta impulse, and the restitution impulse.
	public float totalNormalImpulse;

	/// Relative normal velocity pre-solve. Negative when approaching. This is only computed if hit
	/// events are enabled.
	public float normalVelocity;

	/// Cached separation used for contact recycling
	public float baseSeparation;

	/// Uniquely identifies a contact point between two shapes
	public ushort id;

	/// Did this contact point exist the previous step?
	public bool persisted;
}

/// A contact manifold describes the contact points between colliding shapes.
/// @note Box2D uses speculative collision so some contact points may be separated.
public struct Manifold
{
	/// The unit normal vector in world space, points from shape A to shape B.
	public Vec2 normal;

	/// Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s
	public float rollingImpulse;

	/// The manifold points, up to two are possible in 2D.
	public FixedArray2<ManifoldPoint> points;

	/// The number of contact points, will be 0, 1, or 2.
	public int pointCount;
}

/// Contact manifold point in local coordinates (frame A).
public struct LocalManifoldPoint
{
	/// Contact point in frame A
	public Vec2 point;

	/// The separation of the contact point, negative if penetrating. May be positive or negative.
	public float separation;

	/// Uniquely identifies a contact point between two shapes
	public ushort id;
}

/// Contact manifold in local coordinates (frame A).
public struct LocalManifold
{
	/// The unit normal vector in frame A, points from shape A to shape B
	public Vec2 normal;

	/// The manifold points, up to two are possible in 2D
	public FixedArray2<LocalManifoldPoint> points;

	/// The number of contact points, will be 0, 1, or 2
	public int pointCount;
}

/// These are performance results returned by dynamic tree queries.
public struct TreeStats
{
	/// Number of internal nodes visited during the query
	public int nodeVisits;

	/// Number of leaf nodes visited during the query
	public int leafVisits;
}

/// Input for casting an AABB through a dynamic tree
public struct BoxCastInput
{
	/// The AABB to cast, in the tree's frame.
	public AABB box;

	/// The sweep translation.
	public Vec2 translation;

	/// The maximum fraction of the translation to consider, typically 1.
	public float maxFraction;
}

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
public delegate bool TreeQueryCallbackFcn( int proxyId, ulong userData, object? context );

/// This function receives clipped ray cast input for a proxy. The function returns the new ray
/// fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
public delegate float TreeRayCastCallbackFcn( in RayCastInput input, int proxyId, ulong userData, object? context );

/// This function receives clipped AABB cast input for a proxy. The function returns the new cast
/// fraction.
public delegate float TreeBoxCastCallbackFcn( in BoxCastInput input, int proxyId, ulong userData, object? context );

/// Allocation free version of TreeQueryCallbackFcn used with generic tree queries.
public interface ITreeQueryCallback
{
	bool Invoke( int proxyId, ulong userData );
}

/// Allocation free version of TreeRayCastCallbackFcn used with generic tree queries.
public interface ITreeRayCastCallback
{
	float Invoke( in RayCastInput input, int proxyId, ulong userData );
}

/// Allocation free version of TreeBoxCastCallbackFcn used with generic tree queries.
public interface ITreeBoxCastCallback
{
	float Invoke( in BoxCastInput input, int proxyId, ulong userData );
}

/// These are the collision planes returned from b2World_CollideMover. The plane and point are
/// relative to the query origin, matching the mover capsule.
public struct PlaneResult
{
	/// The collision plane between the mover and a convex shape
	public Plane plane;

	/// The collision point on the shape.
	public Vec2 point;

	/// Did the collision register a hit? If not this plane should be ignored.
	public bool hit;
}

/// These are collision planes that can be fed to b2SolvePlanes. Normally this is assembled by the
/// user from plane results in b2PlaneResult.
public struct CollisionPlane
{
	/// The collision plane between the mover and some shape.
	public Plane plane;

	/// Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can make the
	/// plane collision soft. Usually in meters.
	public float pushLimit;

	/// The push on the mover determined by b2SolvePlanes. Usually in meters.
	public float push;

	/// Indicates if b2ClipVector should clip against this plane. Should be false for soft
	/// collision.
	public bool clipVelocity;
}

/// Result returned by b2SolvePlanes.
public struct PlaneSolverResult
{
	/// The final relative translation.
	public Vec2 delta;

	/// The number of iterations used by the plane solver. For diagnostics.
	public int iterationCount;
}
