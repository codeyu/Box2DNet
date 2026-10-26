// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Box2DNet;

public static partial class B2
{
	/// Default filter category bits.
	public const ulong DefaultCategoryBits = 1;

	/// Default filter mask bits.
	public const ulong DefaultMaskBits = ulong.MaxValue;
}

#pragma warning disable CS1587

/// Default filter category bits.
/// @ingroup shape
/// Default filter mask bits.
/// @ingroup shape


/// Task interface This is the prototype for a Box2D task. Your task system is expected to run this
/// callback on a worker thread, exactly once per enqueue, passing back the same taskContext pointer
/// supplied to EnqueueTaskCallbackFcn.
/// @ingroup world
public delegate void TaskCallbackFcn( object? taskContext );

/// These functions can be provided to Box2D to invoke a task system. Returns a pointer to the
/// user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
/// serially within the callback and there is no need to call FinishTaskCallbackFcn.
/// @ingroup world
public delegate object? EnqueueTaskCallbackFcn( TaskCallbackFcn task, object? taskContext, object? userContext );

/// Finishes a user task object that wraps a Box2D task.
/// @ingroup world
public delegate void FinishTaskCallbackFcn( object? userTask, object? userContext );

/// Optional friction mixing callback. This intentionally provides no context objects because this
/// is called from a worker thread.
/// @warning This function should not attempt to modify Box2D state or user application state.
/// @ingroup world
public delegate float FrictionCallbackFcn( float frictionA, ulong userMaterialIdA, float frictionB, ulong userMaterialIdB );

/// Optional restitution mixing callback. This intentionally provides no context objects because
/// this is called from a worker thread.
/// @warning This function should not attempt to modify Box2D state or user application state.
/// @ingroup world
public delegate float RestitutionCallbackFcn( float restitutionA, ulong userMaterialIdA, float restitutionB, ulong userMaterialIdB );

/// Result from World_CastRayClosest If there is initial overlap the fraction and normal will be
/// zero while the point is an arbitrary point in the overlap region.
/// @ingroup world
public struct RayResult
{
	/// The shape hit.
	public ShapeId shapeId;

	/// The point of contact, on the surface of the hit shape.
	public Pos point;

	/// The unit normal vector at the point of contact.
	public Vec2 normal;

	/// The fraction of the input ray.
	public float fraction;

	/// The number of BVH nodes visited. Diagnostic.
	public int nodeVisits;

	/// The number of BVH leaves visited. Diagnostic.
	public int leafVisits;

	/// Did the ray hit? If false, all other data is invalid.
	public bool hit;
}

/// Optional world capacities that can be used to avoid run-time allocations.
/// @see World_GetMaxCapacity
/// @ingroup world
public struct Capacity
{
	/// Number of expected static shapes.
	public int staticShapeCount;

	/// Number of expected dynamic and kinematic shapes.
	public int dynamicShapeCount;

	/// Number of expected static bodies.
	public int staticBodyCount;

	/// Number of expected dynamic and kinematic bodies.
	public int dynamicBodyCount;

	/// Number of expected contacts.
	public int contactCount;

	/// Reserve arena allocator space, used for temporary allocations.
	public int arenaByteCount;
}

/// World definition used to create a simulation world. Must be initialized using
/// DefaultWorldDef().
/// @ingroup world
public struct WorldDef
{
	/// Gravity vector. Box2D has no up-vector defined.
	public Vec2 gravity;

	/// Restitution speed threshold, usually in m/s. Collisions above this speed have restitution
	/// applied (will bounce).
	public float restitutionThreshold;

	/// Number of iterations of the restitution solver. More iterations can lead to less box
	/// spinning.
	/// @see MaxRestitutionIterations
	public int restitutionIterations;

	/// Enable full contact propagation in the restitution solver. Expensive. (Experimental)
	public bool enableRestitutionPropagation;

	/// Threshold speed for hit events. Usually meters per second.
	public float hitEventThreshold;

	/// Contact stiffness. Cycles per second. Increasing this increases the speed of overlap
	/// recovery, but can introduce jitter.
	public float contactHertz;

	/// Contact bounciness. Non-dimensional. You can speed up overlap recovery by decreasing this
	/// with the trade-off that overlap resolution becomes more energetic.
	public float contactDampingRatio;

	/// This parameter controls how fast overlap is resolved and usually has units of meters per
	/// second. This only puts a cap on the resolution speed. The resolution speed is increased by
	/// increasing the hertz and/or decreasing the damping ratio.
	public float contactSpeed;

	/// Maximum linear speed. Usually meters per second.
	public float maximumLinearSpeed;

	/// Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).
	public FrictionCallbackFcn? frictionCallback;

	/// Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).
	public RestitutionCallbackFcn? restitutionCallback;

	/// Can bodies go to sleep to improve performance
	public bool enableSleep;

	/// Enable continuous collision
	public bool enableContinuous;

	/// Number of workers for multithreading. Box2D performs best when using performance cores and
	/// accessing a single L3 cache (uniform memory). Efficiency cores and SMT provide little
	/// benefit and may even harm performance. This is clamped to the range [1, MaxWorkers]. If
	/// task callbacks are provided then Box2D will use the user provided task system. If no task
	/// callbacks are installed then Box2D will run single-threaded with a worker count of 1, or use
	/// the internal scheduler with a worker count greater than 1.
	public int workerCount;

	/// Function to spawn tasks
	public EnqueueTaskCallbackFcn? enqueueTask;

	/// Function to finish a task
	public FinishTaskCallbackFcn? finishTask;

	/// User context that is provided to enqueueTask and finishTask
	public object? userTaskContext;

	/// User data
	public object? userData;

	/// Optional initial capacities
	public Capacity capacity;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your world definition
/// @ingroup world

/// The body simulation type. Each body is one of these three types. The type determines how the
/// body behaves in the simulation.
/// @ingroup body
public enum BodyType
{
	/// zero mass, zero velocity, may be manually moved
	staticBody = 0,

	/// zero mass, velocity set by user, moved by solver
	kinematicBody = 1,

	/// positive mass, velocity determined by forces, moved by solver
	dynamicBody = 2,

	/// number of body types
	bodyTypeCount,
}

/// Motion locks to restrict the body movement
/// @ingroup body
public struct MotionLocks
{
	/// Prevent translation along the x-axis
	public bool linearX;

	/// Prevent translation along the y-axis
	public bool linearY;

	/// Prevent rotation around the z-axis
	public bool angularZ;
}

/// A body definition holds all the data needed to construct a rigid body. You can safely re-use
/// body definitions. Shapes are added to a body after construction. Body definitions are temporary
/// objects used to bundle creation parameters. Must be initialized using DefaultBodyDef().
/// @ingroup body
public struct BodyDef
{
	/// The body type: static, kinematic, or dynamic.
	public BodyType type;

	/// The initial world position of the body. Bodies should be created with the desired position.
	/// @note Creating bodies at the origin and then moving them nearly doubles the cost of body
	/// creation, especially if the body is moved after shapes have been added.
	public Pos position;

	/// The initial world rotation of the body. Use MakeRot() if you have an angle.
	public Rot rotation;

	/// The initial linear velocity of the body's origin. Usually in meters per second.
	public Vec2 linearVelocity;

	/// The initial angular velocity of the body. Radians per second.
	public float angularVelocity;

	/// Linear damping is used to reduce the linear velocity. The damping parameter can be larger
	/// than 1 but the damping effect becomes sensitive to the time step when the damping parameter
	/// is large. Generally linear damping is undesirable because it makes objects move slowly as if
	/// they are floating.
	public float linearDamping;

	/// Angular damping is used to reduce the angular velocity. The damping parameter can be larger
	/// than 1.0f but the damping effect becomes sensitive to the time step when the damping
	/// parameter is large. Angular damping can be used to slow down rotating bodies.
	public float angularDamping;

	/// Scale the gravity applied to this body. Non-dimensional.
	public float gravityScale;

	/// Sleep speed threshold, default is 0.05 meters per second
	public float sleepThreshold;

	/// Continuous collision safety factor. The solver only uses continuous collision if there is a
	/// risk of tunneling. If the body is moving fast enough to risk tunneling then it is considered
	/// a "fast body". This improves performance and prevents movement hitches. If a body moving N
	/// meters risks tunneling, then the body will be considered fast if it moves more than a
	/// safetyFactor times N meters over one full time step. Non-dimensional. Recommended range
	/// [0.01, 0.5]. Default is 0.5 for high performance with low tunneling risk.
	public float safetyFactor;

	/// Optional body name for debugging. Up to NameLength characters
	public string? name;

	/// Use this to store application specific body data.
	public object? userData;

	/// Motion locks to restrict linear and angular movement. Caution: may lead to softer
	/// constraints along the locked direction
	public MotionLocks motionLocks;

	/// Set this flag to false if this body should never fall asleep.
	public bool enableSleep;

	/// Is this body initially awake or sleeping?
	public bool isAwake;

	/// Treat this dynamic body as a high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	/// @warning Bullets should be used sparingly. They are not a solution for general
	/// dynamic-versus-dynamic continuous collision. They do not guarantee accurate collision if
	/// both bodies are fast moving because the bullet does a continuous check after all non-bullet
	/// bodies have moved. You could get unlucky and have the bullet body end a time step very close
	/// to a non-bullet body and the non-bullet body then moves over the bullet body. In continuous
	/// collision, initial overlap is ignored to avoid freezing bodies in place. I do not recommend
	/// using them for game projectiles if precise collision timing is needed. Instead consider
	/// using a ray or shape cast. You can use a marching ray or shape cast for a projectile that
	/// moves over time. If you want a fast moving projectile to collide with a fast moving target,
	/// you need to consider the relative movement in your ray or shape cast. This is out of the
	/// scope of Box2D. So what are good use cases for bullets? Pinball games or games with dynamic
	/// containers that hold other objects. It should be a use case where it doesn't break the game
	/// if there is a collision missed, but having them captured improves the quality of the game.
	public bool isBullet;

	/// Used to disable a body. A disabled body does not move or collide.
	public bool isEnabled;

	/// This allows this body to bypass rotational speed limits. Should only be used for circular
	/// objects, like wheels.
	public bool allowFastRotation;

	/// Enable contact recycling. True by default. Leaving this enabled improves performance but may
	/// lead to ghost collisions that should be avoided on characters.
	public bool enableContactRecycling;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your body definition
/// @ingroup body

/// This is used to filter collision on shapes. It affects shape-vs-shape collision and
/// shape-versus-query collision (such as World_CastRay).
/// @ingroup shape
public struct Filter
{
	/// The collision category bits. Normally you would just set one bit. The category bits should
	/// represent your application object types. For example:
	/// @code{.cpp}
	/// enum MyCategories
	/// {
	///    Static  = 0x00000001,
	///    Dynamic = 0x00000002,
	///    Debris  = 0x00000004,
	///    Player  = 0x00000008,
	///    // etc
	/// };
	/// @endcode
	public ulong categoryBits;

	/// The collision mask bits. This states the categories that this shape would accept for
	/// collision. For example, you may want your player to only collide with static objects and
	/// other players.
	/// @code{.c}
	/// maskBits = Static | Player;
	/// @endcode
	public ulong maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative) or always
	/// collide (positive). A group index of zero has no effect. Non-zero group filtering always
	/// wins against the mask bits. For example, you may want ragdolls to collide with other
	/// ragdolls but you don't want ragdoll self-collision. In this case you would give each ragdoll
	/// a unique negative group index and apply that group index to all shapes on the ragdoll.
	public int groupIndex;
}

/// Use this to initialize your filter
/// @ingroup shape

/// The query filter is used to filter collisions between queries and shapes. For example, you may
/// want a ray-cast representing a projectile to hit players and the static environment but not
/// debris.
/// @ingroup shape
public struct QueryFilter
{
	/// The collision category bits of this query. Normally you would just set one bit.
	public ulong categoryBits;

	/// The collision mask bits. This states the shape categories that this query would accept for
	/// collision.
	public ulong maskBits;
}

/// Use this to initialize your query filter
/// @ingroup shape

/// Shape type
/// @ingroup shape
public enum ShapeType
{
	/// A circle with an offset
	circleShape,

	/// A capsule is an extruded circle
	capsuleShape,

	/// A line segment
	segmentShape,

	/// A convex polygon
	polygonShape,

	/// A line segment owned by a chain shape
	chainSegmentShape,

	/// The number of shape types
	shapeTypeCount
}

/// The surface material groups together parameters related to the shape surface, which is the 2D
/// boundary. Surface materials allow chain shapes to have per segment properties.
/// @ingroup shape
public struct SurfaceMaterial
{
	/// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	public float friction;

	/// The coefficient of restitution (bounce) usually in the range [0,1].
	/// https://en.wikipedia.org/wiki/Coefficient_of_restitution
	public float restitution;

	/// The rolling resistance usually in the range [0,1].
	public float rollingResistance;

	/// The tangent speed for conveyor belts
	public float tangentSpeed;

	/// User material identifier. This is passed with query results and to friction and restitution
	/// combining functions. It is not used internally.
	public ulong userMaterialId;

	/// Custom debug draw color.
	public uint customColor;
}

/// Use this to initialize your surface material
/// @ingroup shape

/// Used to create a shape. This is a temporary object used to bundle shape creation parameters. You
/// may use the same shape definition to create multiple shapes. Must be initialized using
/// DefaultShapeDef().
/// @ingroup shape
public struct ShapeDef
{
	/// Use this to store application specific shape data.
	public object? userData;

	/// The surface material for this shape.
	public SurfaceMaterial material;

	/// The density, usually in kg/m^2. This is not part of the surface material because this is for
	/// the interior, which may have other considerations, such as being hollow. For example a wood
	/// barrel may be hollow or full of water.
	public float density;

	/// Collision filtering data.
	public Filter filter;

	/// Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See
	/// World_SetCustomFilterCallback.
	public bool enableCustomFiltering;

	/// A sensor shape generates overlap events but never generates a collision response. Fast
	/// moving sensors may miss overlaps. Instead, use a ray or shape cast for those scenarios.
	/// Sensors still contribute to the body mass if they have non-zero density.
	/// @note Sensor events are disabled by default.
	/// @see enableSensorEvents
	public bool isSensor;

	/// Enable sensor events for this shape. This applies to sensors and non-sensors. Both shapes
	/// involved must have this flag set to true. False by default, even for sensors.
	public bool enableSensorEvents;

	/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Only one
	/// shape involved needs this flag set to true. Ignored for sensors. False by default.
	public bool enableContactEvents;

	/// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Only one
	/// shape involved needs this flag set to true. Ignored for sensors. False by default.
	public bool enableHitEvents;

	/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are
	/// expensive and must be carefully handled due to multithreading. Ignored for sensors.
	public bool enablePreSolveEvents;

	/// When shapes are created they will scan the environment for collision the next time step.
	/// This can significantly slow down static body creation when there are many static shapes.
	/// This flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
	public bool invokeContactCreation;

	/// Should the body update the mass properties when this shape is created. Default is true.
	/// Warning: if this is false, you MUST call Body_UpdateMassFromShapes or Body_SetMassData
	/// before simulating the world.
	public bool updateBodyMass;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your shape definition
/// @ingroup shape

/// Used to create a chain of line segments. This is designed to eliminate ghost collisions with
/// some limitations.
/// - chains are one-sided
/// - chains have no mass and should be used on static bodies
/// - chains have a counter-clockwise winding order (normal points right of segment direction)
/// - chains are either a loop or open
/// - the distance between any two points must be greater than LinearSlop
/// - a chain shape should not self intersect (this is not validated)
/// - an open chain shape needs a leading and trailing ghost point.
/// - you may overlap two open chains using the ghost points to get smooth collision.
/// - a chain shape creates multiple line segment shapes on the body
/// https://en.wikipedia.org/wiki/Polygonal_chain
/// Must be initialized using DefaultChainDef().
/// @warning Do not use chain shapes unless you understand the limitations. This is an advanced
/// feature.
/// @ingroup chain
public struct ChainDef
{
	/// Use this to store application specific shape data.
	public object? userData;

	/// Array of chain points. Each consecutive pair forms a segment. For a loop the last and first
	/// point form a segment. Cloned.
	public Vec2[] points;

	/// The point count. At least 2 for an open chain and at least 3 for a loop.
	/// segmentCount = isLoop ? pointCount : pointCount - 1
	public int pointCount;

	/// Leading ghost point for an open chain. Distance from first point must be greater than
	/// LinearSlop. Ignored for loops.
	public Vec2 ghost1;

	/// Ending ghost point for an open chain. Distance from last point must be greater than
	/// LinearSlop. Ignored for loops.
	public Vec2 ghost2;

	/// One material for the whole chain or one for each segment. Cloned.
	public SurfaceMaterial[] materials;

	/// The material count. Must be 1 or the segment count.
	public int materialCount;

	/// Contact filtering data.
	public Filter filter;

	/// Indicates a closed chain formed by connecting the first and last point.
	public bool isLoop;

	/// Enable sensors to detect this chain. False by default.
	public bool enableSensorEvents;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your chain definition
/// @ingroup chain

/// Profiling data. Times are in milliseconds.
/// @ingroup world
public struct Profile
{
	public float step;				   ///<
	public float pairs;			   ///<
	public float collide;			   ///<
	public float solve;			   ///<
	public float solverSetup;		   ///<
	public float constraints;		   ///<
	public float prepareConstraints;  ///<
	public float integrateVelocities; ///<
	public float warmStart;		   ///<
	public float solveImpulses;	   ///<
	public float integratePositions;  ///<
	public float relaxImpulses;	   ///<
	public float restitution;		   ///<
	public float storeImpulses;	   ///<
	public float splitIslands;		   ///<
	public float transforms;		   ///<
	public float sensorHits;		   ///<
	public float jointEvents;		   ///<
	public float hitEvents;		   ///<
	public float refit;			   ///<
	public float bullets;			   ///<
	public float sleepIslands;		   ///<
	public float sensors;			   ///<
}

/// Counters that give details of the simulation size.
/// @ingroup world
public struct Counters
{
	public int byteCount;	  ///<
	public int bodyCount;		  ///<
	public int shapeCount;		  ///<
	public int contactCount;	  ///<
	public int jointCount;		  ///<
	public int islandCount;	  ///<
	public int stackUsed;		  ///<
	public int staticTreeHeight; ///<
	public int treeHeight;		  ///<
	public int taskCount;		  ///<
	public FixedArray24<int> colorCounts;  ///<

	/// Number of contacts touched by the collide pass (graph contacts + awake-set non-touching).
	public int awakeContactCount;

	/// Number of contacts recycled in the most recent step.
	public int recycledContactCount;

}

/// Joint type enumeration
///
/// This is useful because all joint types use JointId and sometimes you want to get the type of a
/// joint.
/// @ingroup joint
public enum JointType
{
	distanceJoint,  ///< constrain the distance between two points
	filterJoint,	   ///< filter collision between two bodies
	motorJoint,	   ///< fully control the movement of a body
	moverJoint,	   ///< control the movement of the dynamic character mover
	pogoJoint,	   ///< ground interaction for the dynamic character mover
	prismaticJoint, ///< slide along an axis
	revoluteJoint,  ///< rotate around a pivot
	weldJoint,	   ///< bring two bodies together with some flex
	wheelJoint,	   ///< wheel spins with shock absorber
}

/// Base joint definition used by all joint types. The local frames are measured from the body's
/// origin rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken.
/// @ingroup joint
public struct JointDef
{
	/// User data pointer
	public object? userData;

	/// The first attached body
	public BodyId bodyIdA;

	/// The second attached body
	public BodyId bodyIdB;

	/// The first local joint frame
	public Transform localFrameA;

	/// The second local joint frame
	public Transform localFrameB;

	/// Force threshold for joint events
	public float forceThreshold;

	/// Torque threshold for joint events
	public float torqueThreshold;

	/// Constraint hertz (advanced feature)
	public float constraintHertz;

	/// Constraint damping ratio (advanced feature)
	public float constraintDampingRatio;

	/// Debug draw scale
	public float drawScale;

	/// Set this flag to true if the attached bodies should collide
	public bool collideConnected;

}

/// Distance joint definition Connects a point on body A with a point on body B by a segment. Useful
/// for ropes and springs.
/// @ingroup distance_joint
public struct DistanceJointDef
{
	/// Base joint definition
	public JointDef @base;

	/// The rest length of this joint. Clamped to a stable minimum value.
	public float length;

	/// Enable the distance constraint to behave like a spring. If false then the distance joint
	/// will be rigid, overriding the limit and motor.
	public bool enableSpring;

	/// The lower spring force controls how much tension it can sustain.
	public float lowerSpringForce;

	/// The upper spring force controls how much compression it can sustain.
	public float upperSpringForce;

	/// The spring linear stiffness Hertz, cycles per second.
	public float hertz;

	/// The spring linear damping ratio, non-dimensional
	public float dampingRatio;

	/// Enable/disable the joint limit
	public bool enableLimit;

	/// Minimum length for limit. Clamped to a stable minimum value.
	public float minLength;

	/// Maximum length for limit. Must be greater than or equal to the minimum length.
	public float maxLength;

	/// Enable/disable the joint motor
	public bool enableMotor;

	/// The maximum motor force, usually in newtons
	public float maxMotorForce;

	/// The desired motor speed, usually in meters per second
	public float motorSpeed;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your joint definition
/// @ingroup distance_joint

/// A filter joint is used to disable collision between two specific bodies.
///
/// @ingroup filter_joint
public struct FilterJointDef
{
	/// Base joint definition
	public JointDef @base;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your joint definition
/// @ingroup filter_joint

/// A motor joint is used to control the relative velocity and/or transform between two bodies. With
/// a velocity of zero this acts like top-down friction.
/// @ingroup motor_joint
public struct MotorJointDef
{
	/// Base joint definition
	public JointDef @base;

	/// The desired linear velocity
	public Vec2 linearVelocity;

	/// The maximum motor force in newtons
	public float maxVelocityForce;

	/// The desired angular velocity
	public float angularVelocity;

	/// The maximum motor torque in newton-meters
	public float maxVelocityTorque;

	/// Linear spring hertz for position control
	public float linearHertz;

	/// Linear spring damping ratio
	public float linearDampingRatio;

	/// Maximum spring force in newtons
	public float maxSpringForce;

	/// Angular spring hertz for position control
	public float angularHertz;

	/// Angular spring damping ratio
	public float angularDampingRatio;

	/// Maximum spring torque in newton-meters
	public float maxSpringTorque;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your joint definition
/// @ingroup motor_joint

/// A mover joint is used to move a dynamic character mover through velocity commands. The x and y
/// directions are handled separately. Does not affect rotation.
/// @ingroup mover_joint
public struct MoverJointDef
{
	/// Base joint definition
	public JointDef @base;

	/// The desired linear velocity
	public Vec2 linearVelocity;

	/// The maximum motor force in newtons.
	public Vec2 maxVelocityForce;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your joint definition
/// @ingroup mover_joint

/// Pogo joint definition
/// @ingroup pogo_joint
public struct PogoJointDef
{
	/// Base joint definition
	public JointDef @base;

	/// Normal vector at the contact
	public Vec2 normal;

	/// The spring stiffness Hertz, cycles per second
	public float hertz;

	/// The spring damping ratio, non-dimensional
	public float dampingRatio;

	/// Spring length.
	public float restLength;

	/// Maximum tension force. Controls how sticky the mover is to ground.
	public float maxTensionForce;

	/// Maximum compression force. How hard the pogo pushes the mover up from the ground.
	public float maxCompressionForce;

	/// The initial pogo internal impulse. The pogo is typically recreated every frame. This gives a
	/// way to move the internal state forward for smoother spring behavior.
	public float impulse;

	/// The initial pogo internal velocity. The pogo is typically recreated every frame. This gives
	/// a way to move the internal state forward for smoother spring behavior.
	public float velocity;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your joint definition
/// @ingroup pogo_joint

/// Prismatic joint definition Body B may slide along the x-axis in local frame A. Body B cannot
/// rotate relative to body A. The joint translation is zero when the local frame origins coincide
/// in world space.
/// @ingroup prismatic_joint
public struct PrismaticJointDef
{
	/// Base joint definition
	public JointDef @base;

	/// Enable a linear spring along the prismatic joint axis
	public bool enableSpring;

	/// The spring stiffness Hertz, cycles per second
	public float hertz;

	/// The spring damping ratio, non-dimensional
	public float dampingRatio;

	/// The target translation for the joint in meters. The spring-damper will drive to this
	/// translation.
	public float targetTranslation;

	/// Enable/disable the joint limit
	public bool enableLimit;

	/// The lower translation limit
	public float lowerTranslation;

	/// The upper translation limit
	public float upperTranslation;

	/// Enable/disable the joint motor
	public bool enableMotor;

	/// The maximum motor force, typically in newtons
	public float maxMotorForce;

	/// The desired motor speed, typically in meters per second
	public float motorSpeed;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your joint definition
/// @ingroup prismatic_joint

/// Revolute joint definition A point on body B is fixed to a point on body A. Allows relative
/// rotation.
/// @ingroup revolute_joint
public struct RevoluteJointDef
{
	/// Base joint definition
	public JointDef @base;

	/// The target angle for the joint in radians. The spring-damper will drive to this angle.
	public float targetAngle;

	/// Enable a rotational spring on the revolute hinge axis
	public bool enableSpring;

	/// The spring stiffness Hertz, cycles per second
	public float hertz;

	/// The spring damping ratio, non-dimensional
	public float dampingRatio;

	/// A flag to enable joint limits
	public bool enableLimit;

	/// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
	public float lowerAngle;

	/// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
	public float upperAngle;

	/// A flag to enable the joint motor
	public bool enableMotor;

	/// The maximum motor torque, typically in newton-meters
	public float maxMotorTorque;

	/// The desired motor speed in radians per second
	public float motorSpeed;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your joint definition.
/// @ingroup revolute_joint

/// Weld joint definition Connects two bodies together rigidly. This constraint provides springs to
/// mimic soft-body simulation.
/// @note The approximate solver in Box2D cannot hold many bodies together rigidly
/// @ingroup weld_joint
public struct WeldJointDef
{
	/// Base joint definition
	public JointDef @base;

	/// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	public float linearHertz;

	/// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	public float angularHertz;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	public float linearDampingRatio;

	/// Angular damping ratio, non-dimensional. Use 1 for critical damping.
	public float angularDampingRatio;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your joint definition
/// @ingroup weld_joint

/// Wheel joint definition Body B is a wheel that may rotate freely and slide along the local x-axis
/// in frame A. The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup wheel_joint
public struct WheelJointDef
{
	/// Base joint definition
	public JointDef @base;

	/// Enable a linear spring along the local axis
	public bool enableSpring;

	/// Spring stiffness in Hertz
	public float hertz;

	/// Spring damping ratio, non-dimensional
	public float dampingRatio;

	/// Enable/disable the joint linear limit
	public bool enableLimit;

	/// The lower translation limit
	public float lowerTranslation;

	/// The upper translation limit
	public float upperTranslation;

	/// Enable/disable the joint rotational motor
	public bool enableMotor;

	/// The maximum motor torque, typically in newton-meters
	public float maxMotorTorque;

	/// The desired motor speed in radians per second
	public float motorSpeed;

	/// Used internally to detect a valid definition. DO NOT SET.
	public int internalValue;
}

/// Use this to initialize your joint definition
/// @ingroup wheel_joint

/// The explosion definition is used to configure options for explosions. Explosions consider shape
/// geometry when computing the impulse.
/// @ingroup world
public struct ExplosionDef
{
	/// Mask bits to filter shapes
	public ulong maskBits;

	/// The center of the explosion in world space
	public Pos position;

	/// The radius of the explosion
	public float radius;

	/// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
	public float falloff;

	/// Impulse per unit length. This applies an impulse according to the shape perimeter that is
	/// facing the explosion. Explosions only apply to circles, capsules, and polygons. This may be
	/// negative for implosions.
	public float impulsePerLength;
}

/// Use this to initialize your explosion definition
/// @ingroup world

/**
 * @defgroup events Events
 * World event types.
 *
 * Events are used to collect events that occur during the world time step. These events are then
 * available to query after the time step is complete. This is preferable to callbacks because Box2D
 * uses multithreaded simulation.
 *
 * Also when events occur in the simulation step it may be problematic to modify the world, which is
 * often what applications want to do when events occur.
 *
 * With event arrays, you can scan the events in a loop and modify the world. However, you need to
 * be careful that some event data may become invalid. There are several samples that show how to do
 * this safely.
 *
 * @{
 */

/// A begin touch event is generated when a shape starts to overlap a sensor shape.
public struct SensorBeginTouchEvent
{
	/// The id of the sensor shape
	public ShapeId sensorShapeId;

	/// The id of the shape that began touching the sensor shape
	public ShapeId visitorShapeId;
}

/// An end touch event is generated when a shape stops overlapping a sensor shape. Normally this
/// event is generated from simulation movement. However, this event can also be generated from API
/// interactions: setting the body transform, destroying a body or shape, or changing a filter. You
/// will also get an end event if the sensor is destroyed. Therefore you should always confirm the
/// shape id is valid using Shape_IsValid.
public struct SensorEndTouchEvent
{
	/// The id of the sensor shape
	/// @warning this shape may have been destroyed
	/// @see Shape_IsValid
	public ShapeId sensorShapeId;

	/// The id of the shape that stopped touching the sensor shape
	/// @warning this shape may have been destroyed
	/// @see Shape_IsValid
	public ShapeId visitorShapeId;

}

/// Sensor events are buffered in the world and are available as begin/end overlap event arrays
/// after the time step is complete. Note: these may become invalid if bodies and/or shapes are
/// destroyed
public struct SensorEvents
{
	/// Array of sensor begin touch events
	public SensorBeginTouchEvent[] beginEvents;

	/// Array of sensor end touch events
	public SensorEndTouchEvent[] endEvents;

	/// The number of begin touch events
	public int beginCount;

	/// The number of end touch events
	public int endCount;
}

/// A begin touch event is generated when two shapes begin touching.
public struct ContactBeginTouchEvent
{
	/// Id of the first shape
	public ShapeId shapeIdA;

	/// Id of the second shape
	public ShapeId shapeIdB;

	/// The transient contact id. This contact may be destroyed automatically when the world is
	/// modified or simulated. Use Contact_IsValid before using this id.
	public ContactId contactId;
}

/// An end touch event is generated when two shapes stop touching. You will get an end event if you
/// do anything that destroys contacts previous to the last world step. These include things like
/// setting the transform, destroying a body or shape, or changing a filter or body type.
public struct ContactEndTouchEvent
{
	/// Id of the first shape
	/// @warning this shape may have been destroyed
	/// @see Shape_IsValid
	public ShapeId shapeIdA;

	/// Id of the second shape
	/// @warning this shape may have been destroyed
	/// @see Shape_IsValid
	public ShapeId shapeIdB;

	/// Id of the contact.
	/// @warning this contact may have been destroyed
	/// @see Contact_IsValid
	public ContactId contactId;
}

/// A hit event is generated when two shapes collide with a speed faster than the hit event
/// threshold. This may be reported for speculative contacts that have a confirmed impulse.
public struct ContactHitEvent
{
	/// Id of the first shape
	public ShapeId shapeIdA;

	/// Id of the second shape
	public ShapeId shapeIdB;

	/// Id of the contact.
	/// @warning this contact may have been destroyed
	/// @see Contact_IsValid
	public ContactId contactId;

	/// Point where the shapes hit. This is a mid-point between the two surfaces. It could be a
	/// speculative point where the shapes were not touching at the beginning of the time step. The
	/// point is reconstructed after the step, so when both bodies are moving it may trail the
	/// impact by up to one step of motion.
	public Pos point;

	/// Normal vector pointing from shape A to shape B
	public Vec2 normal;

	/// The speed the shapes are approaching. Always positive. Typically in meters per second.
	public float approachSpeed;
}

/// Contact events are buffered in the Box2D world and are available as event arrays after the time
/// step is complete. Note: these may become invalid if bodies and/or shapes are destroyed
public struct ContactEvents
{
	/// Array of begin touch events
	public ContactBeginTouchEvent[] beginEvents;

	/// Array of end touch events
	public ContactEndTouchEvent[] endEvents;

	/// Array of hit events
	public ContactHitEvent[] hitEvents;

	/// Number of begin touch events
	public int beginCount;

	/// Number of end touch events
	public int endCount;

	/// Number of hit events
	public int hitCount;
}

/// Body move events triggered when a body moves. Triggered when a body moves due to simulation. Not
/// reported for bodies moved by the user. This also has a flag to indicate that the body went to
/// sleep so the application can also sleep that actor/entity/object associated with the body. On
/// the other hand if the flag does not indicate the body went to sleep then the application can
/// treat the actor/entity/object associated with the body as awake. This is an efficient way for an
/// application to update game object transforms rather than calling functions such as
/// Body_GetTransform() because this data is delivered as a contiguous array and it is only
/// populated with bodies that have moved.
/// @note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
public struct BodyMoveEvent
{
	/// The body user data.
	public object? userData;

	/// The world transform of the body.
	public WorldTransform transform;

	/// The id of the body that moved.
	public BodyId bodyId;

	/// Did the body fall asleep this time step?
	public bool fellAsleep;
}

/// Body events are buffered in the Box2D world and are available as event arrays after the time
/// step is complete. Note: this data becomes invalid if bodies are destroyed
public struct BodyEvents
{
	/// Array of move events
	public BodyMoveEvent[] moveEvents;

	/// Number of move events
	public int moveCount;
}

/// Joint events report joints that are awake and have a force and/or torque exceeding the
/// threshold. The observed forces and torques are not returned for efficiency reasons.
public struct JointEvent
{
	/// The joint id
	public JointId jointId;

	/// The user data from the joint for convenience
	public object? userData;
}

/// Joint events are buffered in the world and are available as event arrays after the time step is
/// complete. Note: this data becomes invalid if joints are destroyed
public struct JointEvents
{
	/// Array of events
	public JointEvent[] jointEvents;

	/// Number of events
	public int count;
}

/**@}*/

/// The contact data for two shapes. By convention the manifold normal points from shape A to shape
/// B.
/// @see Shape_GetContactData() and Body_GetContactData()
/// @ingroup contact
public struct ContactData
{
	/// The contact id. You may hold onto this to track a contact across time steps. This id may
	/// become orphaned. Use Contact_IsValid before using it for other functions.
	public ContactId contactId;

	/// The first shape id.
	public ShapeId shapeIdA;

	/// The second shape id.
	public ShapeId shapeIdB;

	/// The manifold copied from the contact.
	public Manifold manifold;
}

/// Prototype for a contact filter callback. This is called when a contact pair is considered for
/// collision. This allows you to perform custom logic to prevent collision between shapes. This is
/// only called if one of the two shapes has custom filtering enabled. Notes:
/// - this function must be thread-safe
/// - this is only called if one of the two shapes has enabled custom filtering
/// - this may be called for awake dynamic bodies and sensors Return false if you want to disable
///   the collision
/// @see ShapeDef
/// @warning Do not attempt to modify the world inside this callback
/// @ingroup world
public delegate bool CustomFilterCallbackFcn( ShapeId shapeIdA, ShapeId shapeIdB, object? context );

/// Prototype for a pre-solve callback. This is called after a contact manifold is updated. This
/// allows you to inspect a manifold before it goes to the solver. If you are careful, you can
/// modify the manifold (e.g. modify the normal). You can set the manifold point count to zero to
/// disable collision. Notes:
/// - this function must be thread-safe
/// - this is only called if the shape has enabled pre-solve events
/// - this is called only for awake dynamic bodies
/// - this is not called for sensors
/// - the supplied manifold impulses are not reliable
/// @warning Do not attempt to modify the world inside this callback
/// @ingroup world
public delegate void PreSolveCallbackFcn( ShapeId shapeIdA, ShapeId shapeIdB, ref Manifold manifold, object? context );

/// Prototype for a pre-continuous callback. This is called when a time of impact event is computed
/// in continuous collision. This function can be used to skip the event. Unlike the pre-solve
/// callback, this only supplies the point and normal. At this stage of the solver the manifold may
/// not exist or may be stale. Notes:
/// - this function must be thread-safe
/// - this is only called if the shape has enabled pre-solve events
/// - this is called only for awake dynamic bodies
/// - this is not called for sensors Return false if you want to disable the time of impact event.
/// @warning Do not attempt to modify the world inside this callback
/// @ingroup world
public delegate bool PreContinuousCallbackFcn( ShapeId shapeIdA, ShapeId shapeIdB, Pos point, Vec2 normal, object? context );

/// Prototype callback for overlap queries. Called for each shape found in the query.
/// @see World_OverlapAABB
/// @return false to terminate the query.
/// @ingroup world
public delegate bool OverlapResultCallbackFcn( ShapeId shapeId, object? context );

/// Prototype callback for ray and shape casts. Called for each shape found in the query. You
/// control how the ray cast proceeds by returning a float:
/// - return -1: ignore this shape and continue
/// - return 0: terminate the ray cast
/// - return fraction: clip the ray to this point
/// - return 1: don't clip the ray and continue A cast with initial overlap will return a zero
///   fraction and a zero normal.
/// @param shapeId the shape hit by the ray
/// @param point the point of initial intersection
/// @param normal the normal vector at the point of intersection, zero for a shape cast with initial
/// overlap
/// @param fraction the fraction along the ray at the point of intersection, zero for a shape cast
/// with initial overlap
/// @param context the user context
/// @return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
/// @see World_CastRay
/// @ingroup world
public delegate float CastResultCallbackFcn( ShapeId shapeId, Pos point, Vec2 normal, float fraction, object? context );

/// Used to collect collision planes for character movers. Return true to continue gathering planes.
/// @ingroup character
public delegate bool PlaneResultCallbackFcn( ShapeId shapeId, in PlaneResult plane, object? context );

/**
 * @defgroup debug Debug Draw
 */

/// These colors are used for debug draw and mostly match the named SVG colors.
/// See https://www.rapidtables.com/web/color/index.html
/// https://johndecember.com/html/spec/colorsvg.html
/// https://upload.wikimedia.org/wikipedia/commons/2/2b/SVG_Recognized_color_keyword_names.svg
/// @ingroup debug
public enum HexColor
{
	colorAliceBlue = 0xF0F8FF,
	colorAntiqueWhite = 0xFAEBD7,
	colorAqua = 0x00FFFF,
	colorAquamarine = 0x7FFFD4,
	colorAzure = 0xF0FFFF,
	colorBeige = 0xF5F5DC,
	colorBisque = 0xFFE4C4,
	colorBlack = 0x000000,
	colorBlanchedAlmond = 0xFFEBCD,
	colorBlue = 0x0000FF,
	colorBlueViolet = 0x8A2BE2,
	colorBrown = 0xA52A2A,
	colorBurlywood = 0xDEB887,
	colorCadetBlue = 0x5F9EA0,
	colorChartreuse = 0x7FFF00,
	colorChocolate = 0xD2691E,
	colorCoral = 0xFF7F50,
	colorCornflowerBlue = 0x6495ED,
	colorCornsilk = 0xFFF8DC,
	colorCrimson = 0xDC143C,
	colorCyan = 0x00FFFF,
	colorDarkBlue = 0x00008B,
	colorDarkCyan = 0x008B8B,
	colorDarkGoldenRod = 0xB8860B,
	colorDarkGray = 0xA9A9A9,
	colorDarkGreen = 0x006400,
	colorDarkKhaki = 0xBDB76B,
	colorDarkMagenta = 0x8B008B,
	colorDarkOliveGreen = 0x556B2F,
	colorDarkOrange = 0xFF8C00,
	colorDarkOrchid = 0x9932CC,
	colorDarkRed = 0x8B0000,
	colorDarkSalmon = 0xE9967A,
	colorDarkSeaGreen = 0x8FBC8F,
	colorDarkSlateBlue = 0x483D8B,
	colorDarkSlateGray = 0x2F4F4F,
	colorDarkTurquoise = 0x00CED1,
	colorDarkViolet = 0x9400D3,
	colorDeepPink = 0xFF1493,
	colorDeepSkyBlue = 0x00BFFF,
	colorDimGray = 0x696969,
	colorDodgerBlue = 0x1E90FF,
	colorFireBrick = 0xB22222,
	colorFloralWhite = 0xFFFAF0,
	colorForestGreen = 0x228B22,
	colorFuchsia = 0xFF00FF,
	colorGainsboro = 0xDCDCDC,
	colorGhostWhite = 0xF8F8FF,
	colorGold = 0xFFD700,
	colorGoldenRod = 0xDAA520,
	colorGray = 0x808080,
	colorGreen = 0x008000,
	colorGreenYellow = 0xADFF2F,
	colorHoneyDew = 0xF0FFF0,
	colorHotPink = 0xFF69B4,
	colorIndianRed = 0xCD5C5C,
	colorIndigo = 0x4B0082,
	colorIvory = 0xFFFFF0,
	colorKhaki = 0xF0E68C,
	colorLavender = 0xE6E6FA,
	colorLavenderBlush = 0xFFF0F5,
	colorLawnGreen = 0x7CFC00,
	colorLemonChiffon = 0xFFFACD,
	colorLightBlue = 0xADD8E6,
	colorLightCoral = 0xF08080,
	colorLightCyan = 0xE0FFFF,
	colorLightGoldenRodYellow = 0xFAFAD2,
	colorLightGray = 0xD3D3D3,
	colorLightGreen = 0x90EE90,
	colorLightPink = 0xFFB6C1,
	colorLightSalmon = 0xFFA07A,
	colorLightSeaGreen = 0x20B2AA,
	colorLightSkyBlue = 0x87CEFA,
	colorLightSlateGray = 0x778899,
	colorLightSteelBlue = 0xB0C4DE,
	colorLightYellow = 0xFFFFE0,
	colorLime = 0x00FF00,
	colorLimeGreen = 0x32CD32,
	colorLinen = 0xFAF0E6,
	colorMagenta = 0xFF00FF,
	colorMaroon = 0x800000,
	colorMediumAquaMarine = 0x66CDAA,
	colorMediumBlue = 0x0000CD,
	colorMediumOrchid = 0xBA55D3,
	colorMediumPurple = 0x9370DB,
	colorMediumSeaGreen = 0x3CB371,
	colorMediumSlateBlue = 0x7B68EE,
	colorMediumSpringGreen = 0x00FA9A,
	colorMediumTurquoise = 0x48D1CC,
	colorMediumVioletRed = 0xC71585,
	colorMidnightBlue = 0x191970,
	colorMintCream = 0xF5FFFA,
	colorMistyRose = 0xFFE4E1,
	colorMoccasin = 0xFFE4B5,
	colorNavajoWhite = 0xFFDEAD,
	colorNavy = 0x000080,
	colorOldLace = 0xFDF5E6,
	colorOlive = 0x808000,
	colorOliveDrab = 0x6B8E23,
	colorOrange = 0xFFA500,
	colorOrangeRed = 0xFF4500,
	colorOrchid = 0xDA70D6,
	colorPaleGoldenRod = 0xEEE8AA,
	colorPaleGreen = 0x98FB98,
	colorPaleTurquoise = 0xAFEEEE,
	colorPaleVioletRed = 0xDB7093,
	colorPapayaWhip = 0xFFEFD5,
	colorPeachPuff = 0xFFDAB9,
	colorPeru = 0xCD853F,
	colorPink = 0xFFC0CB,
	colorPlum = 0xDDA0DD,
	colorPowderBlue = 0xB0E0E6,
	colorPurple = 0x800080,
	colorRebeccaPurple = 0x663399,
	colorRed = 0xFF0000,
	colorRosyBrown = 0xBC8F8F,
	colorRoyalBlue = 0x4169E1,
	colorSaddleBrown = 0x8B4513,
	colorSalmon = 0xFA8072,
	colorSandyBrown = 0xF4A460,
	colorSeaGreen = 0x2E8B57,
	colorSeaShell = 0xFFF5EE,
	colorSienna = 0xA0522D,
	colorSilver = 0xC0C0C0,
	colorSkyBlue = 0x87CEEB,
	colorSlateBlue = 0x6A5ACD,
	colorSlateGray = 0x708090,
	colorSnow = 0xFFFAFA,
	colorSpringGreen = 0x00FF7F,
	colorSteelBlue = 0x4682B4,
	colorTan = 0xD2B48C,
	colorTeal = 0x008080,
	colorThistle = 0xD8BFD8,
	colorTomato = 0xFF6347,
	colorTurquoise = 0x40E0D0,
	colorViolet = 0xEE82EE,
	colorWheat = 0xF5DEB3,
	colorWhite = 0xFFFFFF,
	colorWhiteSmoke = 0xF5F5F5,
	colorYellow = 0xFFFF00,
	colorYellowGreen = 0x9ACD32,

	colorBox2DRed = 0xDC3132,
	colorBox2DBlue = 0x30AEBF,
	colorBox2DGreen = 0x8CC924,
	colorBox2DYellow = 0xFFEE8C
}

/// Get the visualization color assigned to a constraint graph color slot. The last index
/// (GraphColorCount - 1) is the overflow color.
/// @ingroup debug

/// This holds callbacks you can implement to draw a Box2D world. Callbacks receive world
/// coordinates. In large world mode the translation is double precision so it stays accurate far
/// from the origin. Shift into your own camera frame inside the callbacks. Initialize with
/// DefaultDebugDraw.
/// @ingroup debug
public struct DebugDraw
{
	/// Draw a closed polygon provided in CCW order.
	public DrawPolygonFcnDelegate DrawPolygonFcn;

	public delegate void DrawPolygonFcnDelegate( WorldTransform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, HexColor color, object? context );

	/// Draw a solid closed polygon provided in CCW order.
	public DrawSolidPolygonFcnDelegate DrawSolidPolygonFcn;

	public delegate void DrawSolidPolygonFcnDelegate( WorldTransform transform, ReadOnlySpan<Vec2> vertices, int vertexCount, float radius, HexColor color, object? context );

	/// Draw a circle.
	public DrawCircleFcnDelegate DrawCircleFcn;

	public delegate void DrawCircleFcnDelegate( Pos center, float radius, HexColor color, object? context );

	/// Draw a solid circle.
	public DrawSolidCircleFcnDelegate DrawSolidCircleFcn;

	public delegate void DrawSolidCircleFcnDelegate( WorldTransform transform, Vec2 center, float radius, HexColor color, object? context );

	/// Draw a solid capsule.
	public DrawSolidCapsuleFcnDelegate DrawSolidCapsuleFcn;

	public delegate void DrawSolidCapsuleFcnDelegate( Pos p1, Pos p2, float radius, HexColor color, object? context );

	/// Draw a line segment.
	public DrawLineFcnDelegate DrawLineFcn;

	public delegate void DrawLineFcnDelegate( Pos p1, Pos p2, HexColor color, object? context );

	/// Draw a transform. Choose your own length scale.
	public DrawTransformFcnDelegate DrawTransformFcn;

	public delegate void DrawTransformFcnDelegate( WorldTransform transform, object? context );

	/// Draw a point.
	public DrawPointFcnDelegate DrawPointFcn;

	public delegate void DrawPointFcnDelegate( Pos p, float size, HexColor color, object? context );

	/// Draw a string in world space
	public DrawStringFcnDelegate DrawStringFcn;

	public delegate void DrawStringFcnDelegate( Pos p, string s, HexColor color, object? context );

	/// Draw a bounding box. With double precision enabled, the single precision bounding box gets
	/// increasing padding when moving far from the origin.
	public DrawBoundsFcnDelegate DrawBoundsFcn;

	public delegate void DrawBoundsFcnDelegate( AABB aabb, HexColor color, object? context );

	/// World bounds used for debug draw. An empty box draws nothing.
	public AABB drawingBounds;

	/// Scale to use when drawing forces
	public float forceScale;

	/// Global scaling for joint drawing
	public float jointScale;

	/// Option to draw contact points
	public bool drawContacts;

	/// Draw anchor A for contact points (instead of anchorB)
	public bool drawAnchorA;

	/// Option to draw shapes
	public bool drawShapes;

	/// Option to draw chain shape normals
	public bool drawChainNormals;

	/// Option to draw joints
	public bool drawJoints;

	/// Option to draw additional information for joints
	public bool drawJointExtras;

	/// Option to draw the bounding boxes for shapes
	public bool drawBounds;

	/// Option to draw the mass and center of mass of dynamic bodies
	public bool drawMass;

	/// Option to draw body names
	public bool drawBodyNames;

	/// Option to visualize the graph coloring used for contacts and joints
	public bool drawGraphColors;

	/// Option to draw contact feature ids
	public bool drawContactFeatures;

	/// Option to draw contact normals
	public bool drawContactNormals;

	/// Option to draw contact normal forces
	public bool drawContactForces;

	/// Option to draw contact friction forces
	public bool drawFrictionForces;

	/// Option to draw islands as bounding boxes
	public bool drawIslands;

	/// User context that is passed as an argument to drawing callback functions
	public object? context;
}

/// Use this to initialize your drawing interface. This allows you to implement a subset of the
/// drawing functions.
/// @ingroup debug

/// Opaque handle for incremental playback of a recording.
/// @ingroup replay
/// Static metadata describing a recording, resolved once when the player opens the file.
/// @ingroup replay
public struct ReplayInfo
{
	public int frameCount;	   ///< total recorded steps
	public int workerCount;   ///< worker count used for the replay world
	public float timeStep;	   ///< dt of the recorded steps
	public int subStepCount;  ///< recorded sub-steps
	public float lengthScale; ///< length units per meter in effect when recorded
	public AABB bounds;	   ///< accumulated world bounds over the recording, zero-extent if unavailable
}

/// The kind of a recorded spatial query, matching the public query and cast functions.
/// @ingroup replay
public enum ReplayQueryType
{
	replayQueryOverlapAABB,
	replayQueryOverlapShape,
	replayQueryCastRay,
	replayQueryCastShape,
	replayQueryCollideMover,
	replayQueryCastRayClosest,
	replayQueryCastMover,
}

/// A spatial query recorded during a replayed frame, exposed for inspection.
/// @ingroup replay
public struct ReplayQueryInfo
{
	public ReplayQueryType type; ///< the query type
	public QueryFilter filter;	///< the query filter
	public AABB aabb;			///< overlap AABB, relative to origin
	public Pos origin;			///< query origin
	public Vec2 translation;		///< ray and cast translation
	public int hitCount;			///< number of recorded results
}

/// One result of a recorded spatial query.
/// @ingroup replay
public struct ReplayQueryHit
{
	public ShapeId shapeId; ///< the shape hit
	public Pos point;	   ///< the hit point on the shape surface
	public Vec2 normal;	   ///< the unit normal vector at the hit point
	public float fraction;	   ///< the query translation fraction
}
