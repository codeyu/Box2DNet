﻿using System; 
using System.Numerics;
using Box2DNet.Dynamics;
using Box2DNet.Dynamics;
using Box2DNet.Collision;
using Box2DNet.Common;

namespace HelloWorld
{
    class Program
    {
        // This is a simple example of building and running a simulation
        // using Box2DNet. Here we create a large ground box and a small dynamic
        // box.
        static void Main(string[] args)
        {
            // Define the size of the world. Simulation will still work
            // if bodies reach the end of the world, but it will be slower.
            AABB worldAABB = new AABB
            {
                UpperBound = new Vector2(100, 100),
                LowerBound = new Vector2(-100, -100)
            };
 
            // Define the gravity vector.
            Vector2 gravity = new Vector2(0.0f, -10.0f);
 
            // Do we want to let bodies sleep?
            bool doSleep = true;
 
            // Construct a world object, which will hold and simulate the rigid bodies.
            World world = new World(worldAABB, gravity, doSleep);
 
            // Define the ground body.
            BodyDef groundBodyDef = new BodyDef();
            groundBodyDef.Position = new Vector2(0.0f, -10.0f);
 
            // Call the body factory which  creates the ground box shape.
            // The body is also added to the world.
            Body groundBody = world.CreateBody(groundBodyDef);
 
            // Define the ground box shape.
            PolygonDef groundShapeDef = new PolygonDef();
 
            // The extents are the half-widths of the box.
            groundShapeDef.SetAsBox(50.0f, 10.0f);
 
            // Add the ground shape to the ground body.
            groundBody.CreateFixture(groundShapeDef);
 
            // Define the dynamic body. We set its position and call the body factory.
            BodyDef bodyDef = new BodyDef();
            bodyDef.Position = new Vector2(0.0f, 4.0f);
            Body body = world.CreateBody(bodyDef);
 
            // Define another box shape for our dynamic body.
            PolygonDef shapeDef = new PolygonDef();
            shapeDef.SetAsBox(1.0f, 1.0f);
 
            // Set the box density to be non-zero, so it will be dynamic.
            shapeDef.Density = 1.0f;
 
            // Override the default friction.
            shapeDef.Friction = 0.3f;
 
            // Add the shape to the body.
            body.CreateFixture(shapeDef);
 
            // Now tell the dynamic body to compute it's mass properties base
            // on its shape.
            body.SetMassFromShapes();
 
            // Prepare for simulation. Typically we use a time step of 1/60 of a
            // second (60Hz) and 10 iterations. This provides a high quality simulation
            // in most game scenarios.
            float timeStep = 1.0f / 60.0f;
            int velocityIterations = 8;
            int positionIterations = 1;
 
            // This is our little game loop.
            for (int i = 0; i < 100; ++i)
            {
                // Instruct the world to perform a single step of simulation. It is
                // generally best to keep the time step and iterations fixed.
                world.Step(timeStep, velocityIterations, positionIterations);
 
                // Now print the position and angle of the body.
                Vector2 position = body.GetPosition();
                float angle = body.GetAngle();
 
                Console.WriteLine("Step: {3} - X: {0}, Y: {1}, Angle: {2}", new object[] { position.X.ToString(), position.Y.ToString(), angle.ToString(), i.ToString() });
            }
 
            // When the world destructor is called, all bodies and joints are freed. This can
            // create orphaned pointers, so be careful about your world management.
            Console.ReadLine();
        }
    }
}