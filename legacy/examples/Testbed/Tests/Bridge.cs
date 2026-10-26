using Box2DNet.Collision;
using Box2DNet.Dynamics;
using Testbed.Framework;

namespace Testbed.Tests
{
    public class Bridge : Test
	{
		private const int Count = 30;

        private BridgeTest()
        {
            Body ground;
            {
                ground = new Body(World);

                EdgeShape shape = new EdgeShape(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
                ground.CreateFixture(shape);
            }
            {
                Vertices box = PolygonTools.CreateRectangle(0.5f, 0.125f);
                PolygonShape shape = new PolygonShape(box, 20);

                Body prevBody = ground;
                for (int i = 0; i < Count; ++i)
                {
                    Body body = BodyFactory.CreateBody(World);
                    body.BodyType = Body.BodyType.Dynamic;
                    body.Position = new Vector2(-14.5f + 1.0f * i, 5.0f);

                    Fixture fixture = body.CreateFixture(shape);
                    fixture.Friction = 0.2f;

                    Vector2 anchor = new Vector2(-15f + 1.0f * i, 5.0f);
                    RevoluteJoint jd = new RevoluteJoint(prevBody, body, anchor, true);
                    World.AddJoint(jd);

                    prevBody = body;
                }

                Vector2 anchor2 = new Vector2(-15.0f + 1.0f * Count, 5.0f);
                RevoluteJoint jd2 = new RevoluteJoint(ground, prevBody, anchor2, true);
                World.AddJoint(jd2);
            }

            Vertices vertices = new Vertices(3);
            vertices.Add(new Vector2(-0.5f, 0.0f));
            vertices.Add(new Vector2(0.5f, 0.0f));
            vertices.Add(new Vector2(0.0f, 1.5f));

            for (int i = 0; i < 2; ++i)
            {
                PolygonShape shape = new PolygonShape(vertices, 1);

                Body body = BodyFactory.CreateBody(World);
                body.BodyType = Body.BodyType.Dynamic;
                body.Position = new Vector2(-8.0f + 8.0f * i, 12.0f);

                body.CreateFixture(shape);
            }

            for (int i = 0; i < 3; ++i)
            {
                CircleShape shape = new CircleShape(0.5f, 1);

                Body body = BodyFactory.CreateBody(World);
                body.BodyType = Body.BodyType.Dynamic;
                body.Position = new Vector2(-6.0f + 6.0f * i, 10.0f);

                body.CreateFixture(shape);
            }
        }

        internal static Test Create()
        {
            return new BridgeTest();
        }
	}
}