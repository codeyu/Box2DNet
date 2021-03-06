using Box2DNet.Common;
using Box2DNet.Dynamics;
using Testbed.Framework;
using Settings = Testbed.Framework.Settings;

namespace Testbed.Tests
{
    public class Car : Test
	{
		Body _leftWheel;
		Body _rightWheel;
		Body _vehicle;
		RevoluteJoint _leftJoint;
		RevoluteJoint _rightJoint;

		public Car()
		{
			{	// car body
				PolygonDef poly1 = new PolygonDef(), poly2 = new PolygonDef();

				// bottom half
				poly1.VertexCount = 5;
				poly1.Vertices[4].Set(-2.2f, -0.74f);
				poly1.Vertices[3].Set(-2.2f, 0);
				poly1.Vertices[2].Set(1.0f, 0);
				poly1.Vertices[1].Set(2.2f, -0.2f);
				poly1.Vertices[0].Set(2.2f, -0.74f);
				poly1.Filter.GroupIndex = -1;

				poly1.Density = 20.0f;
				poly1.Friction = 0.68f;
				poly1.Filter.GroupIndex = -1;

				// top half
				poly2.VertexCount = 4;
				poly2.Vertices[3].Set(-1.7f, 0);
				poly2.Vertices[2].Set(-1.3f, 0.7f);
				poly2.Vertices[1].Set(0.5f, 0.74f);
				poly2.Vertices[0].Set(1.0f, 0);
				poly2.Filter.GroupIndex = -1;

				poly2.Density = 5.0f;
				poly2.Friction = 0.68f;
				poly2.Filter.GroupIndex = -1;

				BodyDef bd = new BodyDef();
				bd.Position.Set(-35.0f, 2.8f);

				_vehicle = _world.CreateBody(bd);
				_vehicle.CreateFixture(poly1);
				_vehicle.CreateFixture(poly2);
				_vehicle.SetMassFromShapes();
			}

			{	// vehicle wheels
				CircleDef circ = new CircleDef();
				circ.Density = 40.0f;
				circ.Radius = 0.38608f;
				circ.Friction = 0.8f;
				circ.Filter.GroupIndex = -1;

				BodyDef bd = new BodyDef();
				bd.AllowSleep = false;
				bd.Position.Set(-33.8f, 2.0f);

				_rightWheel = _world.CreateBody(bd);
				_rightWheel.CreateFixture(circ);
				_rightWheel.SetMassFromShapes();

				bd.Position.Set(-36.2f, 2.0f);
				_leftWheel = _world.CreateBody(bd);
				_leftWheel.CreateFixture(circ);
				_leftWheel.SetMassFromShapes();
			}

			{	// join wheels to chassis
				Vec2 anchor = new Vec2();
				RevoluteJointDef jd = new RevoluteJointDef();
				jd.Initialize(_vehicle, _leftWheel, _leftWheel.GetWorldCenter());
				jd.CollideConnected = false;
				jd.EnableMotor = true;
				jd.MaxMotorTorque = 10.0f;
				jd.MotorSpeed = 0.0f;
				_leftJoint = (RevoluteJoint)_world.CreateJoint(jd);

				jd.Initialize(_vehicle, _rightWheel, _rightWheel.GetWorldCenter());
				jd.CollideConnected = false;
				_rightJoint = (RevoluteJoint)_world.CreateJoint(jd);
			}

			{	// ground
				PolygonDef box = new PolygonDef();
				box.SetAsBox(19.5f, 0.5f);
				box.Friction = 0.62f;

				BodyDef bd = new BodyDef();
				bd.Position.Set(-25.0f, 1.0f);

				Body ground = _world.CreateBody(bd);
				ground.CreateFixture(box);
			}

			{	// more ground
				PolygonDef box = new PolygonDef();
				BodyDef bd = new BodyDef();

				box.SetAsBox(9.5f, 0.5f, Vec2.Zero, 0.1f * Box2DNet.Common.Settings.Pi);
				box.Friction = 0.62f;
				bd.Position.Set(27.0f - 30.0f, 3.1f);

				Body ground = _world.CreateBody(bd);
				ground.CreateFixture(box);
			}

			{	// more ground
				PolygonDef box = new PolygonDef();
				BodyDef bd = new BodyDef();

				box.SetAsBox(9.5f, 0.5f, Vec2.Zero, -0.1f * Box2DNet.Common.Settings.Pi);
				box.Friction = 0.62f;
				bd.Position.Set(55.0f - 30.0f, 3.1f);

				Body ground = _world.CreateBody(bd);
				ground.CreateFixture(box);
			}

			{	// more ground
				PolygonDef box = new PolygonDef();
				BodyDef bd = new BodyDef();

				box.SetAsBox(9.5f, 0.5f, Vec2.Zero, 0.03f * Box2DNet.Common.Settings.Pi);
				box.Friction = 0.62f;
				bd.Position.Set(41.0f, 2.0f);

				Body ground = _world.CreateBody(bd);
				ground.CreateFixture(box);
			}

			{	// more ground
				PolygonDef box = new PolygonDef();
				BodyDef bd = new BodyDef();

				box.SetAsBox(5.0f, 0.5f, Vec2.Zero, 0.15f * Box2DNet.Common.Settings.Pi);
				box.Friction = 0.62f;
				bd.Position.Set(50.0f, 4.0f);

				Body ground = _world.CreateBody(bd);
				ground.CreateFixture(box);
			}

			{	// more ground
				PolygonDef box = new PolygonDef();
				BodyDef bd = new BodyDef();

				box.SetAsBox(20.0f, 0.5f);
				box.Friction = 0.62f;
				bd.Position.Set(85.0f, 2.0f);

				Body ground = _world.CreateBody(bd);
				ground.CreateFixture(box);
			}
		}

		public static Test Create()
		{
			return new Car();
		}

		public override void Keyboard(System.Windows.Forms.Keys key)
		{
			switch (key)
			{
				case System.Windows.Forms.Keys.A:
					_leftJoint.SetMaxMotorTorque(800.0f);
					_leftJoint.MotorSpeed = 12.0f;
					break;

				case System.Windows.Forms.Keys.S:
					_leftJoint.SetMaxMotorTorque(100.0f);
					_leftJoint.MotorSpeed = 0.0f;
					break;

				case System.Windows.Forms.Keys.D:
					_leftJoint.SetMaxMotorTorque(1200.0f);
					_leftJoint.MotorSpeed = -36.0f;
					break;
			}
		}

		public override void Step(Settings settings)
		{
			OpenGLDebugDraw.DrawString(5, _textLine, "Keys: left = a, brake = s, right = d");
			_textLine += 15;

			base.Step(settings);
		}
	}
}