using System;
using Box2DNet.Collision;
using Box2DNet.Dynamics;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace Testbed.Framework
{
    public class Test
    {
        internal DebugViewXNA DebugView;
        internal int StepCount;
        internal World World;
        private FixedMouseJoint _fixedMouseJoint;
        internal int TextLine;

        protected Test()
        {
            World = new World(new Vector2(0.0f, -10.0f));

            TextLine = 30;

            World.JointRemoved += JointRemoved;
            World.ContactManager.PreSolve += PreSolve;
            World.ContactManager.PostSolve += PostSolve;

            StepCount = 0;
        }

        public Game1 GameInstance { protected get; set; }

        public virtual void Initialize()
        {
            DebugView = new DebugViewXNA(World);
            DebugView.LoadContent(GameInstance.GraphicsDevice, GameInstance.Content);
        }

        protected virtual void JointRemoved(Joint joint)
        {
            if (_fixedMouseJoint == joint)
                _fixedMouseJoint = null;
        }

        public void DrawTitle(int x, int y, string title)
        {
            DebugView.DrawString(x, y, title);
        }

        public virtual void Update(GameSettings settings, GameTime gameTime)
        {
            float timeStep = Math.Min((float)gameTime.ElapsedGameTime.TotalMilliseconds * 0.001f, (1f / 30f));

            if (settings.Pause)
            {
                if (settings.SingleStep)
                    settings.SingleStep = false;
                else
                    timeStep = 0.0f;

                DrawString("****PAUSED****");
            }
            else
                World.Step(timeStep);

            if (timeStep > 0.0f)
                ++StepCount;
        }

        public virtual void Keyboard(KeyboardManager keyboardManager)
        {
            if (keyboardManager.IsNewKeyPress(Keys.F11))
                WorldSerializer.Serialize(World, "out.xml");

            if (keyboardManager.IsNewKeyPress(Keys.F12))
            {
                World = WorldSerializer.Deserialize("out.xml");
                Initialize();
            }
        }

        public virtual void Gamepad(GamePadState state, GamePadState oldState)
        {
        }

        public virtual void Mouse(MouseState state, MouseState oldState)
        {
            Vector2 position = GameInstance.ConvertScreenToWorld(state.X, state.Y);

            if (state.LeftButton == ButtonState.Released && oldState.LeftButton == ButtonState.Pressed)
                MouseUp();
            else if (state.LeftButton == ButtonState.Pressed && oldState.LeftButton == ButtonState.Released)
                MouseDown(position);

            MouseMove(position);
        }

        private void MouseDown(Vector2 p)
        {
            if (_fixedMouseJoint != null)
                return;

            Fixture fixture = World.TestPoint(p);

            if (fixture != null)
            {
                Body body = fixture.Body;
                _fixedMouseJoint = new FixedMouseJoint(body, p);
                _fixedMouseJoint.MaxForce = 1000.0f * body.Mass;
                World.AddJoint(_fixedMouseJoint);
                body.Awake = true;
            }
        }

        private void MouseUp()
        {
            if (_fixedMouseJoint != null)
            {
                World.RemoveJoint(_fixedMouseJoint);
                _fixedMouseJoint = null;
            }
        }

        private void MouseMove(Vector2 p)
        {
            if (_fixedMouseJoint != null)
                _fixedMouseJoint.WorldAnchorB = p;
        }

        protected virtual void PreSolve(Contact contact, ref Manifold oldManifold)
        {
        }

        protected virtual void PostSolve(Contact contact, ContactVelocityConstraint impulse)
        {
        }

        protected void DrawString(string text)
        {
            DebugView.DrawString(50, TextLine, text);
            TextLine += 15;
        }
    }
}