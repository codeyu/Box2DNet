using System; 
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
            using (var game = new Game1())
            {
                game.Run();
            }
        }
    }
}