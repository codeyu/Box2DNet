using Testbed.Tests;

namespace Testbed.Framework
{
    public static class TestEntries
    {
        public static readonly TestEntry[] TestList =
        {
            
            new TestEntry {Name = "Car", CreateTest = Car.Create},
            
            new TestEntry {Name = "Bridge", CreateTest = Bridge.Create},
            
            new TestEntry {Name = "Simple Test", CreateTest = SimpleTest.Create},

            new TestEntry {Name = null, CreateTest = null}
        };
    }
}