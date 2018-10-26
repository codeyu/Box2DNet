using System;

namespace Testbed.Framework
{
    public struct TestEntry
    {
        public Func<Test> CreateTest;
        public string Name;
    }
}