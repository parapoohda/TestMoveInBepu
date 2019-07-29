using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities.Memory;
using Com.Nelalen.GameObject;
using Demos;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace TestBepuPhysic
{
    class Program
    {

        static void Main(string[] args)
        {
            Console.WriteLine("main");
            Map map = new Map();
            map.Init();
            map.AddChar();
        }

       
    }

}
