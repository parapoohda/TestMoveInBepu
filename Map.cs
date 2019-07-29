using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using Com.Nelalen.GameObject;
using Demos;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace TestBepuPhysic
{
    class Map
    {

        private BufferPool bufferPool;
        private Simulation simulation;
        private SimpleThreadDispatcher threadDispatcher;

        private SortedDictionary<int, Character> characters = new SortedDictionary<int, Character>();
        // <handleId, Character(that walk)>
        private SortedDictionary<int, Character> walkCharacters = new SortedDictionary<int, Character>();
        // <handleId, Character>
        private SortedDictionary<int, Unit> handleUnits = new SortedDictionary<int, Unit>();
        private bool isStop;

        public void Init()
        {
            bufferPool = new BufferPool();
            var collider = new BodyProperty<Collider>();
            simulation = Simulation.Create(bufferPool, new CarCallbacks { Collider = collider }, new DemoPoseIntegratorCallbacks(new System.Numerics.Vector3(0, -10, 0)));

            Simulation.Statics.Add(new StaticDescription(new System.Numerics.Vector3(0f, 0f, 0f), new CollidableDescription(Simulation.Shapes.Add(new Box(30, 1, 30)), 0.04f)));
            threadDispatcher = new SimpleThreadDispatcher(Environment.ProcessorCount);
            (new Thread(PoolThread)).Start();
            
        }

        public void AddChar() {
            var character = new PlayerCharacter(1, 1, "test", 1, new System.Numerics.Vector3(0f, 0f, 0f), this);
            AddPlayerCharacter(character);

        }

        void PoolThread()
        {
            while (!Console.KeyAvailable)
            {
#if DEBUG
                //Console.WriteLine($"================================= KeyAvailable");
#endif
                simulation.Timestep(0.03f, threadDispatcher);
                Console.WriteLine(walkCharacters.Count);
                foreach (var character in walkCharacters)
                {
                    Character charac = character.Value;
                    /*if (character.Value.CalculateMagnitude() > 0.5)
                    {*/
                        if (!charac.IsCalculateVelocityYet)
                        {
                            charac.CalculateVelocity();
                        }
                        //charac.MunipulateVelocity();
                        //var velocity = charac.Velocity;
                        charac.CharacterInputt.UpdateCharacterGoals(new System.Numerics.Vector2(-1f, -1f), charac.ViewDirection, 0.03f);
#if DEBUG
                        Console.WriteLine($"charac position : {charac.Position}");
#endif
                        //new BepuPhysics.BodyReference(charac.BodyHandle, simulation.Bodies).Velocity;

                    /*}
                    else
                    {
#if DEBUG
                        Console.WriteLine($"IN ELSE");
#endif
                        walkCharacters.Remove(charac.collider.bodyHandle);
                    }*/
                }
                if (isStop)
                {
                    break;
                }
            }
        }

        internal void AddCharacter(Character character)
        {
            var characterShape = character.Shape();
            characterShape.ComputeInertia(1, out var characterInertia);

            int handle = simulation.Bodies.Add(BodyDescription.CreateDynamic(character.GetStartPosition(), characterInertia, new CollidableDescription(simulation.Shapes.Add(characterShape), 0.1f), new BodyActivityDescription(0.01f)));

            handleUnits.Add(handle, character);
            character.SetBodyHandle(handle);
            character.PeriodicAABB();
            character.SetCharacterInput(bufferPool, handle, simulation);
            if (walkCharacters.TryAdd<int, Character>(1, character)) {
                //walkCharacters.Add(1, character);
                //Console.WriteLine("===========", walkCharacters.Count);
            }
            //character.SetBodyReference(Simulation.Bodies.GetBodyReference(handle));
        }

        internal Simulation Simulation => simulation;

        internal void AddPlayerCharacter(PlayerCharacter character)
        {
            int charId = character.GetClientId();
            if (characters.ContainsKey(charId))
            {
                characters.Remove(charId);
            }
            characters.Add(charId, character);
            AddCharacter(character);

        }

        struct CarCallbacks : INarrowPhaseCallbacks
        {
            public BodyProperty<Collider> Collider;
            public void Initialize(Simulation simulation)
            {
                Collider.Initialize(simulation.Bodies);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
            {
                //It's impossible for two statics to collide, and pairs are sorted such that bodies always come before statics.
                /*if (b.Mobility != CollidableMobility.Static)
                {
                    return SubgroupCollisionFilter.AllowCollision(Properties[a.Handle].Filter, Properties[b.Handle].Filter);
                }*/

                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                return true;
            }

            /*[MethodImpl(MethodImplOptions.AggressiveInlining)]
            unsafe void CreateMaterial(CollidablePair pair, out PairMaterialProperties pairMaterial)
            {
                pairMaterial.FrictionCoefficient = Properties[pair.A.Handle].Friction;
                if (pair.B.Mobility != CollidableMobility.Static)
                {
                    //If two bodies collide, just average the friction.
                    pairMaterial.FrictionCoefficient = (pairMaterial.FrictionCoefficient + Properties[pair.B.Handle].Friction) * 0.5f;
                }
                pairMaterial.MaximumRecoveryVelocity = 2f;
                pairMaterial.SpringSettings = new SpringSettings(30, 1);
            }*/
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void GetMaterial(out PairMaterialProperties pairMaterial)
            {
                pairMaterial = new PairMaterialProperties { FrictionCoefficient = 1, MaximumRecoveryVelocity = 2, SpringSettings = new SpringSettings(30, 1) };
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
            {
                GetMaterial(out pairMaterial);
                return true;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
            {
                GetMaterial(out pairMaterial);
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold)
            {
                return true;
            }

            public void Dispose()
            {
                Collider.Dispose();
            }
        }
        
    }
    public struct DemoPoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public Vector3 Gravity;
        public float LinearDamping;
        public float AngularDamping;
        Vector3 gravityDt;
        float linearDampingDt;
        float angularDampingDt;

        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

        public DemoPoseIntegratorCallbacks(Vector3 gravity, float linearDamping = .03f, float angularDamping = .03f) : this()
        {
            Gravity = gravity;
            LinearDamping = linearDamping;
            AngularDamping = angularDamping;
        }

        public void PrepareForIntegration(float dt)
        {
            //No reason to recalculate gravity * dt for every body; just cache it ahead of time.
            gravityDt = Gravity * dt;
            //Since this doesn't use per-body damping, we can precalculate everything.
            linearDampingDt = MathF.Pow(MathHelper.Clamp(1 - LinearDamping, 0, 1), dt);
            angularDampingDt = MathF.Pow(MathHelper.Clamp(1 - AngularDamping, 0, 1), dt);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IntegrateVelocity(int bodyIndex, in RigidPose pose, in BodyInertia localInertia, int workerIndex, ref BodyVelocity velocity)
        {
            //Note that we avoid accelerating kinematics. Kinematics are any body with an inverse mass of zero (so a mass of ~infinity). No force can move them.
            if (localInertia.InverseMass > 0)
            {
                velocity.Linear = (velocity.Linear + gravityDt) * linearDampingDt;
                velocity.Angular = velocity.Angular * angularDampingDt;
            }
            //Implementation sidenote: Why aren't kinematics all bundled together separately from dynamics to avoid this per-body condition?
            //Because kinematics can have a velocity- that is what distinguishes them from a static object. The solver must read velocities of all bodies involved in a constraint.
            //Under ideal conditions, those bodies will be near in memory to increase the chances of a cache hit. If kinematics are separately bundled, the the number of cache
            //misses necessarily increases. Slowing down the solver in order to speed up the pose integrator is a really, really bad trade, especially when the benefit is a few ALU ops.

            //Note that you CAN technically modify the pose in IntegrateVelocity. The PoseIntegrator has already integrated the previous velocity into the position, but you can modify it again
            //if you really wanted to.
            //This is also a handy spot to implement things like position dependent gravity or per-body damping.
        }

    }
}
