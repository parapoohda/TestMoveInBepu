using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuPhysics;
using BepuPhysics.Collidables;
using TestBepuPhysic;

namespace Com.Nelalen.GameObject
{
        unsafe struct HitHandler : BepuUtilities.IBreakableForEach<CollidableReference>
    {
        public Character character;
        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        public bool LoopBody(CollidableReference collidable)
        {
            return true;
        }
    }

    internal class Character : Unit
    {
        private Map map;
        private CharacterInput characterInput;
        internal CharacterInput CharacterInputt => characterInput;
        private float walkSpeed = 1f;
        private float runSpeed = 2f;
        private bool isRun;
        private bool isCalculateVelocityYet = true;
        internal bool IsCalculateVelocityYet => isCalculateVelocityYet;
        System.Numerics.Vector3 sizeBB = new System.Numerics.Vector3(16, 16, 16);
        System.Numerics.Vector3 target = new System.Numerics.Vector3();
        internal enum BepuTypeId { Sphere, Capsule }
        internal System.Numerics.Vector3 Position => new BepuPhysics.BodyReference(collider.bodyHandle, map.Simulation.Bodies).Pose.Position;
        private BepuTypeId shapeTypeId = BepuTypeId.Capsule;
        private System.Numerics.Vector3 velocity;
        private Demos.Demos.Characters.CharacterControllers characterControllers;
        private System.Numerics.Vector3 viewDirection;
        internal System.Numerics.Vector3 ViewDirection => viewDirection;

        internal System.Numerics.Vector3 Velocity => velocity;

        internal Character(int unitId, string name, System.Numerics.Vector3 startPosition, TestBepuPhysic.Map map) : base(unitId, name, startPosition)
        {
            this.map = map;
            collider.isPassThrough = false;
            collider.type = Collider.Type.Characer;
        }

        internal void CalculateVelocity()
        {
            float moveSpeed;
            if (isRun) {
                moveSpeed = walkSpeed;
            }else {
                moveSpeed = runSpeed;
            }
            velocity = (Position - target)/CalculateMagnitude()* moveSpeed;
            isCalculateVelocityYet = true;
        }

        internal void SetCharacterInput(BepuUtilities.Memory.BufferPool bufferpool, int bodyHandle, Simulation simulation)
        {
            characterControllers = new Demos.Demos.Characters.CharacterControllers(bufferpool);
            collider.bodyHandle = bodyHandle;
            
            characterInput = new CharacterInput(characterControllers, bodyHandle, simulation, this.Position, new Capsule(0.5f, 1), 0.1f, 1, 20, 100, 6, 4, MathF.PI * 0.4f);
        }

        internal float CalculateMagnitude(){
            System.Numerics.Vector3 distance;
            distance = this.target - Position;
            return distance.LengthSquared();
        }

        /*internal virtual void Move(Com.Nelalen.FlatBuffers.Gate.Vector3 target, PlayerCharacterMoveType moveType)
        {
            //for call in player character
            throw new NotImplementedException();
        }*/

        internal void PeriodicAABB()
        {
            var hitHandler = new HitHandler { character = this };
            System.Numerics.Vector3 minBB = Position - (sizeBB/2);
            System.Numerics.Vector3 maxBB = Position + (sizeBB/2);
            var box = new BepuUtilities.BoundingBox(minBB, maxBB);
            map.Simulation.BroadPhase.GetOverlaps(box, ref hitHandler);
        }


        public Capsule Shape() {
            return new Capsule(.15f,1.5f);
        }

        internal BepuTypeId ShapeTypeId => shapeTypeId;
        internal struct CharacterInput
        {
            int bodyHandle;
            Demos.Demos.Characters.CharacterControllers characters;
            float speed;
            Capsule shape;
            Simulation simulation;

            internal int BodyHandle { get { return bodyHandle; } }

            internal CharacterInput(Demos.Demos.Characters.CharacterControllers characters, int bodyHandle, Simulation simulation, System.Numerics.Vector3 initialPosition, Capsule shape,
                float speculativeMargin, float mass, float maximumHorizontalForce, float maximumVerticalGlueForce,
                float jumpVelocity, float speed, float maximumSlope = MathF.PI * 0.25f)
            {
                this.bodyHandle = bodyHandle;
                this.simulation = simulation;
#if DEBUG       
                Console.WriteLine($"======================================= {this.bodyHandle}");
#endif
                this.characters = characters;

                ref var character = ref characters.AllocateCharacter(bodyHandle);
                character.LocalUp = new System.Numerics.Vector3(0, 1, 0);
                character.CosMaximumSlope = MathF.Cos(maximumSlope);
                character.JumpVelocity = jumpVelocity;
                character.MaximumVerticalForce = maximumVerticalGlueForce;
                character.MaximumHorizontalForce = maximumHorizontalForce;
                character.MinimumSupportDepth = shape.Radius * -0.01f;
                character.MinimumSupportContinuationDepth = -speculativeMargin;
                this.speed = speed;
                this.shape = shape;
            }



            internal void UpdateCharacterGoals(System.Numerics.Vector2 velocity, System.Numerics.Vector3 viewDirection, float simulationTimestepDuration)
            {
                var movementDirectionLengthSquared = velocity.LengthSquared();
                System.Numerics.Vector2 movementDirection;
                if (movementDirectionLengthSquared > 0)
                {
                    movementDirection = velocity / MathF.Sqrt(movementDirectionLengthSquared);
                }
                ref var character = ref characters.GetCharacterByBodyHandle(bodyHandle);
#if DEBUG
                Console.WriteLine($" characters: {characters}");
                Console.WriteLine($" simulation : {simulation}");
                //Console.WriteLine($" characters.Simulation.Bodies : {characters.Simulation.Bodies}");
#endif
                var characterBody = new BodyReference(bodyHandle, simulation.Bodies);
                var newTargetVelocity = velocity;
                //Modifying the character's raw data does not automatically wake the character up, so we do so explicitly if necessary.
                //If you don't explicitly wake the character up, it won't respond to the changed motion goals.
                //(You can also specify a negative deactivation threshold in the BodyActivityDescription to prevent the character from sleeping at all.)
                if (!characterBody.Awake &&
                     character.Supported ||
                    newTargetVelocity != character.TargetVelocity ||
                    (newTargetVelocity != System.Numerics.Vector2.Zero))
                {
                    simulation.Awakener.AwakenBody(character.BodyHandle);
                }
                Console.WriteLine(newTargetVelocity);
                character.TargetVelocity = newTargetVelocity;
                character.ViewDirection = viewDirection;

                //The character's motion constraints aren't active while the character is in the air, so if we want air control, we'll need to apply it ourselves.
                //(You could also modify the constraints to do this, but the robustness of solved constraints tends to be a lot less important for air control.)
                //There isn't any one 'correct' way to implement air control- it's a nonphysical gameplay thing, and this is just one way to do it.
                //Note that this permits accelerating along a particular direction, and never attempts to slow down the character.
                //This allows some movement quirks common in some game character controllers.
                //Consider what happens if, starting from a standstill, you accelerate fully along X, then along Z- your full velocity magnitude will be sqrt(2) * maximumAirSpeed.
                //Feel free to try alternative implementations. Again, there is no one correct approach.
                if (!character.Supported && movementDirectionLengthSquared > 0)
                {
                    BepuUtilities.Quaternion.Transform(character.LocalUp, characterBody.Pose.Orientation, out var characterUp);
                    var characterRight = System.Numerics.Vector3.Cross(character.ViewDirection, characterUp);
                    var rightLengthSquared = characterRight.LengthSquared();
                    if (rightLengthSquared > 1e-10f)
                    {
                        characterRight /= MathF.Sqrt(rightLengthSquared);
                        var characterForward = System.Numerics.Vector3.Cross(characterUp, characterRight);
                        var worldMovementDirection = characterRight * velocity.X + characterForward * velocity.Y;
                        var currentVelocity = System.Numerics.Vector3.Dot(characterBody.Velocity.Linear, worldMovementDirection);
                        //We'll arbitrarily set air control to be a fraction of supported movement's speed/force.
                        const float airControlForceScale = .2f;
                        const float airControlSpeedScale = .2f;
                        var airAccelerationDt = characterBody.LocalInertia.InverseMass * character.MaximumHorizontalForce * airControlForceScale * simulationTimestepDuration;
                        var maximumAirSpeed = speed * airControlSpeedScale;
                        var targetVelocity = MathF.Min(currentVelocity + airAccelerationDt, maximumAirSpeed);
                        //While we shouldn't allow the character to continue accelerating in the air indefinitely, trying to move in a given direction should never slow us down in that direction.
                        var velocityChangeAlongMovementDirection = MathF.Max(0, targetVelocity - currentVelocity);
                        characterBody.Velocity.Linear += worldMovementDirection * velocityChangeAlongMovementDirection;
                        System.Diagnostics.Debug.Assert(characterBody.Awake, "Velocity changes don't automatically update objects; the character should have already been woken up before applying air control.");
                    }
                }
            }
            /// <summary>
            /// Removes the character's body from the simulation and the character from the associated characters set.
            /// </summary>
            public void Dispose()
            {
                simulation.Shapes.Remove(new BodyReference(bodyHandle, simulation.Bodies).Collidable.Shape);
                simulation.Bodies.Remove(bodyHandle);
                characters.RemoveCharacterByBodyHandle(bodyHandle);
            }
        }

        
    }
}
