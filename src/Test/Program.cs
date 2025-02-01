global using Jitter2;
global using Jitter2.Collision;
global using Jitter2.Collision.Shapes;
global using Jitter2.Dynamics;
global using Jitter2.LinearMath;
global using Jitter2.Parallelization;
global using Jitter2.UnmanagedMemory;
using JitterDemo;
using JitterDemo.Renderer;
using JitterTests;
using System.Diagnostics;


namespace Test
{
    internal class Program
    {
        static World world = null;
        static void Main(string[] args)
        {
            Test();
            Console.ReadKey();
            return;

            TickHelper T = new TickHelper();

            T.Tick("new World.Capacity");
            World.Capacity capacity = new World.Capacity
            {
                BodyCount = 1_000,
                ConstraintCount = 1_000,
                ContactCount = 1_000,
                SmallConstraintCount = 1_000
            };
            T.Tick("new World.Capacity end");
            world = new World(capacity);

            Jitter2.Parallelization.ThreadPool.Instance.ChangeThreadCount(4);
            world.AllowDeactivation = false;
            world.SolverIterations = (10, 4);
            world.SpeculativeRelaxationFactor = 0;
            T.Tick("setting end");


            for (int i = 0; i < 1000; i++)
            {
                Console.WriteLine("================");

                Helper.BuildTower(world, JVector.Zero, 5);
                T.Tick("build tower end");

                Console.WriteLine(world.RigidBodies.Count);
                Console.WriteLine(world.DynamicTree.Nodes.Length);
                Helper.AdvanceWorld(world, 2, 0.2f, false);
                Console.ForegroundColor = ConsoleColor.Green;
                T.Tick("AdvanceWorld end");
                Console.ForegroundColor = ConsoleColor.White;
                world.Clear();
                T.Tick("clear end");
            }

            Console.WriteLine("ok");
            Console.ReadKey();
        }

        static void Test()
        {

            World.Capacity capacity = new World.Capacity
            {
                BodyCount = 1_000,
                ConstraintCount = 1_000,
                ContactCount = 1_000,
                SmallConstraintCount = 1_000
            };

            World world = new World(capacity);

            world.SpeculativeRelaxationFactor = 0;
            world.DynamicTree.Filter = World.DefaultDynamicTreeFilter;

            // 关闭重力
            world.Gravity = JVector.Zero;
            world.EnableAuxiliaryContactPoints = true;


            world.DynamicTree.Filter = World.DefaultDynamicTreeFilter;
            world.SubstepCount = 1;
            world.SolverIterations = (3, 0);
            world.BroadPhaseFilter = null;

            // 创建第一个长方体的形状和刚体
            RigidBodyShape boxShape1 = new BoxShape(1.0f, 1.0f, 1.0f);
            RigidBody body1 = world.CreateRigidBody();
            body1.AddShape(boxShape1);
            body1.Position = new JVector(-4.0f, 0.0f, 0.0f);
            body1.Velocity = new JVector(2.0f, 0.0f, 0.0f);
            body1.Friction = 0.0f;
            body1.Restitution = 0.0f;
            body1.Damping = (0, 0);
            body1.EnableSpeculativeContacts = true;

            // 创建第二个长方体的形状和刚体
            RigidBodyShape boxShape2 = new BoxShape(1.0f, 1.0f, 1.0f);
            RigidBody body2 = world.CreateRigidBody();
            body2.AddShape(boxShape2);
            body2.Position = new JVector(4.0f, 0.0f, 0.0f);
            body2.Velocity = new JVector(-2.0f, 0.0f, 0.0f);
            body2.Friction = 0.0f;
            body2.Restitution = 0.0f;
            body2.Damping = (0, 0);
            body2.EnableSpeculativeContacts = true;


            var start = Stopwatch.StartNew();
            // 运行模拟
            for (int i = 0; i < 500; i++)
            {
                world.Step(1.0f / 60.0f); // 每步模拟1/60秒
                Console.WriteLine($"Step {i + 1}:");
                Console.WriteLine($"Body1 Position: {body1.Position}");
                Console.WriteLine($"Body2 Position: {body2.Position}");

                if (body1.Contacts.Count > 0)
                {
                    Console.WriteLine("===============================");
                    start.Stop();
                    Console.WriteLine(start.ElapsedMilliseconds);
                    break;
                }
            }
        }
    }

    public class TickHelper
    {
        DateTime Time { get; set; }
        public TickHelper()
        {
            Time = DateTime.Now;
        }

        public void Tick(string msg = "")
        {
            Console.WriteLine($"{DateTime.Now: ss fff} {(DateTime.Now - Time).TotalMilliseconds} ms : {msg}");
            Time = DateTime.Now;
        }
    }
}
