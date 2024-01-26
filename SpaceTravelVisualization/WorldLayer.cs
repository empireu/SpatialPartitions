using System.Numerics;
using Common;
using GameFramework;
using GameFramework.Extensions;
using GameFramework.ImGui;
using ImGuiNET;
using Veldrid;

namespace SpaceTravelVisualization;

internal sealed class WorldLayer : VisualizationLayer3d
{
    private sealed class Simulation
    {
        private readonly double _shipMass;

        public Vector3d ShipPos { get; private set; }
        public Vector3d ShipVel { get; private set; }
        public Vector3d TargetPos { get; private set; }
        public Vector3d TargetVel { get; private set; }

        public readonly TrajectoryLeg[] Trajectory;

        public double Time { get; private set; }

        public bool Done { get; private set; }

        public Simulation(
            Vector3d shipPos, 
            Vector3d shipVel, 
            Vector3d targetPos, 
            Vector3d targetVel,
            double shipMass,
            TrajectoryLeg[] trajectory
        ) 
        {
            ShipPos = shipPos;
            ShipVel = shipVel;
            TargetPos = targetPos;
            TargetVel = targetVel;
            _shipMass = shipMass;
            Trajectory = trajectory;
        }

        public readonly List<Vector3d> History = new();

        public void Update(double dt)
        {
            if (Done)
            {
                return;
            }

            TrajectoryLeg leg;

            if (Trajectory.Length == 1)
            {
                if (Time < Trajectory[0].Duration)
                {
                    leg = Trajectory[0];
                }
                else
                {
                    Done = true;
                    return;
                }
            }
            else
            {
                if (Time < Trajectory[0].Duration)
                {
                    leg = Trajectory[0];
                }
                else if (Time < Trajectory[0].Duration + Trajectory[1].Duration)
                {
                    leg = Trajectory[1];
                }
                else
                {
                    Done = true;
                    return;
                }
            }

            ShipVel += (leg.Force / _shipMass) * dt;
            ShipPos += ShipVel * dt;
            TargetPos += TargetVel * dt;
            Time += dt;

            if (History.Count == 0)
            {
                History.Add(ShipPos);
            }
            else if (Vector3d.DistanceSqr(History.Last(), ShipPos) > 0.25)
            {
                History.Add(ShipPos);
            }
        }

        public static Simulation? Create(
            Vector3d shipPos, Vector3d shipVel, 
            Vector3d targetPos, Vector3d targetVel,
            double shipForce, double shipMass
        )
        {
            var parameters = new TrajectoryParameters(shipPos, shipVel, targetPos, targetVel, shipForce, shipMass);
            var trajectory = TrajectoryGenerator.ComputeRendezvousTrajectory(parameters);

            if (trajectory == null)
            {
                return null;
            }

            return new Simulation(shipPos, shipVel, targetPos, targetVel, shipMass, trajectory);
        }
    }

    private Vector3 _shipPos;
    private Vector3 _shipVel;
    private Vector3 _targetPos = Vector3.One;
    private Vector3 _targetVel;
    private double _force = 1f;
    private double _mass = 1;
    private float _speed = 1;

    private Simulation? _simulation;
    private string _lastMessage = "Waiting";

    public WorldLayer(VisualizationApp app, ImGuiLayer imGui) : base(app, imGui)
    {
        CameraController.MoveSpeed = 5f;
    }

    protected override void ImGuiOnSubmit(ImGuiRenderer sender)
    {
        if (ImGui.Begin("Space"))
        {
            ImGui.InputFloat3("Ship Pos", ref _shipPos);
            ImGui.InputFloat3("Ship Vel", ref _shipVel);
            ImGui.InputDouble("Force", ref _force);
            ImGui.InputDouble("Mass", ref _mass);
            ImGui.InputFloat3("Target Pos", ref _targetPos);
            ImGui.InputFloat3("Target Vel", ref _targetVel);

            ImGui.Separator();
            ImGui.Text(_lastMessage);

            if (ImGui.Button("Start"))
            {
                _simulation = Simulation.Create(_shipPos, _shipVel, _targetPos, _targetVel, _force, _mass);

                if (_simulation == null)
                {
                    _lastMessage = "Failed to create trajectory";
                }
            }

            ImGui.Separator();

            ImGui.SliderFloat("Speed", ref _speed, 0, 1);

            if (_simulation != null)
            {
                ImGui.Text($"t1={_simulation.Trajectory[0].Duration:F4}, t2={_simulation.Trajectory[1].Duration:F4}");
                ImGui.Text($"T={_simulation.Trajectory[0].Duration + _simulation.Trajectory[1].Duration:F4}");
                ImGui.Text($"f1={_simulation.Trajectory[0].Force.Norm:F4}, f2={_simulation.Trajectory[1].Force.Norm:F4}");

                ImGui.Separator();

                ImGui.Text(_simulation.Done ? "T Done" : $"T+{_simulation.Time:F4}");
                ImGui.Text($"Ship [Dx={_simulation.ShipVel.X:F2}, Dy={_simulation.ShipVel.Y:F2}, Dz={_simulation.ShipVel.Z:F2}]");
                ImGui.Text($"Distance: {Vector3d.Distance(_simulation.ShipPos, _simulation.TargetPos):F6}");
            }
        }

        ImGui.End();
    }

    protected override void Update(FrameInfo frameInfo)
    {
        base.Update(frameInfo);

        const int subSteps = 10_000;
        var dt = ((double)frameInfo.DeltaTime) / subSteps * _speed;

        for (var i = 0; i < subSteps; i++)
        {
            _simulation?.Update(dt);
        }
    }

    protected override void RenderStack()
    {
        RenderPassMain(batch =>
        {
            const float axisThickness = 0.01f;
            batch.ColoredQuadBox(Matrix4x4.CreateScale(2f, axisThickness, axisThickness), new QuadColors(1f, 0f, 0f, 0.5f));
            batch.ColoredQuadBox(Matrix4x4.CreateScale(axisThickness, 2f, axisThickness), new QuadColors(0f, 1f, 0f, 0.5f));
            batch.ColoredQuadBox(Matrix4x4.CreateScale(axisThickness, axisThickness, 2f), new QuadColors(0f, 0f, 1f, 0.5f));

            if (_simulation != null)
            {
                const float boxSize = 0.1f;

                var shipMatrix = Matrix4x4.CreateScale(boxSize) * Matrix4x4.CreateTranslation(_simulation.ShipPos);

                batch.ColoredQuadBox(shipMatrix, new QuadColors(1f, 0f, 0f, 0.5f));

                foreach (var history in _simulation.History)
                {
                    shipMatrix.Translation = history;
                    batch.ColoredQuadBox(shipMatrix, new QuadColors(1f, 0f, 0f, 0.1f));
                }

                batch.ColoredQuadBoxFrame(_simulation.TargetPos - Vector3d.One * 0.5 * boxSize, _simulation.TargetPos + Vector3d.One * 0.5 * boxSize, 0.01f, new QuadColors(0f, 1f, 0f, 0.5f));
            }
        });
    }
}