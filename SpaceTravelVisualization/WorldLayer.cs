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
        public double Time { get; private set; }
        private readonly double _shipForce;
        protected double _shipMass;
        private readonly Vector3d _targetAccel;

        private TrajectoryLeg[]? _trajectory;
        public IReadOnlyList<TrajectoryLeg>? Trajectory => _trajectory;

        protected readonly List<Vector3d> _history = new();

        public IReadOnlyList<Vector3d> History => _history;

        public Vector3d ShipPos { get; protected set; }
        public Vector3d ShipVel { get; protected set; }
        public Vector3d TargetPos { get; protected set; }
        public Vector3d TargetVel { get; protected set; }

        public Simulation(
            Vector3d shipPos, 
            Vector3d shipVel, 
            double shipForce,
            double shipMass, 
            Vector3d targetPos, 
            Vector3d targetVel, 
            Vector3d targetAccel)
        {
            _shipForce = shipForce;
            _shipMass = shipMass;
            _targetAccel = targetAccel;
            ShipPos = shipPos;
            ShipVel = shipVel;
            TargetPos = targetPos;
            TargetVel = targetVel;
        }

        public void Update(double dt)
        {
            var parameters = new TrajectoryParameters(
                ShipPos, ShipVel, TargetPos, TargetVel,
                _shipForce, _shipMass
            );

            var trajectory = TrajectoryGenerator.ComputeRendezvousTrajectory(parameters);

            _trajectory = trajectory;

            if (trajectory == null)
            {
                return;
            }

            TrajectoryLeg leg;

            var Time = 0.0;

            if (trajectory.Length == 1)
            {
                if (Time < trajectory[0].Duration)
                {
                    leg = trajectory[0];
                }
                else
                {
                    return;
                }
            }
            else
            {
                if (Time < trajectory[0].Duration)
                {
                    leg = trajectory[0];
                }
                else if (Time < trajectory[0].Duration + trajectory[1].Duration)
                {
                    leg = trajectory[1];
                }
                else
                {
                    return;
                }
            }

            var acceleration = leg.Force / _shipMass;

            ShipVel += acceleration * dt;
            ShipPos += ShipVel * dt;
            TargetVel += _targetAccel * dt;
            TargetPos += TargetVel * dt;

            Time += dt;

            if (_history.Count == 0)
            {
                _history.Add(ShipPos);
            }
            else if (Vector3d.DistanceSqr(_history.Last(), ShipPos) > 0.25)
            {
                _history.Add(ShipPos);
            }
        }
    }

    private Vector3 _shipPos;
    private Vector3 _shipVel;
    private Vector3 _targetPos = Vector3.One;
    private Vector3 _targetVel;
    private Vector3 _targetAccel;
    private double _force = 1f;
    private double _mass = 1;
    private float _speed = 1;

    private Simulation? _simulation;

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
            ImGui.InputFloat3("Target Accel", ref _targetAccel);

            if (ImGui.Button("Start"))
            {
                _simulation = new Simulation(_shipPos, _shipVel, _force, _mass, _targetPos, _targetVel, _targetAccel);
            }

            ImGui.Separator();

            ImGui.SliderFloat("Speed", ref _speed, 0, 1);

            if (_simulation != null)
            {
                if (_simulation.Trajectory == null)
                {
                    ImGui.Text("Trajectory: nil");
                }
                else
                {
                    ImGui.Text($"Trajectory: {_simulation.Trajectory.Count} segments, " +
                               $"{(_simulation.Trajectory.Count == 1 
                                   ? ($"a: {_simulation.Trajectory[0].Duration:F3}") 
                                   : ($"a: {_simulation.Trajectory[0].Duration:F3}, b: {_simulation.Trajectory[1].Duration:F3}"))}");

                }
                ImGui.Text($"T+{_simulation.Time:F4}");
                ImGui.Text($"Ship Velocity [X={_simulation.ShipVel.X:F2} Y={_simulation.ShipVel.Y:F2} Z={_simulation.ShipVel.Z:F2}]");
                ImGui.Text($"Distance: {Vector3d.Distance(_simulation.ShipPos, _simulation.TargetPos):F2}");
            }
        }

        ImGui.End();
    }

    protected override void Update(FrameInfo frameInfo)
    {
        base.Update(frameInfo);

        const int subSteps = 10;
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