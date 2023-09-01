using System.Numerics;
using Common;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer;
using GameFramework.Renderer.Batch;
using ImGuiNET;
using Veldrid;

namespace ExplorationVisualization;

internal sealed class WorldLayer : VisualizationLayer2d
{
    protected override float MinZoom => 0.1f;

    private Simulation? _simulation;

    private int _sizeX = 128;
    private int _sizeY = 128;
    private int _visionRadius = 8;
    private int _robotCount = 1;
    private bool _changed = true;
    private float _noiseScale = 0.050f;
    private float _noiseThreshold = 200f;
    private int _seed = Random.Shared.Next(int.MinValue, int.MaxValue);
    private bool _createFailed;
    private string _implementationName = string.Empty;

    public WorldLayer(VisualizationApp app, ImGuiLayer imGui) : base(app, imGui)
    {

    }

    protected override void ImGuiOnSubmit(ImGuiRenderer sender)
    {
        if (ImGui.Begin("Simulation"))
        {
            _changed |= ImGui.SliderInt("Width", ref _sizeX, 16, 128);
            _changed |= ImGui.SliderInt("Height", ref _sizeY, 16, 128);
            _changed |= ImGui.SliderInt("Radius", ref _visionRadius, 4, 32);
            _changed |= ImGui.SliderInt("Robot Count", ref _robotCount, 1, 100);
            _changed |= ImGui.InputFloat("Noise Scale", ref _noiseScale);
            _changed |= ImGui.InputFloat("Noise Threshold", ref _noiseThreshold);
            _changed |= ImGuix.StringComboBox(Robot.ImplementationNames, ref _implementationName, "Algorithm");

            ImGui.Text($"Seed: {_seed}");
            ImGui.SameLine();
            if (ImGui.Button("New"))
            {
                _seed = Random.Shared.Next(int.MinValue, int.MaxValue);
                _changed = true;
            }

            if (_changed)
            {
                if (ImGui.Button("Create"))
                {
                    _changed = false;
                    _createFailed = !Simulation.TryCreateSimulation(
                        new Vector2ds(_sizeX, _sizeY), 
                        _noiseScale,
                        _noiseThreshold, 
                        _seed, 
                        _robotCount, 
                        _visionRadius,
                        Robot.Implementations[_implementationName],
                        out _simulation // Guaranteed to be null if failed
                    );

                    if (!_createFailed)
                    {
                        CameraController.FuturePosition2 = new Vector2(_sizeX, _sizeY) / 2f;
                        CameraController.FutureZoom = MathF.Sqrt(_sizeX * _sizeY);
                    }
                }
            }

            if (_createFailed)
            {
                ImGui.Text("Failed to create");
            }

            if (_simulation != null)
            {
                ImGui.Separator();

                if (ImGui.Button("Update") || ImGui.IsKeyReleased(ImGuiKey.Space))
                {
                    _simulation.Update();
                }
            }
        }
    }

    private void RenderOverviewTerrain(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }

        for (var y = 0; y < _simulation.Map.Size.Y; y++)
        {
            for (var x = 0; x < _simulation.Map.Size.X; x++)
            {
                batch.Quad(
                    new Vector2(x, y), 
                    _simulation.Map[x, y] 
                        ? new RgbaFloat4(0.8f, 0.5f, 0.5f, 1f) 
                        : new RgbaFloat4(0.1f, 0.1f, 0.2f, 1f)
                );
            }
        }
    }

    private void RenderVisitedTerrain(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }

        for (var y = 0; y < _simulation.Map.Size.Y; y++)
        {
            for (var x = 0; x < _simulation.Map.Size.X; x++)
            {
                var exploreCount = _simulation.Explored[x, y];

                if (exploreCount > 0)
                {
                    batch.Quad(new Vector2(x, y), new RgbaFloat4(0, 1, 0, exploreCount / (float)_simulation.ExploredMax * 0.5f));
                }
            }
        }
    }

    private void RenderRobotVisions(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }

        var size = new Vector2(0.8f);

        foreach (var robot in _simulation.Robots)
        {
            foreach (var tile in _simulation.TilesInView(robot))
            {
                batch.Quad(tile, size, robot.Color with { W = 0.25f });
            }
        }
    }

    private void RenderRobots(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }

        foreach (var robot in _simulation.Robots)
        {
            batch.Quad(robot.Position, robot.Color);
        }
    }

    private void RenderTooltips(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }

        var position = MouseI;

        if (!_simulation.Map.IsWithinBounds(MouseI))
        {
            return;
        }

        var times = _simulation.Explored[position];

        if (times > 0)
        {
            App.Font.Render(batch, Mouse + new Vector2(CameraController.Camera.Zoom * 0.025f, 0f),
                $"""
                 Seen {times} times
                 """, size: CameraController.Camera.Zoom * 0.05f);
        }
    }

    protected override void RenderStack()
    {
        RenderPass(RenderOverviewTerrain);
        RenderPass(RenderVisitedTerrain);
        RenderPass(RenderRobotVisions);
        RenderPass(RenderRobots);
        RenderPass(RenderTooltips);
    }
}