using System.Numerics;
using System.Text;
using Common;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer;
using GameFramework.Renderer.Batch;
using GameFramework.Utilities.Extensions;
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
    private int _selectedRobot;
    private bool _showViewRadius = true;
    private bool _showFrontiers = true;
    private bool _showVisitedTiles = true;

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
                _selectedRobot = 0;
            }

            if (ImGui.Button($"Create{(_changed ? "*" : "")}"))
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

            if (_createFailed)
            {
                ImGui.Text("Failed to create");
            }

            if (_simulation != null)
            {
                if (_simulation.IsFinished)
                {
                    ImGui.Text("Finished");
                    ImGui.Text($"{_simulation.Ticks} ticks");
                }
                else
                {
                    ImGui.Separator();

                    ImGui.SliderInt("Follow Robot", ref _selectedRobot, 0, _simulation.Robots.Count - 1);
                    ImGui.Checkbox("Show View", ref _showViewRadius);
                    ImGui.Checkbox("Show Frontiers", ref _showFrontiers);
                    ImGui.Checkbox("Show Visited", ref _showVisitedTiles);
                    ImGui.Separator();
                    ImGui.Text($"Tick {_simulation.Ticks}");
                    if (ImGui.Button("Update") || ImGui.IsKeyReleased(ImGuiKey.Space))
                    {
                        _simulation.Update();
                    }
                }
            }
        }
    }

    private void RenderTerrain(QuadBatch batch)
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
        if (_simulation == null || !_showVisitedTiles)
        {
            return;
        }

        for (var y = 0; y < _simulation.Map.Size.Y; y++)
        {
            for (var x = 0; x < _simulation.Map.Size.X; x++)
            {
                var exploreCount = _simulation.ExploredHistogram[x, y];

                if (exploreCount > 0)
                {
                    batch.Quad(new Vector2(x, y), new RgbaFloat4(0, 1, 0, exploreCount / (float)_simulation.ExploredMax * 0.5f));
                }
            }
        }
    }

    private void RenderRobotVisions(QuadBatch batch)
    {
        if (_simulation == null || !_showViewRadius)
        {
            return;
        }

        var size = new Vector2(0.8f);

        foreach (var robot in _simulation.Robots)
        {
            var color = robot.Color;

            color.A = 0.25f;

            foreach (var tile in _simulation.TilesInView(robot))
            {
                batch.Quad(tile, size, color);
            }
        }
    }

    private void RenderFrontierEdges(QuadBatch batch)
    {
        if (_simulation == null || !_showFrontiers)
        {
            return;
        }

        var robot = _simulation.Robots[_selectedRobot];
        var random = new Random(robot.GetHashCode());

        foreach (var center in robot.FrontierRegions.Keys)
        {
            var region = robot.FrontierRegions[center];
            var color = random.NextVector4(min: 0.25f, max: 1f) with { W = 0.5f };

            foreach (var frontier in region)
            {
                batch.Quad(frontier, color);
            }
        }
    }

    private void RenderFrontierCenters(QuadBatch batch)
    {
        if (_simulation == null || !_showFrontiers)
        {
            return;
        }

        var robot = _simulation.Robots[_selectedRobot];
        var random = new Random(robot.GetHashCode());

        var size = new Vector2(1.1f, 1.1f);
        const float rotation = MathF.PI / 4f;

        foreach (var center in robot.FrontierRegions.Keys)
        {
            var color = random.NextVector4(min: 0.25f, max: 1f) with { W = 0.7f };
            batch.Quad(center, size, rotation, color);
        }
    }

    private void RenderRobots(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }

        var size = new Vector2(0.5f);

        foreach (var robot in _simulation.Robots)
        {
            batch.Quad(robot.Position, size, robot.Color);
        }
    }

    private void RenderRobotDebugView(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }

        _simulation.Robots[_selectedRobot].DebugDraw(batch);
    }

    private void RenderTooltips(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }

        if (!_simulation.Map.IsWithinBounds(MouseI))
        {
            return;
        }

        var textSize = CameraController.Camera.Zoom * 0.025f;

        var globalTooltip = new StringBuilder();
        var times = _simulation.ExploredHistogram[MouseI];

        if (times > 0)
        {
            globalTooltip.Append($"Seen {times} times");
        }

        if (globalTooltip.Length > 0)
        {
            var str = globalTooltip.ToString();

            App.Font.Render(
                batch,
                Mouse - new Vector2(App.Font.MeasureText(str, textSize).X, 0),
                str,
                size: textSize,
                color: new Vector3(0.8f, 0 , 0)
            );
        }

        var robot = _simulation.Robots[_selectedRobot];
        var robotTooltip = new StringBuilder();

        robotTooltip.AppendLine($"Robot {_selectedRobot}");
        robotTooltip.AppendLine($"Evidence: {robot.OccupancyGridView[MouseI]}");
        robotTooltip.AppendLine($"Explored: {robot.ExploreProgress * 100:F2}%");

        if (robotTooltip.Length > 0)
        {
            App.Font.Render(
                batch,
                Mouse + new Vector2(CameraController.Camera.Zoom * 0.025f, 0f),
                robotTooltip.ToString(),
                size: textSize,
                color: new Vector3(0, 0.8f, 0f)
            );
        }
    }

    protected override void RenderStack()
    {
        RenderPass(RenderTerrain);

        RenderPass(RenderVisitedTerrain);
        RenderPass(RenderRobotVisions);
        
        RenderPass(RenderFrontierEdges);
        RenderPass(RenderFrontierCenters);
        
        RenderPass(RenderRobots);

        RenderPass(RenderRobotDebugView);
        
        RenderPass(RenderTooltips);
    }
}