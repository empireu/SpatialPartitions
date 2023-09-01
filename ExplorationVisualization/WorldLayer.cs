using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using Common;
using GameFramework;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer;
using GameFramework.Renderer.Batch;
using GameFramework.Utilities;
using GameFramework.Utilities.Extensions;
using ImGuiNET;
using Simplex;
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

internal interface IRobotPeer
{
    Vector2ds Position { get; }
    Vector4 Color { get; }
}

internal sealed class Robot : IRobotPeer
{
    private readonly Vector2ds _worldSize;

    public Vector2ds Position { get; private set; }

    public Vector4 Color { get; }

    public Robot(Vector2ds worldSize, Vector2ds position, Vector4 color)
    {
        _worldSize = worldSize;
        Position = position;
        Color = color;
    }

    public void Update(Dictionary<Vector2ds, bool> view, IRobotPeer[] peers)
    {

    }
}

internal sealed class Simulation
{
    private readonly Grid2d<bool> _map;
    private readonly Grid2d<int> _explored;
    private readonly Robot[] _robots;
    private readonly Vector2ds[] _previousPositions;

    private readonly HashSet<Vector2ds> _visionOffsets;

    private Simulation(Grid2d<bool> map, Robot[] robots, int visionRadius)
    {
        _map = map;
        _explored = new Grid2d<int>(map.Size);
        _robots = robots;
        _previousPositions = robots.Select(x => x.Position).ToArray();
        _visionOffsets = new HashSet<Vector2ds>();

        for (var x = -visionRadius; x <= visionRadius; x++)
        {
            for (var y = -visionRadius; y <= visionRadius; y++)
            {
                if (x == 0 && y == 0)
                {
                    continue;
                }

                var v = new Vector2ds(x, y);

                if (v.Norm <= visionRadius)
                {
                    _visionOffsets.Add(v);
                }
            }
        }
    }

    public IReadOnlyGrid2d<bool> Map => _map;
    public IReadOnlyGrid2d<int> Explored => _explored;
    public int ExploredMax { get; private set; }
    public IReadOnlyList<IRobotPeer> Robots => _robots;

    public IEnumerable<Vector2ds> TilesInView(IRobotPeer robot) => 
        _visionOffsets
        .Select(offset => robot.Position + offset)
        .Where(tile => _map.IsWithinBounds(tile));

    public void Update()
    {
        for (var index = 0; index < _robots.Length; index++)
        {
            var robot = _robots[index];
            var view = new Dictionary<Vector2ds, bool>();

            foreach (var tile in TilesInView(robot))
            {
                _explored[tile]++;
                ExploredMax = Math.Max(ExploredMax, _explored[tile]);
                view.Add(tile, _map[tile]);
            }

            robot.Update(view, _robots.Where(x => x != robot).Cast<IRobotPeer>().ToArray());

            Assert.IsTrue(Vector2ds.Distance(robot.Position, _previousPositions[index]) < 1.5);
        
            _previousPositions[index] = robot.Position;
        }
    }

    public static bool TryCreateSimulation(
        Vector2ds size, 
        float noiseScale, 
        float noiseThreshold, 
        int seed,
        int robotCount, 
        int visionRadius,
        [NotNullWhen(true)] out Simulation? result)
    {
        if (!TryCreateMap(size, noiseScale, noiseThreshold, seed, robotCount, out var map, out var robotPositions))
        {
            result = null;
            return false;
        }

        var robots = new Robot[robotCount];

        var random = new Random(seed);

        for (var i = 0; i < robotCount; i++)
        {
            robots[i] = new Robot(
                size, 
                robotPositions[i], 
                random.NextVector4(min: 0.25f, max: 1f) with { W = 1 }
            );
        }

        result = new Simulation(map, robots, visionRadius);

        return true;
    }

    private static bool TryCreateMap(
        Vector2ds size, 
        float scale, 
        float threshold, 
        int seed, 
        int robotCount,
        [NotNullWhen(true)] out Grid2d<bool>? map,
        [NotNullWhen(true)] out Vector2ds[]? robotPositions)
    {
        map = null;
        robotPositions = null;

        var grid = new Grid2d<bool>(size);

        var noise = new Noise
        {
            Seed = seed
        };

        var noiseGrid = noise.Calc2D(size.X, size.Y, scale);
        var emptySlots = new List<Vector2ds>();

        for (var y = 0; y < size.Y; y++)
        {
            for (var x = 0; x < size.X; x++)
            {
                var isWall = noiseGrid[x, y] > threshold;
                
                grid[x, y] = isWall;

                if (!isWall)
                {
                    emptySlots.Add(new Vector2ds(x, y));
                }
            }
        }

        if (emptySlots.Count < robotCount)
        {
            return false;
        }

        map = grid;

        robotPositions = new Vector2ds[robotCount];
        var random = new Random(seed);

        for (var i = 0; i < robotCount; i++)
        {
            var position = emptySlots[random.Next(0, emptySlots.Count - 1)];
            emptySlots.Remove(position);
            robotPositions[i] = position;
        }

        return true;
    }
}