using System.Numerics;
using Common;
using GameFramework;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Renderer;
using GameFramework.Renderer.Batch;
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
    private int _seed = 0;

    public WorldLayer(GameApplication app, ImGuiLayer imGui) : base(app, imGui)
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
                    _simulation = new Simulation(
                        new Vector2ds(_sizeX, _sizeY), 
                        _visionRadius, 
                        _robotCount, 
                        _noiseScale, 
                        _noiseThreshold,
                        _seed
                    );
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

        for (var y = 0; y < _simulation.Size.Y; y++)
        {
            for (var x = 0; x < _simulation.Size.X; x++)
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
    }

    private void RenderRobotVisions(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }
    }

    private void RenderRobots(QuadBatch batch)
    {
        if (_simulation == null)
        {
            return;
        }
    }

    protected override void RenderStack()
    {
        RenderPass(RenderOverviewTerrain);
        RenderPass(RenderVisitedTerrain);
        RenderPass(RenderRobotVisions);
        RenderPass(RenderRobots);
    }
}

internal sealed class Simulation
{
    public Vector2ds Size { get; }

    public Grid2d<bool> Map { get; }

    public Simulation(Vector2ds size, int visionRadius, int robotCount, float noiseScale, float noiseThreshold, int seed)
    {
        Size = size;
        Map = new Grid2d<bool>(size);
        
        RandomizeMap(Map, noiseScale, noiseThreshold, seed);
    }

    private static void RandomizeMap(Grid2d<bool> grid, float scale, float threshold, int seed)
    {
        var noise = new Noise
        {
            Seed = seed
        };

        var noiseGrid = noise.Calc2D(grid.Size.X, grid.Size.Y, scale);

        for (var y = 0; y < grid.Size.Y; y++)
        {
            for (var x = 0; x < grid.Size.X; x++)
            {
                grid[x, y] = noiseGrid[x, y] > threshold;
            }
        }
    }
}