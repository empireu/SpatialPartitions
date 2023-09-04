using System.Diagnostics.CodeAnalysis;
using System.Text;
using Common;
using GameFramework.Renderer;
using GameFramework.Renderer.Batch;
using GameFramework.Utilities;
using GameFramework.Utilities.Extensions;
using ImGuiNET;
using Simplex;

namespace ExplorationVisualization;

internal interface IRobotPeer<TSelf> where TSelf : IRobotPeer<TSelf>
{
    Vector2ds Position { get; }
    RgbaFloat4 Color { get; }
    bool IsFinished { get; }
}

internal interface IRobotView
{
    Vector2ds Position { get; }
    RgbaFloat4 Color { get; }
    double ExploreProgress { get; }
    IReadOnlyGrid2d<Occupancy> OccupancyGridView { get; }
    IReadOnlySet<Vector2ds> FrontierEdges { get; }
    IReadOnlyMultiMap<Vector2ds, Vector2ds> FrontierRegions { get; }
    bool IsFinished { get; }

    void DebugDraw(QuadBatch batch, Vector2ds mouse);
    void AddTooltips(StringBuilder sb, Vector2ds mouse);
}

internal interface IRobot : IRobotView
{
    void BeforeStart();
    void PrepareUpdate(IReadOnlyDictionary<Vector2ds, bool> view, IEnumerable<IRobot> peers);
    void Update();
    void AfterUpdate();
}

internal interface IRobotGui
{
    void Submit();
}

internal abstract class Robot
{
    public delegate IRobot FactoryDelegate(RobotCreateInfo ci);

    private delegate IRobot FactoryDelegateWithGui(RobotCreateInfo ci, IRobotGui? gui);

    public readonly struct RobotOptions
    {
        public RobotOptions(FactoryDelegate factory, IRobotGui? gui = null)
        {
            Factory = factory;
            Gui = gui;
        }

        public FactoryDelegate Factory { get; }
        public IRobotGui? Gui { get; }
    }


    private static void Register(string name, IRobotGui? gui, FactoryDelegateWithGui factory)
    {
        var wrapper = new FactoryDelegate(ci => factory(ci, gui));
        _implementations.Add(name, new RobotOptions(wrapper, gui));
    }

    private static readonly Dictionary<string, RobotOptions> _implementations;

    static Robot()
    {
        _implementations = new Dictionary<string, RobotOptions>();

        Register(
            "Cooperative Robot", 
            new SmartRobotGui(), 
            (ci, gui) => new SmartRobot(ci, Assert.Is<SmartRobotGui>(gui))
        );

        Register("Implicit Robot", null, (ci, gui) => new TestRobot(ci));

        ImplementationNames =  Implementations.Keys.ToArray();
    }

    public static IReadOnlyDictionary<string, RobotOptions> Implementations => _implementations;

    public static readonly string[] ImplementationNames;

    protected static readonly Vector2ds[] NeighborOffsets = Enum.GetValues<Base8Direction2d>().Select(x => x.Step()).ToArray();
}

internal abstract class Robot<TSelf> : Robot, IRobot, IRobotPeer<TSelf> where TSelf : Robot<TSelf>
{
    protected readonly Grid2d<Occupancy> OccupancyGrid;
    protected readonly HashSet<Vector2ds> _frontierEdges = new();
    protected readonly HashMultiMap<Vector2ds, Vector2ds> _frontierRegions = new();
    private bool _moveApplied;

    protected readonly Vector2ds[] VisionOffsets;

    public Robot(RobotCreateInfo ci)
    {
        OccupancyGrid = new Grid2d<Occupancy>(ci.WorldSize);
        Position = ci.Position;
        Color = ci.Color;
        VisionOffsets = ci.ViewOffsets;
    }

    protected Vector2ds WorldSize => OccupancyGrid.Size;
    public IReadOnlyGrid2d<Occupancy> OccupancyGridView => OccupancyGrid;
    public IReadOnlySet<Vector2ds> FrontierEdges => _frontierEdges;
    public IReadOnlyMultiMap<Vector2ds, Vector2ds> FrontierRegions => _frontierRegions;
    public Vector2ds Position { get; private set; }
    public bool IsFinished { get; private set; }
    public RgbaFloat4 Color { get; }
    public double ExploreProgress { get; private set; }

    protected bool IsWithinBounds(Vector2ds position) => 
        position.X >= 0 && position.X < WorldSize.X &&
        position.Y >= 0 && position.Y < WorldSize.Y;

    protected bool IsWall(Vector2ds position) => !IsWithinBounds(position) || OccupancyGrid[position] == Occupancy.Obstacle;
  
    protected bool IsWalkable(Vector2ds position) => IsWithinBounds(position) && OccupancyGrid[position] != Occupancy.Obstacle;

    protected void ApplyMove(Base8Direction2d direction)
    {
        Assert.IsTrue(!_moveApplied, "Apply multiple moves");
        Position += direction;
        Assert.IsTrue(IsWithinBounds(Position), "Went out of bounds");
        Assert.IsTrue(!IsWall(Position), "Collided with wall");
    }

    public void SetFinished()
    {
        IsFinished = true;
    }

    protected bool IsFrontierCell(Vector2ds position)
    {
        var neighborOffsets = NeighborOffsets;

        for (var i = 0; i < neighborOffsets.Length; i++)
        {
            var neighbor = position + neighborOffsets[i];

            if (IsWithinBounds(neighbor) && OccupancyGrid[neighbor] == Occupancy.Unexplored)
            {
                return true;
            }
        }

        return false;
    }

    private void UpdateFrontiers()
    {
        _frontierEdges.Clear();
        _frontierRegions.Clear();

        var size = OccupancyGrid.Size;
        var neighborOffsets = NeighborOffsets;

        // Finds frontier edges:
        for (var y = 0; y < size.Y; y++)
        {
            for (var x = 0; x < size.X; x++)
            {
                var position = new Vector2ds(x, y);

                if (OccupancyGrid[position] != Occupancy.Open)
                {
                    continue;
                }

                for (var i = 0; i < neighborOffsets.Length; i++)
                {
                    var neighbor = position + neighborOffsets[i];

                    if (IsWithinBounds(neighbor) && OccupancyGrid[neighbor] == Occupancy.Unexplored)
                    {
                        _frontierEdges.Add(position);
                        break;
                    }
                }
            }
        }

        // Finds frontier regions:
        var pendingFrontierCells = new HashSet<Vector2ds>(_frontierEdges);
        var queue = new Queue<Vector2ds>();
        
        while (pendingFrontierCells.Count > 0)
        {
            var group = new HashSet<Vector2ds>();

            queue.Enqueue(pendingFrontierCells.First());

            var centerAverage = new Average2d();
           
            while (queue.Count > 0)
            {
                var front = queue.Dequeue();

                if (!group.Add(front))
                {
                    continue;
                }

                Assert.IsTrue(pendingFrontierCells.Remove(front));

                centerAverage.Add(front);

                for (var i = 0; i < neighborOffsets.Length; i++)
                {
                    var neighbor = front + neighborOffsets[i];

                    if (pendingFrontierCells.Contains(neighbor))
                    {
                        queue.Enqueue(neighbor);
                    }
                }
            }
            
            queue.Clear();

            var center = centerAverage.Value.Round();

            Assert.IsTrue(_frontierRegions.Place(group.MinBy(x => Vector2ds.DistanceSqr(x, center)), group) == null);
        }

        var invalidFrontiers = _frontierRegions.Keys.Where(frontier => FindPath(Position, frontier) == null);

        foreach (var frontier in invalidFrontiers)
        {
            Assert.IsTrue(_frontierRegions.Remove(frontier, out var edges));

            foreach (var edge in edges)
            {
                Assert.IsTrue(_frontierEdges.Remove(edge));
            }
        }
    }

    private void UpdateExplorationStatus()
    {
        var exploredCells = 0;

        var size = OccupancyGrid.Size;
        
        for (var y = 0; y < size.Y; y++)
        {
            for (var x = 0; x < size.X; x++)
            {
                if (OccupancyGrid[x, y] != Occupancy.Unexplored)
                {
                    ++exploredCells;
                }
            }
        }

        ExploreProgress = exploredCells / (double)(size.X * size.Y);
    }

    public virtual void BeforeStart()
    {
        
    }

    private TSelf[]? _peers;
    private IReadOnlyDictionary<Vector2ds, bool>? _view;

    protected IReadOnlyList<TSelf> Peers => Assert.NotNull(_peers);
    protected IReadOnlyDictionary<Vector2ds, bool> View => Assert.NotNull(_view);

    protected virtual void LoadFoundTile(Vector2ds position, bool value)
    {
        OccupancyGrid[position] = value ? Occupancy.Obstacle : Occupancy.Open;
    }

    public void PrepareUpdate(IReadOnlyDictionary<Vector2ds, bool> view, IEnumerable<IRobot> peers)
    {
        _view = view;
        _peers = peers.Cast<TSelf>().ToArray();
        PrepareUpdateCore();
    }

    public void Update()
    {
        foreach (var (position, isWall) in View)
        {
            LoadFoundTile(position, isWall);
        }

        LoadFoundTile(Position, false);

        UpdateExplorationStatus();
        UpdateFrontiers();

        if (_frontierEdges.Count == 0)
        {
            Assert.IsTrue(_frontierRegions.Count == 0);
            IsFinished = true;
            return;
        }

        Assert.IsTrue(_frontierRegions.Count > 0);

        UpdateCore();
    }

    public void AfterUpdate()
    {
        AfterUpdateCore();
        _moveApplied = false;
    }

    protected virtual void PrepareUpdateCore() { }
    protected virtual void UpdateCore() { }
    protected virtual void AfterUpdateCore() { }

    protected List<Vector2ds>? FindPath(Vector2ds source, Vector2ds destination)
    {
        if (!IsWithinBounds(source) || !IsWithinBounds(destination))
        {
            return null;
        }

        if (OccupancyGrid[source] == Occupancy.Obstacle || OccupancyGrid[destination] == Occupancy.Obstacle)
        {
            return null;
        }

        if (source == destination)
        {
            return new List<Vector2ds> { source, destination };
        }

        var queue = new PriorityQueue<(Vector2ds? Ancestor, Vector2ds Position), int>();
        var visited = new HashSet<Vector2ds>();
        var ancestors = new Dictionary<Vector2ds, Vector2ds>();

        queue.Enqueue((null, source), 0);

        var offsets = NeighborOffsets;

        while (queue.Count > 0)
        {
            var (ancestor, front) = queue.Dequeue();

            if (!visited.Add(front))
            {
                continue;
            }

            if (ancestor.HasValue)
            {
                ancestors[front] = ancestor.Value;
            }

            if (front == destination)
            {
                var results = new List<Vector2ds>();

                while (true)
                {
                    results.Add(front);

                    if (!ancestors.TryGetValue(front, out front))
                    {
                        break;
                    }
                }

                results.Reverse();

                return results;
            }

            for (var i = 0; i < offsets.Length; i++)
            {
                var neighbor = front + offsets[i];

                if (IsWithinBounds(neighbor) && OccupancyGrid[neighbor] != Occupancy.Obstacle)
                {
                    queue.Enqueue((front, neighbor), Vector2ds.DistanceSqr(destination, neighbor));
                }
            }
        }

        return null;
    }

    public virtual void DebugDraw(QuadBatch batch, Vector2ds mouse)
    {

    }

    public virtual void AddTooltips(StringBuilder sb, Vector2ds mouse)
    {

    }
}

public enum Occupancy : byte
{
    Unexplored = 0,
    Open = 1,
    Obstacle = 2
}

internal readonly struct RobotCreateInfo
{
    public required Vector2ds WorldSize { get; init; }
    public required Vector2ds Position { get; init; }
    public required RgbaFloat4 Color { get; init; }
    public Vector2ds[] ViewOffsets { get; init; }
}

internal sealed class Simulation
{
    private readonly Grid2d<bool> _map;
    private readonly Grid2d<int> _exploredHistogram;
    private readonly IRobot[] _robots;
    private readonly Vector2ds[] _visionOffsets;

    private IEnumerable<IRobot> Unfinished => _robots.Where(x => !x.IsFinished);

    private Simulation(Grid2d<bool> map, IRobot[] robots, IEnumerable<Vector2ds> visionOffsets)
    {
        _map = map;
        _exploredHistogram = new Grid2d<int>(map.Size);
        _robots = robots;
        _visionOffsets = visionOffsets.ToArray();

        foreach (var robot in robots)
        {
            robot.BeforeStart();
        }
    }

    public IReadOnlyGrid2d<bool> Map => _map;
    public IReadOnlyGrid2d<int> ExploredHistogram => _exploredHistogram;
    public int ExploredMax { get; private set; }
    public IReadOnlyList<IRobotView> Robots => _robots;

    public bool IsFinished { get; private set; }
    public int Ticks { get; private set; }

    public IEnumerable<Vector2ds> TilesInView(IRobotView robot) =>
        _visionOffsets
            .Select(offset => robot.Position + offset)
            .Where(tile => _map.IsWithinBounds(tile));

    public void Update()
    {
        if (IsFinished)
        {
            return;
        }

        if (Ticks == 0)
        {
            foreach (var robot in _robots)
            {
                robot.BeforeStart();
            }
        }

        ++Ticks;

        foreach (var robot in Unfinished)
        {
            var view = new Dictionary<Vector2ds, bool>();

            foreach (var tile in TilesInView(robot))
            {
                _exploredHistogram[tile]++;
                ExploredMax = Math.Max(ExploredMax, _exploredHistogram[tile]);
                view.Add(tile, _map[tile]);
            }

            robot.PrepareUpdate(view, _robots.Where(x => x != robot));
        }

        foreach (var robot in Unfinished)
        {
            robot.Update();
        }

        foreach (var robot in Unfinished)
        {
            robot.AfterUpdate();
        }

        IsFinished = !Unfinished.Any();
    }

    public static bool TryCreateSimulation(
        Vector2ds size,
        float noiseScale,
        float noiseThreshold,
        int seed,
        int robotCount,
        int visionRadius,
        Robot.FactoryDelegate factory,
        [NotNullWhen(true)] out Simulation? result)
    {
        if (!TryCreateMap(size, noiseScale, noiseThreshold, seed, robotCount, out var map, out var robotPositions))
        {
            result = null;
            return false;
        }

        var robots = new IRobot[robotCount];

        var random = new Random(seed);

        var visionOffsets = new List<Vector2ds>();

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
                    visionOffsets.Add(v);
                }
            }
        }

        for (var i = 0; i < robotCount; i++)
        {
            robots[i] = factory(new RobotCreateInfo
            {
                WorldSize = size,
                Position = robotPositions[i],
                Color = new RgbaFloat4(
                    random.NextVector4(min: 0.25f, max: 1f) with { W = 1 }, 
                    random.NextVector4(min: 0.25f, max: 1f) with { W = 1 },
                    random.NextVector4(min: 0.25f, max: 1f) with { W = 1 },
                    random.NextVector4(min: 0.25f, max: 1f) with { W = 1 }
                ),
                ViewOffsets = visionOffsets.ToArray()
            });
        }

        result = new Simulation(map, robots, visionOffsets);

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