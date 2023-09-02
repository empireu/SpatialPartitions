using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using Common;
using GameFramework.Extensions;
using GameFramework.Renderer;
using GameFramework.Renderer.Batch;
using GameFramework.Utilities;
using GameFramework.Utilities.Extensions;
using Simplex;

namespace ExplorationVisualization;

internal sealed class TestRobot : Robot<TestRobot>
{
    private LinkedList<Vector2ds>? _lastPath;

    public TestRobot(RobotCreateInfo ci) : base(ci)
    {

    }

    private static void CommunicateFindings(Dictionary<Vector2ds, bool> view, TestRobot[] peers)
    {
        foreach (var robot in peers)
        {
            foreach (var (position, status) in view)
            {
                robot.OccupancyGrid[position] = status ? Occupancy.Obstacle : Occupancy.Open;
            }
        }        
    }

    private bool TryFollowPath()
    {
        if (_lastPath?.Last == null || _lastPath.First == null || Position == _lastPath.Last.Value)
        {
            goto skip;
        }

        if (!IsFrontierCell(_lastPath.Last.Value))
        {
            goto skip;
        }

        Debug.Assert(_lastPath.First.Value == Position);

        _lastPath.RemoveFirst();

        // Check if path is still possible:

        foreach (var position in _lastPath)
        {
            if (!IsWalkable(position))
            {
                goto skip;
            }
        }

        ApplyMove(Position.Base8DirectionTo(_lastPath.First.Value));

        return true;

        skip:
        _lastPath = null;
        return false;
    }

    private bool TryCreatePath()
    {
        foreach (var frontier in FrontierRegions.Keys.OrderBy(x => Vector2ds.DistanceSqr(x, Position)))
        {
            var path = FindPath(Position, frontier);

            if (path == null)
            {
                continue;
            }

            Debug.Assert(path.First() == Position);

            _lastPath = new LinkedList<Vector2ds>(path);

            return true;
        }

        return false;
    }

    protected override void UpdateAlgorithm(Dictionary<Vector2ds, bool> view, TestRobot[] peers)
    {
        CommunicateFindings(view, peers);

        if (TryFollowPath())
        {
            return;
        }

        if (TryCreatePath() && TryFollowPath())
        {
            return;
        }

        // Cannot find paths
        SetFinished();
    }

    public override void DebugDraw(QuadBatch batch)
    {
        if (_lastPath != null)
        {
            Vector2ds? a = null;

            foreach (var b in _lastPath)
            {
                if (a != null)
                {
                    batch.Line(a.Value, b, Color, 0.1f);
                }

                a = b;
            }
        }
    }
}

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

    void DebugDraw(QuadBatch batch);
}

internal interface IRobot : IRobotView
{
    void Update(Dictionary<Vector2ds, bool> view, IEnumerable<IRobot> peers);
}

internal abstract class Robot
{
    public delegate IRobot FactoryDelegate(RobotCreateInfo ci);

    public static readonly IReadOnlyDictionary<string, FactoryDelegate> Implementations = new Dictionary<string, FactoryDelegate>
    {
        { "Test", ci => new TestRobot(ci) }
    };

    public static readonly string[] ImplementationNames = Implementations.Keys.ToArray();

    protected static readonly Vector2ds[] NeighborOffsets = Enum.GetValues<Base8Direction2d>().Select(x => x.Step()).ToArray();
}

internal abstract class Robot<TSelf> : Robot, IRobot, IRobotPeer<TSelf> where TSelf : Robot<TSelf>
{
    protected readonly Grid2d<Occupancy> OccupancyGrid;
    private readonly HashSet<Vector2ds> _frontierEdges = new();
    private readonly HashMultiMap<Vector2ds, Vector2ds> _frontierRegions = new();
    private bool _moveApplied;

    public Robot(RobotCreateInfo ci)
    {
        OccupancyGrid = new Grid2d<Occupancy>(ci.WorldSize);
        Position = ci.Position;
        Color = ci.Color;
    }

    protected Vector2ds WorldSize => OccupancyGrid.Size;
    public IReadOnlyGrid2d<Occupancy> OccupancyGridView => OccupancyGrid;
    public IReadOnlySet<Vector2ds> FrontierEdges => _frontierEdges;
    public IReadOnlyMultiMap<Vector2ds, Vector2ds> FrontierRegions => _frontierRegions;
    public Vector2ds Position { get; private set; }
    public bool IsFinished { get; protected set; }
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

    public void Update(Dictionary<Vector2ds, bool> view, IEnumerable<IRobot> peers)
    {
        foreach (var (position, isWall) in view)
        {
            OccupancyGrid[position] = isWall ? Occupancy.Obstacle : Occupancy.Open;
        }

        OccupancyGrid[Position] = Occupancy.Open;

        UpdateExplorationStatus();
        UpdateFrontiers();

        if (_frontierEdges.Count == 0)
        {
            IsFinished = true;
            return;
        }

        UpdateAlgorithm(view, peers.Cast<TSelf>().ToArray());

        _moveApplied = false;
    }

    protected abstract void UpdateAlgorithm(Dictionary<Vector2ds, bool> view, TSelf[] peers);

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

    public abstract void DebugDraw(QuadBatch batch);
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
}

internal sealed class Simulation
{
    private readonly Grid2d<bool> _map;
    private readonly Grid2d<int> _exploredHistogram;
    private readonly IRobot[] _robots;

    private readonly HashSet<Vector2ds> _visionOffsets;

    private Simulation(Grid2d<bool> map, IRobot[] robots, int visionRadius)
    {
        _map = map;
        _exploredHistogram = new Grid2d<int>(map.Size);
        _robots = robots;
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

        IsFinished = true;

        for (var index = 0; index < _robots.Length; index++)
        {
            var robot = _robots[index];

            if (robot.IsFinished)
            {
                continue;
            }

            IsFinished = false;

            var view = new Dictionary<Vector2ds, bool>();

            foreach (var tile in TilesInView(robot))
            {
                _exploredHistogram[tile]++;
                ExploredMax = Math.Max(ExploredMax, _exploredHistogram[tile]);
                view.Add(tile, _map[tile]);
            }

            robot.Update(view, _robots.Where(x => x != robot));
        }

        if (!IsFinished)
        {
            ++Ticks;
        }
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
                )
            });
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