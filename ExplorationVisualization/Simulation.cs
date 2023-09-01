using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using Common;
using GameFramework.Utilities;
using GameFramework.Utilities.Extensions;
using Simplex;

namespace ExplorationVisualization;

internal sealed class TestRobot : Robot
{
    public TestRobot(RobotCreateInfo ci) : base(ci)
    {

    }

    protected override void UpdateAlgorithm(Dictionary<Vector2ds, bool> view, IRobotPeer[] peers)
    {
        ApplyMove(Enum.GetValues<Direction2d>()[Random.Shared.Next(0, 4)]);
    }
}

internal abstract class Robot : IRobotPeer
{
    protected readonly Vector2ds WorldSize;
    protected readonly Grid2d<bool?> Seen;

    public Vector2ds Position { get; private set; }

    public Vector4 Color { get; }

    private bool _moveApplied;

    public Robot(RobotCreateInfo ci)
    {
        WorldSize = ci.WorldSize;
        Seen = new Grid2d<bool?>(ci.WorldSize);
        Position = ci.Position;
        Color = ci.Color;
    }

    protected bool IsWithinBounds(Vector2ds position) => position.X >= 0 && Position.Y >= 0 &&
                                                         Position.X < WorldSize.X && Position.Y < WorldSize.Y;

    protected bool IsWall(Vector2ds position) => !IsWithinBounds(position) || Seen[position] == true;

    protected bool IsKnown(Vector2ds position) => !IsWithinBounds(position) || Seen[position].HasValue;

    protected void ApplyMove(Vector2ds newPosition)
    {
        Assert.IsTrue(newPosition != Position);
        Assert.IsTrue(Vector2ds.Distance(newPosition, Position).ApproxEq(1));
        ApplyMove(Position.DirectionTo(newPosition));
    }

    protected void ApplyMove(Direction2d direction)
    {
        Assert.IsTrue(!_moveApplied, "Apply multiple moves");
        Position += direction;
        Assert.IsTrue(IsWithinBounds(Position), "Went out of bounds");
        Assert.IsTrue(!IsWall(Position), "Collided with wall");
    }

    public void Update(Dictionary<Vector2ds, bool> view, IRobotPeer[] peers)
    {
        foreach (var (position, isWall) in view)
        {
            Seen[position] = isWall;
        }

        UpdateAlgorithm(view, peers);

        _moveApplied = false;
    }

    protected abstract void UpdateAlgorithm(Dictionary<Vector2ds, bool> view, IRobotPeer[] peers);

    public delegate Robot FactoryDelegate(RobotCreateInfo ci);

    public static readonly IReadOnlyDictionary<string, FactoryDelegate> Implementations =
        new Dictionary<string, FactoryDelegate>
        {
            { "Test", ci => new TestRobot(ci) }
        };

    public static readonly string[] ImplementationNames = Implementations.Keys.ToArray();
}

internal readonly struct RobotCreateInfo
{
    public required Vector2ds WorldSize { get; init; }
    public required Vector2ds Position { get; init; }
    public required Vector4 Color { get; init; }
}

internal interface IRobotPeer
{
    Vector2ds Position { get; }
    Vector4 Color { get; }
}

internal sealed class Simulation
{
    private readonly Grid2d<bool> _map;
    private readonly Grid2d<int> _explored;
    private readonly Robot[] _robots;

    private readonly HashSet<Vector2ds> _visionOffsets;

    private Simulation(Grid2d<bool> map, Robot[] robots, int visionRadius)
    {
        _map = map;
        _explored = new Grid2d<int>(map.Size);
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

        var robots = new Robot[robotCount];

        var random = new Random(seed);

        for (var i = 0; i < robotCount; i++)
        {
            robots[i] = factory(new RobotCreateInfo
            {
                WorldSize = size,
                Position = robotPositions[i],
                Color = random.NextVector4(min: 0.25f, max: 1f) with { W = 1 }
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