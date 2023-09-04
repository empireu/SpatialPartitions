using System.Diagnostics;
using System.Text;
using Common;
using GameFramework.Extensions;
using GameFramework.Renderer.Batch;
using GameFramework.Utilities;
using ImGuiNET;

namespace ExplorationVisualization;

internal struct DecentralizedRobotConfig
{
    public required double KCost { get; init; }
    public required double KUtility { get; init; }
    public required double KOccupancy { get; init; }
    public required double OccupancyRadius { get; init; }
}

internal sealed class SmartRobotGui : IRobotGui
{
    private float _kCost = 2f;
    private float _kUtility = 1f;
    private float _kOccupancy = 20f;
    private float _occupancyRadius = 16;

    public DecentralizedRobotConfig Config => new()
    {
        KCost = _kCost,
        KUtility = _kUtility,
        KOccupancy = _kOccupancy,
        OccupancyRadius = _occupancyRadius
    };

    public void Submit()
    {
        ImGui.InputFloat("K Cost", ref _kCost);
        ImGui.InputFloat("K Utility", ref _kUtility);
        ImGui.InputFloat("K Occupancy", ref _kOccupancy);
        ImGui.InputFloat("Occupancy Radius", ref _occupancyRadius);
    }
}

internal sealed class SmartRobot : Robot<SmartRobot>
{
    private readonly DecentralizedRobotConfig _config;

    private LinkedList<Vector2ds>? _path;

    private readonly struct SharedInformation
    {
        public required Vector2ds Position { get; init; }
        public required Vector2ds Goal { get; init; }
    }

    private bool _initialized;
    private Vector2ds _goal;
    private Grid2d<double>? _weights;
    private double _weight;
    private Vector2ds[]? _frontiers;
    private int _originalFrontierCount;

    private SharedInformation GetInformation() => new()
    {
        Position = Position,
        Goal = _goal
    };

    public SmartRobot(RobotCreateInfo ci, SmartRobotGui gui) : base(ci)
    {
        _config = gui.Config;
    }

    // Cost metric for visiting a frontier.
    // This can factor in energy expenditure (fuel), time, etc.
    // Currently, the L2 distance is used as cost.
    private double ImplicitCost(Vector2ds frontierPoint, Vector2ds robotPosition)
    {
        return _config.KCost * Vector2ds.Distance(frontierPoint, robotPosition);
    }

    // Utility (gain) of reaching a frontier (how useful this frontier is, compared to others)
    // Currently, the number of cells discovered is used as utility.
    private double Utility(Vector2ds frontierPoint)
    {
        var kJ = VisionOffsets.Count(offset =>
        {
            var position = frontierPoint + offset;

            return OccupancyGrid.IsWithinBounds(position) && OccupancyGrid[position] == ExplorationVisualization.Occupancy.Unexplored;
        });

        return _config.KUtility * kJ;
    }

    // Cost metric for choosing a frontier, based on the goals chosen by another robot.
    // The cost will be higher if the other robot is targeting that frontier (and thus, the overall coverage decreases)
    public double Occupancy(Vector2ds frontierPoint, Vector2ds goal)
    {
        var radius = _config.OccupancyRadius;

        if (Vector2ds.DistanceSqr(frontierPoint, goal) > radius * radius)
        {
            return 0;
        }

        var d = frontierPoint - goal;

        return _config.KOccupancy * Math.Exp(-((d.X * d.X + d.Y * d.Y) / (2.0 * radius * radius)));
    }

    private Vector2ds[] GetFrontiers()
    {
        Assert.IsTrue(FrontierRegions.Count > 0);

        var rc = Peers.Count + 1;

        _originalFrontierCount = FrontierRegions.Count;

        if (FrontierRegions.Count >= rc)
        {
            return FrontierRegions.Keys.ToArray();
        }

        Console.WriteLine("Insufficient frontiers!");

        var closest = FrontierRegions.Keys.ToList();
        var costs = new Dictionary<Vector2ds, double>();

        foreach (var frontier in closest)
        {
            costs[frontier] = Peers.Sum(p => Vector2ds.DistanceSqr(frontier, p.Position)) +
                              Vector2ds.DistanceSqr(frontier, Position);
        }

        closest.Sort((a, b) => costs[a].CompareTo(costs[b]));

        var adjustedFrontiers = new List<Vector2ds>(rc);

        while (adjustedFrontiers.Count < rc)
        {
            for (var i = 0; i < closest.Count && adjustedFrontiers.Count < rc; i++)
            {
                adjustedFrontiers.Add(closest[i]);
            }    
        }

        Assert.IsTrue(adjustedFrontiers.Count == rc);
        Debug.Assert(FrontierRegions.Keys.All(k => adjustedFrontiers.Contains(k)));

        return adjustedFrontiers.ToArray();
    }

    private double[] CalculateUtilities(Vector2ds[] frontiers)
    {
        var results = new double[frontiers.Length];

        for (var i = 0; i < frontiers.Length; i++)
        {
            results[i] = Utility(frontiers[i]);
        }

        return results;
    }

    private static double CooperativeCost(Vector2ds frontier, SmartRobot robot)
    {
        var result = 0d;
        
        foreach (var peer in robot.Peers)
        {
            result += robot.Occupancy(frontier, peer.GetInformation().Goal);
        }

        return result;
    }

    private Grid2d<double> CalculateWeights(Vector2ds[] frontiers, SmartRobot[] robots)
    {
        var utilities = CalculateUtilities(frontiers);
       
        Assert.IsTrue(frontiers.Length >= robots.Length);
        Assert.IsTrue(robots.First() == this);

        var weights = new Grid2d<double>(frontiers.Length, frontiers.Length);

        for (var i = 0; i < robots.Length; i++)
        {
            var robot = robots[i];

            for (var j = 0; j < frontiers.Length; j++)
            {
                var frontier = frontiers[j];

                var implicitCost = ImplicitCost(frontier, Position);
                var cooperativeCost = CooperativeCost(frontier, robot);
                weights[i, j] = implicitCost + cooperativeCost - utilities[j];
            }
        }

        return weights;
    }

    protected override void PrepareUpdateCore()
    {
        foreach (var peer in Peers)
        {
            foreach (var (pos, v) in View)
            {
                peer.LoadFoundTile(pos, v);
            }
        }

        if (!_initialized)
        {
            _initialized = true;
            _goal = Position;
        }
    }

    public void StartMission()
    {
        _frontiers = GetFrontiers();
        
        var robots = Peers.Prepend(this).ToArray();
        _weights = CalculateWeights(_frontiers, robots);

        var assignments = HungarianAlgorithm.FindAssignments(_weights!);

        _goal = _frontiers[assignments[0]];
        _weight = _weights[0, assignments[0]];

        CreatePath(_goal);
    }

    protected override void UpdateCore()
    {
        if (Follow())
        {
            return;
        }

        StartMission();
    }

    private bool Follow()
    {
        if (_path?.Last == null || _path.First == null || Position == _path.Last.Value)
        {
            goto skip;
        }

        Debug.Assert(_path.First.Value == Position);

        _path.RemoveFirst();

        // Check if path is still possible:

        foreach (var position in _path)
        {
            if (!IsWalkable(position))
            {
                goto skip;
            }
        }

        ApplyMove(Position.Base8DirectionTo(_path.First.Value));

        return true;

        skip:
        _path = null;
        return false;
    }

    private void CreatePath(Vector2ds frontier)
    {
        var path = FindPath(Position, frontier);

        Assert.NotNull(ref path);
        Assert.IsTrue(path.First() == Position);

        _path = new LinkedList<Vector2ds>(path);
    }

    public override void DebugDraw(QuadBatch batch, Vector2ds mouse)
    {
        if (_path != null)
        {
            Vector2ds? a = null;

            foreach (var b in _path)
            {
                if (a != null)
                {
                    batch.Line(a.Value, b, Color, 0.1f);
                }

                a = b;
            }
        }
    }

    public override void AddTooltips(StringBuilder sb, Vector2ds mouse)
    {
        if (_frontiers != null)
        {
            sb.AppendLine($"Effective frontiers: {_originalFrontierCount}, Padded: {_frontiers.Length}");
        }

        sb.AppendLine($"View tiles: {View.Count}");

        if (_weights != null && _frontiers != null && _frontiers.Length > 0)
        {
            var pickedCost = double.MaxValue;
            var pickedIndex = 0;

            for (var index = 0; index < _frontiers.Length; index++)
            {
                var frontier = _frontiers[index];
                var cost = Vector2ds.DistanceSqr(frontier, mouse);

                if (cost < pickedCost)
                {
                    pickedCost = cost;
                    pickedIndex = index;
                }
            }

            if (pickedCost < 12)
            {
                sb.AppendLine($"Weight: {_weights[0, pickedIndex]} ({_weight})");
            }
        }
    }

    public static class HungarianAlgorithm
    {
        private const double Epsilon = 1e-8;

        public static int[] FindAssignments(Grid2d<double> costs)
        {
            costs = costs.Bind();

            var rows = costs.Size.X;
            var columns = costs.Size.Y;

            for (var i = 0; i < rows; i++)
            {
                var min = double.MaxValue;

                for (var j = 0; j < columns; j++)
                {
                    min = Math.Min(min, costs[i, j]);
                }

                for (var j = 0; j < columns; j++)
                {
                    costs[i, j] -= min;
                }
            }

            var masks = new byte[rows, columns];
            var rowsCovered = new bool[rows];
            var colsCovered = new bool[columns];

            for (var i = 0; i < rows; i++)
            {
                for (var j = 0; j < columns; j++)
                {
                    if (costs[i, j].ApproxEq(0.0, Epsilon) && !rowsCovered[i] && !colsCovered[j])
                    {
                        masks[i, j] = 1;
                        rowsCovered[i] = true;
                        colsCovered[j] = true;
                    }
                }
            }

            ClearCovers(rowsCovered, colsCovered, columns, rows);

            var path = new Location[columns * rows];
            var pathStart = default(Location);
            var step = 1;

            while (step != -1)
            {
                step = step switch
                {
                    1 => RunStep1(masks, colsCovered, columns, rows),
                    2 => RunStep2(costs, masks, rowsCovered, colsCovered, columns, rows, ref pathStart),
                    3 => RunStep3(masks, rowsCovered, colsCovered, columns, rows, path, pathStart),
                    4 => RunStep4(costs, rowsCovered, colsCovered, columns, rows),
                    _ => step
                };
            }

            var assignments = new int[rows];

            for (var i = 0; i < rows; i++)
            {
                for (var j = 0; j < columns; j++)
                {
                    if (masks[i, j] == 1)
                    {
                        assignments[i] = j;
                        break;
                    }
                }
            }

            return assignments;
        }

        private static int RunStep1(byte[,] masks, bool[] colsCovered, int w, int h)
        {
            for (var i = 0; i < h; i++)
            {
                for (var j = 0; j < w; j++)
                {
                    if (masks[i, j] == 1)
                    {
                        colsCovered[j] = true;
                    }
                }
            }

            var colsCoveredCount = 0;

            for (var j = 0; j < w; j++)
            {
                if (colsCovered[j])
                {
                    colsCoveredCount++;
                }
            }

            if (colsCoveredCount == h)
            {
                return -1;
            }

            return 2;
        }

        private static int RunStep2(Grid2d<double> costs, byte[,] masks, bool[] rowsCovered, bool[] colsCovered, int w, int h, ref Location pathStart)
        {
            while (true)
            {
                var loc = FindZero(costs, rowsCovered, colsCovered, w, h);

                if (loc.Row == -1)
                {
                    return 4;
                }

                masks[loc.Row, loc.Column] = 2;

                var starCol = FindStarInRow(masks, w, loc.Row);

                if (starCol != -1)
                {
                    rowsCovered[loc.Row] = true;
                    colsCovered[starCol] = false;
                }
                else
                {
                    pathStart = loc;
                    return 3;
                }
            }
        }

        private static int RunStep3(byte[,] masks, bool[] rowsCovered, bool[] colsCovered, int w, int h, Location[] path, Location pathStart)
        {
            var pathIndex = 0;
            path[0] = pathStart;

            while (true)
            {
                var row = FindStarInColumn(masks, h, path[pathIndex].Column);

                if (row == -1)
                {
                    break;
                }

                pathIndex++;

                path[pathIndex] = new Location(row, path[pathIndex - 1].Column);

                var col = FindPrimeInRow(masks, w, path[pathIndex].Row);

                pathIndex++;
                path[pathIndex] = new Location(path[pathIndex - 1].Row, col);
            }

            ConvertPath(masks, path, pathIndex + 1);
            ClearCovers(rowsCovered, colsCovered, w, h);
            ClearPrimes(masks, w, h);

            return 1;
        }

        private static int RunStep4(Grid2d<double> costs, bool[] rowsCovered, bool[] colsCovered, int w, int h)
        {
            var minValue = FindMinimum(costs, rowsCovered, colsCovered, w, h);

            for (var i = 0; i < h; i++)
            {
                for (var j = 0; j < w; j++)
                {
                    if (rowsCovered[i])
                    {
                        costs[i, j] += minValue;
                    }

                    if (!colsCovered[j])
                    {
                        costs[i, j] -= minValue;
                    }
                }
            }

            return 2;
        }

        private static double FindMinimum(Grid2d<double> costs, bool[] rowsCovered, bool[] colsCovered, int w, int h)
        {
            var minValue = double.MaxValue;

            for (var i = 0; i < h; i++)
            {
                for (var j = 0; j < w; j++)
                {
                    if (!rowsCovered[i] && !colsCovered[j])
                    {
                        minValue = Math.Min(minValue, costs[i, j]);
                    }
                }
            }

            return minValue;
        }

        private static int FindStarInRow(byte[,] masks, int w, int row)
        {
            for (var j = 0; j < w; j++)
            {
                if (masks[row, j] == 1)
                {
                    return j;
                }
            }

            return -1;
        }

        private static int FindStarInColumn(byte[,] masks, int h, int col)
        {
            for (var i = 0; i < h; i++)
            {
                if (masks[i, col] == 1)
                {
                    return i;
                }
            }

            return -1;
        }

        private static int FindPrimeInRow(byte[,] masks, int w, int row)
        {
            for (var j = 0; j < w; j++)
            {
                if (masks[row, j] == 2)
                {
                    return j;
                }
            }

            return -1;
        }

        private static Location FindZero(Grid2d<double> costs, bool[] rowsCovered, bool[] colsCovered, int w, int h)
        {
            for (var i = 0; i < h; i++)
            {
                for (var j = 0; j < w; j++)
                {
                    if (costs[i, j].ApproxEq(0, Epsilon) && !rowsCovered[i] && !colsCovered[j])
                    {
                        return new Location(i, j);
                    }
                }
            }

            return new Location(-1, -1);
        }

        private static void ConvertPath(byte[,] masks, Location[] path, int pathLength)
        {
            for (var i = 0; i < pathLength; i++)
            {
                masks[path[i].Row, path[i].Column] = masks[path[i].Row, path[i].Column] switch
                {
                    1 => 0,
                    2 => 1,
                    _ => masks[path[i].Row, path[i].Column]
                };
            }
        }

        private static void ClearPrimes(byte[,] masks, int w, int h)
        {
            for (var i = 0; i < h; i++)
            {
                for (var j = 0; j < w; j++)
                {
                    if (masks[i, j] == 2)
                    {
                        masks[i, j] = 0;
                    }
                }
            }
        }

        private static void ClearCovers(bool[] rowsCovered, bool[] colsCovered, int w, int h)
        {
            Array.Fill(rowsCovered, false);
            Array.Fill(colsCovered, false);
        }

        private readonly struct Location
        {
            public int Row { get; }
            public int Column { get; }

            public Location(int row, int col)
            {
                Row = row;
                Column = col;
            }
        }
    }
}