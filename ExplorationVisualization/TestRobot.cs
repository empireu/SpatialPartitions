using System.Diagnostics;
using Common;
using GameFramework.Extensions;
using GameFramework.Renderer.Batch;

namespace ExplorationVisualization;

internal sealed class TestRobot : Robot<TestRobot>
{
    private LinkedList<Vector2ds>? _lastPath;

    public TestRobot(RobotCreateInfo ci) : base(ci)
    {

    }

    private static void CommunicateFindings(IReadOnlyDictionary<Vector2ds, bool> view, IReadOnlyList<TestRobot> peers)
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

    protected override void UpdateCore()
    {
        CommunicateFindings(View, Peers);

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

    public override void DebugDraw(QuadBatch batch, Vector2ds mouse)
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