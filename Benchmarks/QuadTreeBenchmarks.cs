using System.Diagnostics;
using Common;
using QuadTreeVisualization;

namespace Benchmarks;

internal static class QuadTreeBenchmarks
{

    private static void GeneratePointSet(
        int log,
        float pointDensity,
        float structureSize,
        byte values,
        out List<(Vector2ds, byte)> randomPoints,
        out List<(Vector2ds, byte)> structuredPoints)
    {
        randomPoints = new List<(Vector2ds, byte)>();

        var hs = new HashSet<Vector2ds>();
        var size = 1 << log;
        var cpTarget = Math.Max(1, size * size * pointDensity);

        while (randomPoints.Count < cpTarget)
        {
            var pos = new Vector2ds(Random.Shared.Next(0, size), -Random.Shared.Next(0, size));

            if (hs.Add(pos))
            {
                randomPoints.Add((pos, RandomTileType()));
            }
        }

        hs.Clear();

        var structureUb = (int)Math.Ceiling(size * structureSize);
        var structuredGrid = new Dictionary<Vector2ds, byte>();

        while (structuredGrid.Count < cpTarget)
        {
            var topLeft = new Vector2ds(Random.Shared.Next(0, size - 1), -Random.Shared.Next(0, size - 1));
            var extent = new Vector2ds(Random.Shared.Next(1, Math.Min(structureUb, size - topLeft.X)), Random.Shared.Next(1, Math.Min(structureUb, size + topLeft.Y)));
            var type = RandomTileType();

            for (int y = topLeft.Y; y > topLeft.Y - extent.Y; y--)
            {
                for (int x = topLeft.X; x < topLeft.X + extent.X; x++)
                {
                    structuredGrid[new Vector2ds(x, y)] = type;
                }
            }
        }

        structuredPoints = structuredGrid.Keys.Select(k => (k, structuredGrid[k])).ToList();
        return;

        byte RandomTileType() => (byte)Random.Shared.Next(1, values + 1);
    }

    // todo investigate possible linkedquadtree bug
    public static void Run()
    {
        var iter = 100;

        for (int log = 5; log <= 10; log++)
        {
            GeneratePointSet(log, 0.4f, (1 << log) / 5f, 4, out var randomPoints, out var structuredPoints);

            Console.WriteLine($"Log {log:##}: {randomPoints.Count} rp, {structuredPoints.Count} sp, {iter} iterations\n");
            Console.WriteLine("RANDOM DATA\n");

            PrintBenchmarks(iter, log, randomPoints);

            Console.WriteLine("\nSTRUCTURED DATA\n");

            PrintBenchmarks(iter, log, structuredPoints);

            iter /= 2;

            if (iter <= 0)
            {
                iter = 1;
            }

            Console.WriteLine("\n\n\n");
        }
    }

    private static void PrintBenchmarks(int iter, int log, List<(Vector2ds, byte)> points)
    {
        var sw = new Stopwatch();

        var times = new List<(IQuadTree<byte>, double add, double query, double remove)>();

        var trees = new IQuadTree<byte>[]
        {
            new ClassicQuadTree<byte>((byte)log),
            new HashedQuadTree<byte>((byte)log)
        };

        foreach (var quadTree in trees)
        {
            BenchmarkTree(quadTree, iter, points, out var add, out var query, out var remove);
            times.Add((quadTree, add, query, remove));
        }

        foreach (var (tree, add, query, remove) in times)
        {
            Console.WriteLine($"""
                              {tree.GetType().Name}:
                                A: {TimeStr(add)}
                                Q: {TimeStr(query)}
                                R: {TimeStr(remove)}
                              """);
        }

        return;

        string TimeStr(double time) => $"{time:F3}ms (({((time * 1000.0) / points.Count):F4} µs/p))";
    }

    private static void BenchmarkTree(IQuadTree<byte> tree, int iter, List<(Vector2ds, byte)> points, out double avgAddTime, out double avgQueryTime, out double avgRemoveTime)
    {
        var times = new List<(double add, double remove, double query)>();

        for (int iteration = 0; iteration < iter; iteration++)
        {
            var insertTime = Measure(() =>
            {
                for (var index = 0; index < points.Count; index++)
                {
                    var (pos, data) = points[index];
                    tree.Insert(pos, data);
                }
            });

            var queryTime = Measure(() =>
            {
                for (var i = 0; i < points.Count; i++)
                {
                    var index = tree.Find(points[i].Item1);
                }
            });

            var removeTime = Measure(() =>
            {
                for (var index = 0; index < points.Count; index++)
                {
                    var (pos, data) = points[index];

                    if (!tree.Remove(pos))
                    {
                        throw new InvalidOperationException();
                    }
                }
            });

            if (tree.NodeCount != 1)
            {
                throw new InvalidOperationException();
            }

            times.Add((insertTime, removeTime, queryTime));
        }

        avgAddTime = times.Average(x => x.add);
        avgQueryTime = times.Average(x => x.query);
        avgRemoveTime = times.Average(x => x.remove);
        return;

        static double Measure(Action b)
        {
            var sw = Stopwatch.StartNew();
            b();
            sw.Stop();
            return sw.Elapsed.TotalMilliseconds;
        }
    }
}