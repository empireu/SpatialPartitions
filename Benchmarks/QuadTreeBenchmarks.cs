using System.Diagnostics;
using Common;
using QuadTreeVisualization;

namespace Benchmarks;

internal static class QuadTreeBenchmarks
{
    public static void Run()
    {
        var iter = 100;

        for (int log = 5; log <= 10; log++)
        {
            var size = 1 << log;

            var points = new List<(Vector2di, byte)>();
            var hs = new HashSet<Vector2di>();

            while (points.Count < size * size * 0.1f)
            {
                var pos = new Vector2di(Random.Shared.Next(0, size), -Random.Shared.Next(0, size));

                if (hs.Add(pos))
                {
                    points.Add((pos, (byte)Random.Shared.Next(1, 10)));
                }
            }

            Console.WriteLine($"Log {log:##}: {points.Count} points, {iter} iterations\n");
            Console.WriteLine("RANDOM DATA\n");

            Benchmark(iter, log, points);

            points.Clear();
            hs.Clear();

            while (points.Count < size * size * 0.1f)
            {
                var topLeft = new Vector2di(Random.Shared.Next(0, size - 1), -Random.Shared.Next(0, size - 1));
                var extent = new Vector2di(Random.Shared.Next(1, (size - topLeft.X)) / 10, Random.Shared.Next(1, (size + topLeft.Y)) / 10);
                var type = (byte)Random.Shared.Next(1, 10);
              
                for (int y = topLeft.Y; y > topLeft.Y - extent.Y; y--)
                {
                    for (int x = topLeft.X; x < topLeft.X + extent.X; x++)
                    {
                        var v = new Vector2di(x, y);

                        if (hs.Add(v))
                        {
                            points.Add((v, type));
                        }
                        else
                        {
                            points.RemoveAt(points.FindIndex(t => t.Item1 == v));
                            points.Add((v, type));
                        }
                    }
                }
            }

            Console.WriteLine("STRUCTURED DATA\n");

            Benchmark(iter, log, points);

            iter /= 2;

            if (iter <= 0)
            {
                iter = 1;
            }
        }
    }

    private static void Benchmark(int iter, int log, List<(Vector2di, byte)> points)
    {
        var sw = new Stopwatch();

        var times = new List<(QuadTree<byte>, double add, double query, double remove)>();

        var trees = new QuadTree<byte>[]
        {
            new ClassicQuadTree<byte>((byte)log),
            new LinkedQuadTree<byte>((byte)log),
            new ContiguousQuadTree<byte>((byte)log)
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

    private static void BenchmarkTree(QuadTree<byte> tree, int iter, List<(Vector2di, byte)> points, out double avgAddTime, out double avgQueryTime, out double avgRemoveTime)
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