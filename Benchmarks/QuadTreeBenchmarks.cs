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

        var linkedTree = new LinkedQuadTree<byte>((byte)log);
        var contigTree = new ContiguousQuadTree<byte>((byte)log);

        var finalLinkedNodeCount = 0;
        var finalContigNodeCount = 0;

        double Measure(Action b)
        {
            sw.Restart();
            b();
            sw.Stop();
            return sw.Elapsed.TotalMilliseconds;
        }

        var linkedTimes = new List<(double add, double remove, double query)>();
        var contigTimes = new List<(double add, double remove, double query)>();

        for (int iteration = 0; iteration < iter; iteration++)
        {
            // Linked:
            {

                var linkedInsertTime = Measure(() =>
                {
                    for (var index = 0; index < points.Count; index++)
                    {
                        var (pos, data) = points[index];
                        linkedTree.Insert(pos, data);
                    }
                });

                finalLinkedNodeCount = linkedTree.NodeCount;

                var linkedQueryTime = Measure(() =>
                {
                    for (var i = 0; i < points.Count; i++)
                    {
                        var index = linkedTree.Find(points[i].Item1);
                    }

                });

                var linkedRemoveTime = Measure(() =>
                {
                    for (var index = 0; index < points.Count; index++)
                    {
                        var (pos, data) = points[index];

                        if (!linkedTree.Remove(pos))
                        {
                            throw new InvalidOperationException();
                        }
                    }
                });

                if (linkedTree.NodeCount != 1)
                {
                    throw new InvalidOperationException();
                }

                linkedTimes.Add((linkedInsertTime, linkedRemoveTime, linkedQueryTime));

            }

            // Contig:
            {

                var contigInsertTime = Measure(() =>
                {
                    for (var index = 0; index < points.Count; index++)
                    {
                        var (pos, data) = points[index];
                        contigTree.Insert(pos, data);
                    }
                });

                finalContigNodeCount = contigTree.NodeCount;

                var contigQueryTime = Measure(() =>
                {
                    for (var i = 0; i < points.Count; i++)
                    {
                        var index = contigTree.Find(points[i].Item1);
                    }

                });

                var contigRemovalTime = Measure(() =>
                {
                    for (var index = 0; index < points.Count; index++)
                    {
                        var (pos, data) = points[index];

                        if (!contigTree.Remove(pos))
                        {
                            throw new InvalidOperationException();
                        }
                    }
                });

                if (contigTree.NodeCount != 1)
                {
                    throw new InvalidOperationException();
                }

                contigTimes.Add((contigInsertTime, contigRemovalTime, contigQueryTime));
            }
        }

        var avgLinkedAdd = linkedTimes.Average(x => x.add);
        var avgLinkedQuery = linkedTimes.Average(x => x.query);
        var avgLinkedRemove = linkedTimes.Average(x => x.remove);
        var avgContigAdd = contigTimes.Average(x => x.add);
        var avgContigQuery = contigTimes.Average(x => x.query);
        var avgContigRemove = contigTimes.Average(x => x.remove);

        Console.WriteLine($"Log {log:##}: {points.Count} points, nodes: {finalLinkedNodeCount} linked, {finalContigNodeCount} contiguous, {iter} iterations\n" +
                          $"  Linked:\n" +
                          $"    Add: {avgLinkedAdd:F3}ms ({((avgLinkedAdd * 1000.0) / points.Count):F4} µs/p)\n" +
                          $"    Query: {avgLinkedQuery:F3}ms ({((avgLinkedQuery * 1000.0) / points.Count):F4} µs/p)\n" +
                          $"    Remove: {avgLinkedRemove:F3}ms ({((avgLinkedRemove * 1000.0) / points.Count):F4} µs/p)\n" +
                          $"  Contiguous:\n" +
                          $"    Add: {avgContigAdd:F3}ms ({((avgContigAdd * 1000.0) / points.Count):F4} µs/p)\n" +
                          $"    Query: {avgContigQuery:F3}ms ({((avgContigQuery * 1000.0) / points.Count):F4} µs/p)\n" +
                          $"    Remove: {avgContigRemove:F3}ms ({((avgContigRemove * 1000.0) / points.Count):F4} µs/p)\n\n");
    }
}