using System;
using System.Collections;
using System.ComponentModel;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Common;
using GameFramework.Utilities;
using Vortice.Mathematics;

namespace OctTreeVisualization;

public struct BitOctreeNode
{
    public byte ChildMask;
    public bool IsFilled;

    public readonly bool HasChildren => ChildMask != 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ActivateChild(int childIdx) => ChildMask |= (byte)(1 << childIdx);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ResetChild(byte childIdx) => ChildMask &= (byte)(byte.MaxValue ^ (1 << childIdx));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ActivateChildren() => ChildMask = byte.MaxValue;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ResetChildren() => ChildMask = 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool HasChild(byte childIdx) => ((ChildMask >> childIdx) & 1) > 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly int ChildBit(byte childIdx) => ((ChildMask >> childIdx) & 1);
}

public readonly struct OctreeNodeBounds
{
    public OctreeNodeBounds(Vector3di position, byte log)
    {
        Position = position;
        Log = log;
    }

    public Vector3di Position { get; }
    public byte Log { get; }

    public static implicit operator BoundingBox3di(OctreeNodeBounds b) => new(b.Position, b.Position + (1 << b.Log));
}

public readonly struct HashedOctreeNodeInfo
{
    public HashedOctreeNodeInfo(ulong locationCode, BitOctreeNode node, OctreeNodeBounds bounds)
    {
        LocationCode = locationCode;
        Node = node;
        Bounds = bounds;
    }

    public ulong LocationCode { get; }

    public BitOctreeNode Node { get; }

    // Log can be inferred from location code in constant time (if you have clz instructions available),
    // but position is O(h) so we may as well store both
    public OctreeNodeBounds Bounds { get; }
}

// Bit Octree, as in, 2 states per voxel.

public abstract class Pool
{
    public static Pool<List<T>> ForList<T>() =>
        new(() => new List<T>(), t => t.Clear());

    public static Pool<HashSet<T>> ForHashSet<T>() =>
        new(() => new HashSet<T>(), t => t.Clear());

    public static Pool<Stack<T>> ForStack<T>() =>
        new(() => new Stack<T>(), t => t.Clear());

    public static Pool<Queue<T>> ForQueue<T>() =>
        new(() => new Queue<T>(), t => t.Clear());

    public static Pool<PriorityQueue<TElement, TPriority>> ForPriorityQueue<TElement, TPriority>() =>
        new(() => new PriorityQueue<TElement, TPriority>(), t => t.Clear());
}

// Basic object pool
public sealed class Pool<T> : Pool where T : class
{
    private readonly Func<T> _factory;
    private readonly Action<T>? _clear;
    private readonly Stack<T> _free = new();
    private readonly HashSet<T> _inUse = new();

    public Pool(Func<T> factory, Action<T>? clear = null, int maxItems = 1024)
    {
        _factory = factory;
        _clear = clear;
        MaxItems = maxItems;
    }

    public int MaxItems { get; }
    public int FreeCount => _free.Count;
    public int InUseCount => _inUse.Count;
    public int AllocationCount { get; private set; }

    private T PopOrAllocate()
    {
        T element;

        if (_free.TryPop(out var freeItem))
        {
            element = freeItem;
        }
        else
        {
            element = _factory();
            ++AllocationCount;
        }

        Assert.IsTrue(_inUse.Add(element));
        return element;
    }

    public Handler Handle(out T item)
    {
        item = PopOrAllocate();
        return new Handler(this, item);
    }

    public Handler Handle() => new(this, PopOrAllocate());

    public T Rent() => PopOrAllocate();

    public void Return(T item)
    {
        if (!_inUse.Remove(item))
        {
            throw new ArgumentException("Returned item did not originate from the pool");
        }

        if (_free.Count < MaxItems)
        {
            _clear?.Invoke(item);
            _free.Push(item);
        }
    }

    public readonly struct Handler : IDisposable
    {
        private readonly Pool<T> _pool;
        public readonly T Item;

        public Handler(Pool<T> pool, T item)
        {
            _pool = pool;
            Item = item;
        }

        public void Dispose()
        {
            _pool.Return(Item);
        }
    }
}

public sealed class BitOctree
{
    private static readonly Base6Direction3dMask[] DirectionMaskByOctant;
    private static readonly byte[][] OctantsByDirection;
    private static readonly byte[][] ReflectionsByDirection;

    static BitOctree()
    {
        DirectionMaskByOctant = new Base6Direction3dMask[8];
        var octantsByDirection = new List<byte>[6];

        for (int i = 0; i < octantsByDirection.Length; i++)
        {
            octantsByDirection[i] = new List<byte>();
        }

        for (byte i = 0; i < 8; i++)
        {
            var x = (i >> 0) & 1;
            var y = (i >> 1) & 1;
            var z = (i >> 2) & 1;
            var xd = (x == 0 ? Base6Direction3d.L : Base6Direction3d.R);
            var yd = (y == 0 ? Base6Direction3d.D : Base6Direction3d.U);
            var zd = (z == 0 ? Base6Direction3d.F : Base6Direction3d.B);

            DirectionMaskByOctant[i] = xd.Mask() | yd.Mask() | zd.Mask();

            octantsByDirection[(int)xd].Add(i);
            octantsByDirection[(int)yd].Add(i);
            octantsByDirection[(int)zd].Add(i);

        }

        OctantsByDirection = octantsByDirection.Select(x => x.ToArray()).ToArray();

        ReflectionsByDirection = new byte[6][];

        for (var i = 0; i < 6; i++)
        {
            ReflectionsByDirection[i] = new byte[8];

            var dir = (Base6Direction3d)i;

            for (var j = 0; j < 8; j++)
            {
                var x = (j >> 0) & 1;
                var y = (j >> 1) & 1;
                var z = (j >> 2) & 1;

                switch (dir)
                {
                    case Base6Direction3d.L:
                        x = x == 0 ? 1 : 0;
                        break;
                    case Base6Direction3d.R:
                        x = x == 1 ? 0 : 1;
                        break;
                    case Base6Direction3d.U:
                        y = y == 1 ? 0 : 1;
                        break;
                    case Base6Direction3d.D:
                        y = y == 0 ? 1 : 0;
                        break;
                    case Base6Direction3d.F:
                        z = z == 0 ? 1 : 0;
                        break;
                    case Base6Direction3d.B:
                        z = z == 1 ? 0 : 1;
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }

                ReflectionsByDirection[i][j] = (byte)(x << 0 | y << 1 | z << 2);
            }
        }
    }

    public delegate bool TraverseDelegate(BitOctreeNode node, ulong lc, Vector3di position, byte log);

    public const ulong RootCode = 1;
    public const ulong NullCode = 0;

    private readonly Dictionary<ulong, BitOctreeNode> _nodes = new();

    private readonly Pool<Stack<DescentData>> _descentStackPool = Pool.ForStack<DescentData>();
    private readonly Pool<Stack<LogTraversalData>> _logTraversalStackPool = Pool.ForStack<LogTraversalData>();
    private readonly Pool<Stack<PositionLogTraversalData>> _positionLogTraversalStackPool = Pool.ForStack<PositionLogTraversalData>();
    private readonly Pool<PriorityQueue<LogTraversalData, double>> _logTraversalQueuePool = Pool.ForPriorityQueue<LogTraversalData, double>();

    private readonly PriorityQueue<NodeInfo, float> _nodeQueryQueue = new();
    private readonly PriorityQueue<RegionInfo, float> _regionQueryQueue = new();

    public BitOctree(int log)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(log, 1, nameof(log));
        ArgumentOutOfRangeException.ThrowIfGreaterThan(log, 21, nameof(log));

        Log = (byte)log;

        _nodes.Add(RootCode, new BitOctreeNode());
    }

    public byte Log { get; }
    public int EdgeSize => 1 << Log;
    public Vector3di Min => Vector3di.Zero;
    public Vector3di Max => new(EdgeSize);
    public BoundingBox3di Bounds => new(Min, Max);
    public int NodeCount => _nodes.Count;

    public int Version { get; private set; }

    public bool IsWithinBounds(Vector3di point) => Bounds.ContainsExclusive(point);

    private void AddNode(ulong code, BitOctreeNode node)
    {
        _nodes.Add(code, node);
    }

    private void RemoveNode(ulong code)
    {
#if DEBUG
        Debug.Assert(_nodes.Remove(code));
#else
        _nodes.Remove(code);
#endif
    }

    #region Insertion

    public bool Insert(Vector3di tile)
    {
        if (!IsWithinBounds(tile))
        {
            throw new ArgumentOutOfRangeException(nameof(tile), "Outside of the bounds of the octree");
        }

        using (_descentStackPool.Handle(out var stack))
        {
            var result = InsertCore(stack, RootCode, Log, tile);

            if (result)
            {
                Fill(stack);
                ++Version;
            }

            return result;
        }
    }

    private bool InsertCore(Stack<DescentData> stack, ulong lcNode, byte log, Vector3di targetPos)
    {
        while (true)
        {
            Debug.Assert(log > 0);

            var node = _nodes[lcNode];

            if (node.IsFilled)
            {
                // Node is filled, so we can't insert anything.
                return false;
            }

            var octant = Descend(ref targetPos, log);

            stack.Push(new DescentData
            {
                Lc = lcNode,
                Log = log,
                Octant = octant
            });

            if (log == 1)
            {
                if (node.HasChild(octant))
                {
                    // Already set, so we can't insert here.
                    return false;
                }

                node.ActivateChild(octant);

                _nodes[lcNode] = node;

                return true;
            }

            var lcChild = ChildCode(lcNode, octant);

            if (!node.HasChild(octant))
            {
                node.ActivateChild(octant);
                _nodes[lcNode] = node;
                AddNode(lcChild, new BitOctreeNode());
            }

            lcNode = lcChild;
            --log;
        }
    }

    private void Fill(Stack<DescentData> stack)
    {
        while (stack.TryPop(out var frame))
        {
            var lcNode = frame.Lc;
            var node = _nodes[lcNode];

            Debug.Assert(!node.IsFilled);

            if (node.ChildMask != byte.MaxValue)
            {
                return;
            }

            if (frame.Log > 1)
            {
                for (byte i = 0; i < 8; i++)
                {
                    var child = _nodes[ChildCode(lcNode, i)];

                    if (!child.IsFilled)
                    {
                        // Can't solidify the parent node, which also means we can't solidify further up.
                        return;
                    }
                }

                for (byte i = 0; i < 8; i++)
                {
                    RemoveNode(ChildCode(lcNode, i));
                }
            }

            node.ResetChildren();
            node.IsFilled = true;

            _nodes[lcNode] = node;
        }
    }

    #endregion

    #region Removal

    public bool Remove(Vector3di tile)
    {
        if (!IsWithinBounds(tile))
        {
            throw new ArgumentOutOfRangeException(nameof(tile), "Outside of the bounds of the octree");
        }

        using (_descentStackPool.Handle(out var stack))
        {
            var result = RemoveCore(stack, RootCode, Log, tile);

            if (result)
            {
                Trim(stack);
                ++Version;
            }

            return result;
        }
    }

    private bool RemoveCore(Stack<DescentData> stack, ulong lcNode, byte log, Vector3di targetPos)
    {
        while (true)
        {
            var node = _nodes[lcNode];
            var octant = Descend(ref targetPos, log);

            stack.Push(new DescentData
            {
                Lc = lcNode,
                Log = log,
                Octant = octant
            });

            if (node.IsFilled)
            {
                node.IsFilled = false;
                node.ActivateChildren();

                if (log > 1)
                {
                    // Split into 8 nodes:
                    for (byte i = 0; i < 8; i++)
                    {
                        _nodes.Add(ChildCode(lcNode, i), new BitOctreeNode
                        {
                            IsFilled = true
                        });
                    }
                }
            }
            else if (!node.HasChild(octant))
            {
                return false;
            }

            if (log == 1)
            {
                node.ResetChild(octant);

                _nodes[lcNode] = node;

                return true;
            }

            _nodes[lcNode] = node;
            lcNode = ChildCode(lcNode, octant);
            --log;
        }
    }

    private void Trim(Stack<DescentData> stack)
    {
        while (stack.TryPop(out var frame) && stack.Count > 0)
        {
            var node = _nodes[frame.Lc];

            if (node.IsFilled || node.HasChildren)
            {
                return;
            }

            var parentFrame = stack.Peek();
            var parentNode = _nodes[parentFrame.Lc];

            Debug.Assert(parentNode.HasChild(parentFrame.Octant));

            parentNode.ResetChild(parentFrame.Octant);
            _nodes[parentFrame.Lc] = parentNode;
            _nodes.Remove(frame.Lc);
        }
    }

    #endregion

    #region Traversal

    public void Traverse(TraverseDelegate traverse)
    {
        using (_positionLogTraversalStackPool.Handle(out var stack))
        {
            stack.Push(new PositionLogTraversalData
            {
                Lc = RootCode,
                Position = Vector3di.Zero,
                Log = Log
            });

            while (stack.Count > 0)
            {
                var frame = stack.Pop();
                var lcNode = frame.Lc;
                var node = _nodes[lcNode];

                if (!traverse(node, lcNode, frame.Position, frame.Log))
                {
                    return;
                }

                if (frame.Log == 1)
                {
                    for (byte i = 0; i < 8; i++)
                    {
                        if (node.HasChild(i))
                        {
                            if (!traverse(new BitOctreeNode { IsFilled = true }, ChildCode(lcNode, i), frame.Position + OctantPositionIncrement(i, frame.Log), 0))
                            {
                                return;
                            }
                        }
                    }
                }
                else
                {
                    for (byte i = 0; i < 8; i++)
                    {
                        if (node.HasChild(i))
                        {
                            stack.Push(new PositionLogTraversalData
                            {
                                Lc = ChildCode(lcNode, i),
                                Position = frame.Position + OctantPositionIncrement(i, frame.Log),
                                Log = (byte)(frame.Log - 1)
                            });
                        }
                    }
                }
            }
        }
    }

    // Assumes target is false by default
    public void ReadRange(Vector3di min, IGrid3d<bool> results)
    {
        using (_positionLogTraversalStackPool.Handle(out var stack))
        {
            stack.Push(new PositionLogTraversalData
            {
                Lc = RootCode,
                Position = Vector3di.Zero,
                Log = Log
            });

            var targetBounds = new BoundingBox3di(min, min + results.Size);

            while (stack.TryPop(out var frame))
            {
                var node = _nodes[frame.Lc];
                var position = frame.Position;
                var log = frame.Log;

                if (node.IsFilled)
                {
                    var size = 1 << log;

                    var intersection = BoundingBox3di.Intersect(
                        new BoundingBox3di(position, position + size),
                        targetBounds
                    );

                    Debug.Assert(intersection.IsValid);

                    for (var z = intersection.Min.Z; z < intersection.Max.Z; z++)
                    {
                        for (var y = intersection.Min.Y; y < intersection.Max.Y; y++)
                        {
                            for (var x = intersection.Min.X; x < intersection.Max.X; x++)
                            {
                                Debug.Assert(targetBounds.ContainsExclusive(new Vector3di(x, y, z)));
                                results[x - min.X, y - min.Y, z - min.Z] = true;
                            }
                        }
                    }
                }
                else if (log == 1)
                {
                    for (byte i = 0; i < 8; i++)
                    {
                        var childPos = position + OctantPositionIncrement(i, log);

                        if (targetBounds.ContainsExclusive(childPos) && node.HasChild(i))
                        {
                            results[childPos - min] = true;
                        }
                    }
                }
                else
                {
                    var childLog = (byte)(log - 1);
                    var childSize = 1 << childLog;

                    for (byte i = 0; i < 8; i++)
                    {
                        if (!node.HasChild(i))
                        {
                            continue;
                        }

                        var childPos = position + OctantPositionIncrement(i, log);
                        var childBounds = new BoundingBox3di(childPos, childPos + childSize);

                        if (childBounds.Intersects(targetBounds))
                        {
                            stack.Push(new PositionLogTraversalData
                            {
                                Lc = ChildCode(frame.Lc, i),
                                Position = childPos,
                                Log = childLog
                            });
                        }
                    }
                }
            }
        }
    }

    #endregion

    #region Search

    // Never true for log 0
    public bool Contains(ulong lc) => _nodes.ContainsKey(lc);

    public bool Contains(Vector3di targetPos)
    {
        if (!IsWithinBounds(targetPos))
        {
            return false;
        }

        var lcNode = RootCode;
        var log = Log;

        while (true)
        {
            var node = _nodes[lcNode];

            if (node.IsFilled)
            {
                return true;
            }

            var octant = Descend(ref targetPos, log);

            if (!node.HasChild(octant))
            {
                return false;
            }

            if (log == 1)
            {
                return true;
            }

            lcNode = ChildCode(lcNode, octant);
            --log;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ClosestLeaf(
        BitOctreeNode node,
        byte bit,
        Vector3di nodePosWorld,
        byte log,
        Vector3 targetPosWorld,
        out Vector3di bestPos,
        out byte bestOct)
    {
        var bestCost = float.MaxValue;
        bestPos = default;
        bestOct = default;

        for (byte i = 0; i < 8; i++)
        {
            if (node.ChildBit(i) != bit)
            {
                continue;
            }

            var childPos = nodePosWorld + OctantPositionIncrement(i, log);

            var cost = Vector3.DistanceSquared(
                new Vector3(childPos.X + 0.5f, childPos.Y + 0.5f, childPos.Z + 0.5f),
                targetPosWorld
            );

            if (cost < bestCost)
            {
                bestCost = cost;
                bestPos = childPos;
                bestOct = i;
            }
        }

        Debug.Assert(!bestCost.Equals(float.MaxValue));
    }

    public HashedOctreeNodeInfo? GetClosestVoxel(Vector3di tile)
    {
        _nodeQueryQueue.Enqueue(new NodeInfo
        {
            Lc = RootCode,
            Log = Log,
            Node = _nodes[RootCode],
            Position = Vector3di.Zero
        }, 0);

        var result = GetClosestVoxelCore(tile);

        _nodeQueryQueue.Clear();

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    private HashedOctreeNodeInfo? GetClosestVoxelCore(Vector3di targetTileWorld)
    {
        var targetPosWorld = new Vector3(targetTileWorld.X + 0.5f, targetTileWorld.Y + 0.5f, targetTileWorld.Z + 0.5f);

        while (_nodeQueryQueue.TryDequeue(out var info, out _))
        {
            var node = info.Node;
            var nodePosWorld = info.Position;
            var log = info.Log;

            if (node.IsFilled)
            {
                return new HashedOctreeNodeInfo(info.Lc, node, new OctreeNodeBounds(nodePosWorld, log));
            }

            if (!node.HasChildren)
            {
                continue;
            }

            if (log == 1)
            {
                ClosestLeaf(node, 1, nodePosWorld, log, targetPosWorld, out var bestPos, out var bestOct);

                return new HashedOctreeNodeInfo(
                    ChildCode(info.Lc, bestOct),
                    new BitOctreeNode { IsFilled = true },
                    new OctreeNodeBounds(bestPos, 0)
                );
            }

            var childLog = (byte)(log - 1);

            for (byte i = 0; i < 8; i++)
            {
                if (!node.HasChild(i))
                {
                    continue;
                }

                var childPos = nodePosWorld + OctantPositionIncrement(i, log);
                var childBox = new BoundingBox3di(childPos, childPos + new Vector3di(1 << (log - 1)));
                var lc = ChildCode(info.Lc, i);

                _nodeQueryQueue.Enqueue(new NodeInfo
                {
                    Lc = lc,
                    Node = _nodes[lc],
                    Log = childLog,
                    Position = childPos
                }, childBox.DistanceToSqr(targetPosWorld));
            }
        }

        return null;
    }

    public OctreeNodeBounds? GetClosestEmptyRegion(Vector3di tile)
    {
        _regionQueryQueue.Enqueue(new RegionInfo
        {
            NodeData = new KeyValuePair<ulong, BitOctreeNode>(RootCode, _nodes[RootCode]),
            Position = Vector3di.Zero,
            Log = Log,
        }, 0);

        var result = GetClosestEmptyRegionCore(tile);

        _regionQueryQueue.Clear();

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    private OctreeNodeBounds? GetClosestEmptyRegionCore(Vector3di targetTileWorld)
    {
        var targetPosWorld = new Vector3(targetTileWorld.X + 0.5f, targetTileWorld.Y + 0.5f, targetTileWorld.Z + 0.5f);

        while (_regionQueryQueue.TryDequeue(out var info, out _))
        {
            if (!info.NodeData.HasValue)
            {
                return new OctreeNodeBounds(info.Position, info.Log);
            }

            var nodeData = info.NodeData.Value;
            var node = nodeData.Value;

            if (node.IsFilled)
            {
                continue;
            }

            var nodePosWorld = info.Position;
            var log = info.Log;

            if (log == 1)
            {
                ClosestLeaf(node, 0, nodePosWorld, log, targetPosWorld, out var bestPos, out _);

                return new OctreeNodeBounds(bestPos, 0);
            }

            var lcNode = nodeData.Key;
            var childLog = (byte)(log - 1);

            for (byte i = 0; i < 8; i++)
            {
                KeyValuePair<ulong, BitOctreeNode>? childNodeData;

                if (node.HasChild(i))
                {
                    var lc = ChildCode(lcNode, i);
                    childNodeData = new KeyValuePair<ulong, BitOctreeNode>(lc, _nodes[lc]);
                }
                else
                {
                    childNodeData = null;
                }

                var childPos = nodePosWorld + OctantPositionIncrement(i, log);
                var childBox = new BoundingBox3di(childPos, childPos + new Vector3di(1 << (log - 1)));

                _regionQueryQueue.Enqueue(new RegionInfo
                {
                    NodeData = childNodeData,
                    Position = childPos,
                    Log = childLog
                }, childBox.DistanceToSqr(targetPosWorld));
            }
        }

        return null;
    }

    // Does not check hashtable for ancestor
    // Will fail if and only if the ancestor went out of bounds (no ancestor exists)
    private bool TryGetCommonAncestorCode(
        ulong lcSourceNode,
        Base6Direction3d direction,
        out byte sourceNodeLog,
        out ulong lcAncestor,
        out byte ancestorLog,
        out ulong lcPathToAncestor)
    {
        var directionBit = direction.Mask();

        sourceNodeLog = DecodeLog(lcSourceNode);
        lcAncestor = lcSourceNode;
        lcPathToAncestor = 0;
        ancestorLog = sourceNodeLog;

        while (lcAncestor != RootCode)
        {
            var octant = lcAncestor & 7;
            lcPathToAncestor = (lcPathToAncestor << 3) | octant;
            lcAncestor >>= 3;
            ++ancestorLog;

            if ((DirectionMaskByOctant[octant] & directionBit) == 0)
            {
                return true;
            }
        }

        // Root was reached, so no go.

        return false;
    }

    public bool TryGetTopmostNeighbor(
        ulong lcSourceNode,
        Base6Direction3d direction,
        out ulong lcNeighborNode,
        out byte neighborLog)
    {
        if (!TryGetCommonAncestorCode(
                lcSourceNode,
                direction,
                out var sourceLog,
                out var lcAncestor,
                out neighborLog,
                out var lcAncestorPath)
           )
        {
            lcNeighborNode = NullCode;
            return false;
        }

        if (!_nodes.TryGetValue(lcAncestor, out var node))
        {
            goto fail;
        }

        var reflections = ReflectionsByDirection[(int)direction];

        lcNeighborNode = lcAncestor;

        while (neighborLog > sourceLog && !node.IsFilled)
        {
            var octant = reflections[lcAncestorPath & 7];

            if (!node.HasChild(octant))
            {
                goto fail;
            }

            lcNeighborNode = ChildCode(lcNeighborNode, octant);
            lcAncestorPath >>= 3;
            --neighborLog;

            if (neighborLog == 0)
            {
                break;
            }

            node = _nodes[lcNeighborNode];
        }

        return true;

    fail:
        lcNeighborNode = NullCode;
        return false;
    }

    public IEnumerable<ulong> GetFaceNeighbors(ulong lcNode, Base6Direction3d direction)
    {
        if (!TryGetTopmostNeighbor(
                lcNode,
                direction,
                out var lcTopmostNeighbor,
                out var topmostLog))
        {
            yield break;
        }

        if (topmostLog == 0)
        {
            yield return lcTopmostNeighbor;
            yield break;
        }

        var adjacentOctants = OctantsByDirection[(int)direction.Opposite()];

        using (_logTraversalStackPool.Handle(out var stack))
        {
            stack.Push(new LogTraversalData
            {
                Lc = lcTopmostNeighbor,
                Log = topmostLog
            });

            while (stack.TryPop(out var frame))
            {
                Debug.Assert(frame.Log > 0);

                var lc = frame.Lc;
                var node = _nodes[lc];

                if (node.IsFilled)
                {
                    yield return lc;
                }
                else
                {
                    if (frame.Log == 1)
                    {
                        for (var index = 0; index < adjacentOctants.Length; index++)
                        {
                            var adjacentOctant = adjacentOctants[index];

                            if (node.HasChild(adjacentOctant))
                            {
                                yield return ChildCode(lc, adjacentOctant);
                            }
                        }
                    }
                    else
                    {
                        for (var index = 0; index < adjacentOctants.Length; index++)
                        {
                            var adjacentOctant = adjacentOctants[index];

                            if (node.HasChild(adjacentOctant))
                            {
                                stack.Push(new LogTraversalData
                                {
                                    Lc = ChildCode(lc, adjacentOctant),
                                    Log = (byte)(frame.Log - 1)
                                });
                            }
                        }
                    }
                }
            }
        }
    }

    public IEnumerable<Vector3di> EnumerateFrontierCells(Vector3di target)
    {
        var queue = new PriorityQueue<FrontierEvaluator, double>();

        queue.Enqueue(new TreeEvaluator(new EvaluatorContext(this, target)), 0);

        try
        {
            while (queue.TryPeek(out var evaluator, out var priority))
            {
                var status = evaluator.Update();

                if (!evaluator.Cost.Equals(priority) && status != EvaluationResult.Finished)
                {
                    queue.Dequeue();
                    queue.Enqueue(evaluator, evaluator.Cost);
                }

                switch (status)
                {
                    case EvaluationResult.Finished:
                        queue.Dequeue();
                        evaluator.Dispose();
                        break;
                    case EvaluationResult.Tile:
                        var t = evaluator.Tile;
                        yield return t;
                        break;
                    case EvaluationResult.Evaluator:
                        var next = evaluator.NextEvaluator;
                        queue.Enqueue(next, next.Cost);
                        break;
                    case EvaluationResult.CheckCost:
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }
        }
        finally
        {
            foreach (var (evaluator, _) in queue.UnorderedItems)
            {
                evaluator.Dispose();
            }
        }
    }

    private enum EvaluationResult
    {
        Finished,
        Tile,
        Evaluator,
        CheckCost
    }

    private sealed class EvaluatorContext
    {
        public readonly BitOctree Octree;
        public readonly Vector3di QueryPos;

        public readonly HashSet<Vector3di> Visited = new();

        public EvaluatorContext(BitOctree octree, Vector3di queryPos)
        {
            Octree = octree;
            QueryPos = queryPos;
        }
    }

    private abstract class FrontierEvaluator : IDisposable
    {
        protected readonly EvaluatorContext Context;

        private IEnumerator<EvaluationResult>? _enumerator;

        public FrontierEvaluator(EvaluatorContext context)
        {
            Context = context;
        }

        public double GetCost(Vector3di position)
        {
            return Vector3di.DistanceSqr(position, Context.QueryPos);
        }

        public double GetCost(BoundingBox3di box)
        {
            return Vector3di.DistanceSqr(Vector3di.ClampExclusive(Context.QueryPos, box.Min, box.Max), Context.QueryPos);
        }

        public EvaluationResult UpdateCost(double cost)
        {
            Cost = cost;
            return EvaluationResult.CheckCost;
        }

        public EvaluationResult UpdateCost(Vector3di position)
        {
            Cost = GetCost(position);
            return EvaluationResult.CheckCost;
        }

        public EvaluationResult UpdateCost(BoundingBox3di box)
        {
            Cost = GetCost(box);
            return EvaluationResult.CheckCost;
        }

        public double Cost;

        protected abstract IEnumerator<EvaluationResult> CreateEnumerator();

        public void Dispose()
        {
            _enumerator?.Dispose();
        }

        public EvaluationResult Update()
        {
            _enumerator ??= CreateEnumerator();

            return _enumerator.MoveNext()
                ? _enumerator.Current
                : EvaluationResult.Finished;
        }

        private Vector3di? _tile;
        private FrontierEvaluator? _evaluator;

        protected EvaluationResult SetTile(Vector3di tile)
        {
            if (_tile.HasValue)
            {
                throw new InvalidOperationException("Invalid set tile");
            }

            _tile = tile;
            return EvaluationResult.Tile;
        }

        protected EvaluationResult SetEvaluator(FrontierEvaluator evaluator)
        {
            if (_evaluator != null)
            {
                throw new InvalidOperationException("Invalid set evaluator");
            }

            _evaluator = evaluator;
            return EvaluationResult.Evaluator;
        }

        public Vector3di Tile
        {
            get
            {
                if (_enumerator == null || _enumerator.Current != EvaluationResult.Tile)
                {
                    throw new InvalidOperationException("Invalid tile access");
                }

                if (!_tile.HasValue)
                {
                    throw new InvalidOperationException("Multiple tile access");
                }

                var tile = _tile.Value;
                _tile = null;
                return tile;
            }
        }

        public FrontierEvaluator NextEvaluator
        {
            get
            {
                if (_enumerator == null || _enumerator.Current != EvaluationResult.Evaluator)
                {
                    throw new InvalidOperationException("Invalid evaluator access");
                }

                if (_evaluator == null)
                {
                    throw new InvalidOperationException("Multiple evaluator access");
                }

                var evaluator = _evaluator;
                _evaluator = null;
                return evaluator;
            }
        }
    }

    private sealed class TreeEvaluator : FrontierEvaluator
    {
        public TreeEvaluator(EvaluatorContext context) : base(context)
        {
            Cost = 0;
        }

        protected override IEnumerator<EvaluationResult> CreateEnumerator()
        {
            var queue = new PriorityQueue<LogTraversalData, double>();

            queue.Enqueue(new LogTraversalData
            {
                Lc = RootCode,
                Log = Context.Octree.Log
            }, 0);

            while (queue.TryDequeue(out var frame, out var cost))
            {
                Cost = cost;

                var lc = frame.Lc;
                var node = Context.Octree._nodes[lc];

                if (node.IsFilled)
                {
                    yield return SetEvaluator(new LeafNodeEvaluator(Context, lc));
                }
                else
                {
                    if (frame.Log == 1)
                    {
                        for (byte i = 0; i < 8; i++)
                        {
                            if (node.HasChild(i))
                            {
                                yield return SetEvaluator(new LeafNodeEvaluator(Context, ChildCode(lc, i)));
                            }
                        }
                    }
                    else
                    {
                        var childLog = (byte)(frame.Log - 1);

                        for (byte i = 0; i < 8; i++)
                        {
                            if (node.HasChild(i))
                            {
                                var childCode = ChildCode(lc, i);
                                var childBounds = Context.Octree.NodeBounds(childCode);

                                queue.Enqueue(new LogTraversalData
                                {
                                    Lc = childCode,
                                    Log = childLog
                                }, GetCost(childBounds));
                            }
                        }
                    }
                }
            }
        }
    }

    private sealed class RangeEvaluator : FrontierEvaluator
    {
        private readonly BoundingBox3di _range;

        public RangeEvaluator(EvaluatorContext context, BoundingBox3di range) : base(context)
        {
            _range = range;
            UpdateCost(range);
        }

        protected override IEnumerator<EvaluationResult> CreateEnumerator()
        {
            if (_range.Volume == 1)
            {
                yield return SetTile(_range.Min);
                yield break;
            }

            var queue = new PriorityQueue<Vector3di, double>();
            queue.Enqueue(Vector3di.ClampExclusive(Context.QueryPos, _range.Min, _range.Max), Cost);

            while (queue.TryDequeue(out var front, out var cost))
            {
                if (!Context.Visited.Add(front))
                {
                    continue;
                }

                yield return UpdateCost(cost);
                yield return SetTile(front);

                for (var i = 0; i < 6; i++)
                {
                    var neighbor = front + ((Base6Direction3d)i).Step();

                    if (_range.ContainsExclusive(neighbor))
                    {
                        queue.Enqueue(neighbor, GetCost(neighbor));
                    }
                }
            }
        }
    }

    private sealed class LeafNodeEvaluator : FrontierEvaluator
    {
        private readonly ulong _lcFilledNode;
        private readonly BoundingBox3di _nodeBounds;
        private readonly KeyValuePair<Base6Direction3d, double>[] _facesOrdered;

        public LeafNodeEvaluator(EvaluatorContext context, ulong lcFilledNode) : base(context)
        {
            _lcFilledNode = lcFilledNode;
            _nodeBounds = Context.Octree.NodeBounds(_lcFilledNode);
            _facesOrdered = new KeyValuePair<Base6Direction3d, double>[6];

            for (var i = 0; i < 6; i++)
            {
                var direction = (Base6Direction3d)i;
                var cost = GetCost(_nodeBounds.InnerFace(direction));
                _facesOrdered[i] = new KeyValuePair<Base6Direction3d, double>(direction, cost);
            }

            Array.Sort(_facesOrdered, CompareValue);

            Cost = _facesOrdered[0].Value;
        }

        private static int CompareValue<T>(KeyValuePair<T, double> a, KeyValuePair<T, double> b)
        {
            return a.Value.CompareTo(b.Value);
        }

        private EvaluationResult SetRange(BoundingBox3di box)
        {
            return SetEvaluator(new RangeEvaluator(Context, box));
        }

        protected override IEnumerator<EvaluationResult> CreateEnumerator()
        {
            var isLeaf = _nodeBounds.Volume == 1;

            var orderedChildren = new KeyValuePair<byte, double>[4];

            for (var iFace = 0; iFace < 6; iFace++)
            {
                var dir = _facesOrdered[iFace].Key;

                yield return UpdateCost(_facesOrdered[iFace].Value);

                if (!Context.Octree.TryGetTopmostNeighbor(_lcFilledNode, dir, out var lcNeighborNode, out var neighborLog))
                {
                    if (isLeaf)
                    {
                        goto yieldSelf;
                    }

                    yield return SetRange(_nodeBounds.InnerFace(dir));
                    continue;
                }

                if (neighborLog == 0)
                {
                    continue;
                }

                var adjacentOctants = OctantsByDirection[(int)dir.Opposite()];
                var faceNormal = dir.Step();

                using (Context.Octree._positionLogTraversalStackPool.Handle(out var stack))
                {
                    stack.Push(new PositionLogTraversalData
                    {
                        Lc = lcNeighborNode,
                        Position = Context.Octree.DecodePosition(lcNeighborNode),
                        Log = neighborLog
                    });

                    while (stack.TryPop(out var frame))
                    {
                        Debug.Assert(frame.Log > 0);

                        var lc = frame.Lc;
                        var node = Context.Octree._nodes[lc];

                        if (node.IsFilled)
                        {
                            continue;
                        }

                        var pos = frame.Position;

                        if (frame.Log == 1)
                        {
                            for (var iOctant = 0; iOctant < 4; iOctant++)
                            {
                                var octant = adjacentOctants[iOctant];
                                var position = pos + OctantPositionIncrement(octant, frame.Log) - faceNormal;
                                orderedChildren[iOctant] = new KeyValuePair<byte, double>(octant, GetCost(position));
                            }

                            Array.Sort(orderedChildren, CompareValue);

                            for (var i = 0; i < orderedChildren.Length; i++)
                            {
                                var childInfo = orderedChildren[i];

                                if (!node.HasChild(childInfo.Key))
                                {
                                    if (isLeaf)
                                    {
                                        goto yieldSelf;
                                    }

                                    yield return UpdateCost(childInfo.Value);
                                    yield return SetTile(pos + OctantPositionIncrement(childInfo.Key, frame.Log) - faceNormal);
                                }
                            }
                        }
                        else
                        {
                            var childLog = (byte)(frame.Log - 1);
                            var childSize = 1 << childLog;

                            for (var j = 0; j < 4; j++)
                            {
                                var octant = adjacentOctants[j];
                                var position = pos + OctantPositionIncrement(octant, frame.Log);
                                orderedChildren[j] = new KeyValuePair<byte, double>(octant, GetCost(new BoundingBox3di(position, position + childSize).SliceNormal(dir)));
                            }

                            Array.Sort(orderedChildren, CompareValue);

                            for (var index = 0; index < orderedChildren.Length; index++)
                            {
                                var childInfo = orderedChildren[index];
                                var position = pos + OctantPositionIncrement(childInfo.Key, frame.Log);

                                if (node.HasChild(childInfo.Key))
                                {
                                    stack.Push(new PositionLogTraversalData
                                    {
                                        Lc = ChildCode(lc, childInfo.Key),
                                        Position = position,
                                        Log = childLog
                                    });
                                }
                                else
                                {
                                    if (isLeaf)
                                    {
                                        goto yieldSelf;
                                    }

                                    yield return SetRange(new BoundingBox3di(position, position + childSize).SliceNormal(dir));
                                }
                            }
                        }
                    }
                }
            }

            yield break;

            yieldSelf:
            yield return SetTile(_nodeBounds.Min);
        }
    }

    #endregion

    public void Clear(bool fill = false)
    {
        _nodes.Clear();
        _nodes.Add(RootCode, new BitOctreeNode { IsFilled = fill });
        ++Version;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3di DecodePosition(ulong lc)
    {
        var lzm = 63 - BitOperations.LeadingZeroCount(lc);
        var cb = lc & ~(1UL << lzm);
        return MortonCode3D.Decode(cb << (Log - lzm / 3) * 3);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public byte DecodeLog(ulong lc)
    {
        var lzm = 63 - BitOperations.LeadingZeroCount(lc);
        return (byte)(Log - lzm / 3);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void DecodePositionAndLog(ulong lc, out Vector3di position, out byte log)
    {
        var lzm = 63 - BitOperations.LeadingZeroCount(lc);
        log = (byte)(Log - lzm / 3);
        var cb = lc & ~(1UL << lzm);
        position = MortonCode3D.Decode(cb << log * 3);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public BoundingBox3di NodeBounds(ulong lc)
    {
        DecodePositionAndLog(lc, out var position, out var log);
        return NodeBounds(position, log);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static BoundingBox3di NodeBounds(Vector3di position, byte log)
    {
        var size = 1 << log;
        return new BoundingBox3di(position, position + size);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong ChildCode(ulong lcParent, byte octant)
    {
        Debug.Assert(lcParent != 0);
        Debug.Assert(BitOperations.LeadingZeroCount(lcParent) >= 3);
        Debug.Assert(octant < 8);
        return (lcParent << 3) | octant;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static byte Descend(ref Vector3di position, byte parentLog)
    {
        var lsz = 1 << (parentLog - 1);
        var cell = position / lsz;

        Debug.Assert(cell.X is 0 or 1 && cell.Y is 0 or 1 && cell.Z is 0 or 1);

        position -= cell * lsz;

        return (byte)(cell.X | cell.Y << 1 | cell.Z << 2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3di OctantPositionIncrement(byte octant, byte parentLog)
    {
        var k = 1 << (parentLog - 1);
        return new Vector3di((octant & 1) * k, ((octant >> 1) & 1) * k, ((octant >> 2) & 1) * k);
    }

    private readonly struct DescentData
    {
        public required ulong Lc { get; init; }
        public required byte Log { get; init; }
        public required byte Octant { get; init; }
    }

    private readonly struct PositionLogTraversalData
    {
        public required ulong Lc { get; init; }
        public required Vector3di Position { get; init; }
        public required byte Log { get; init; }
    }

    private readonly struct LogTraversalData
    {
        public required ulong Lc { get; init; }
        public required byte Log { get; init; }
    }

    private readonly struct NodeInfo
    {
        public ulong Lc { get; init; }
        public BitOctreeNode Node { get; init; }
        public Vector3di Position { get; init; }
        public byte Log { get; init; }
    }

    private readonly struct RegionInfo
    {
        public KeyValuePair<ulong, BitOctreeNode>? NodeData { get; init; }
        public Vector3di Position { get; init; }
        public byte Log { get; init; }
    }
}

public static class MortonCode3D
{
    private const ulong A = 0x41041041041041;
    private const ulong B = 0x208208208208208;
    private const ulong C = 0x1000000000000000;
    private const ulong D = 0x3003003003003;
    private const ulong E = 0x1C00C00C00C00C0;
    private const ulong F = 0xF00000F;
    private const ulong G = 0xF00000F000;
    private const ulong H = 0x1F000000000000;
    private const ulong I = 0xFF;
    private const ulong J = 0x1FFF000000;

    public static Vector3di Decode(ulong x) => new(DeInterleave(x), DeInterleave(x >> 1), DeInterleave(x >> 2));

    public static ulong Encode(Vector3di v) => Interleave((ulong)v.X) | Interleave((ulong)v.Y) << 1 | Interleave((ulong)v.Z) << 2;

    private static ulong Interleave(ulong x)
    {
        x = x & I | x << 16 & J;
        x = x & F | x << 8 & G | x << 16 & H;
        x = x & D | x << 4 & E;
        x = x & A | x << 2 & B | x << 4 & C;

        return x;
    }

    private static int DeInterleave(ulong x)
    {
        x = x & A | (x & B) >> 2 | (x & C) >> 4;
        x = x & D | (x & E) >> 4;
        x = x & F | (x & G) >> 8 | (x & H) >> 16;
        x = x & I | (x & J) >> 16;

        return (int)x;
    }
}