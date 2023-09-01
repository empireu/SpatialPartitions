using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Common;

namespace OctTreeVisualization;

public enum Octant : byte
{
    /// <summary>
    ///     Bottom left front
    /// </summary>
    BLF = 0,
    /// <summary>
    ///     Bottom right front
    /// </summary>
    BRF = 1,
    /// <summary>
    ///     Bottom left back
    /// </summary>
    BLB = 2,
    /// <summary>
    ///     Bottom right back
    /// </summary>
    BRB = 3,
    /// <summary>
    ///     Top left front
    /// </summary>
    TLF = 4,
    /// <summary>
    ///     Top right front
    /// </summary>
    TRF = 5,
    /// <summary>
    ///     Top left back
    /// </summary>
    TLB = 6,
    /// <summary>
    ///     Top right back
    /// </summary>
    TRB = 7
}

public struct HashedBitOctreeNode
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
    public HashedOctreeNodeInfo(ulong? locationCode, HashedBitOctreeNode node, OctreeNodeBounds bounds)
    {
        LocationCode = locationCode;
        Node = node;
        Bounds = bounds;
    }

    public ulong? LocationCode { get; }

    public HashedBitOctreeNode Node { get; }

    // Log can be inferred from location code in constant time (if you have clz instructions available),
    // but position is O(h) so we may as well store both
    public OctreeNodeBounds Bounds { get; }
}

// Bit Octree, as in, 2 states per voxel.

public sealed class HashedBitOctree
{
    public delegate bool TraverseDelegate(HashedBitOctreeNode node, Vector3di position, byte log);

    public const ulong RootCode = 1;
    public const ulong NullCode = 0;

    private readonly Dictionary<ulong, HashedBitOctreeNode> _nodes = new();
   
    private readonly Stack<InsertRemoveFrame> _insertRemoveStack = new();
    private readonly Stack<TraverseFrame> _traverseStack = new();
    private readonly PriorityQueue<NodeInfo, float> _nodeQueryQueue = new();
    private readonly PriorityQueue<RegionInfo, float> _regionQueryQueue = new();

    public HashedBitOctree(int log)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(log, 1, nameof(log));
        ArgumentOutOfRangeException.ThrowIfGreaterThan(log, 21, nameof(log));

        Log = (byte)log;

        _nodes.Add(RootCode, new HashedBitOctreeNode());
    }

    public byte Log { get; }
    public int EdgeSize => 1 << Log;
    public Vector3di Min => Vector3di.Zero;
    public Vector3di Max => new(EdgeSize);
    public BoundingBox3di Bounds => new(Min, Max - 1);
    public int NodeCount => _nodes.Count;

    public int Version { get; private set; }
    
    public bool IsWithinBounds(Vector3di point) => Bounds.Contains(point);

    private void AddNode(ulong code, HashedBitOctreeNode node)
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

        var result = InsertCore(RootCode, Log, tile);

        if (result)
        {
            Fill();
            ++Version;
        }

        _insertRemoveStack.Clear();

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    private bool InsertCore(ulong lcNode, byte log, Vector3di targetPos)
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

            var octant = DescendOctant(ref targetPos, log);

            _insertRemoveStack.Push(new InsertRemoveFrame
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
                AddNode(lcChild, new HashedBitOctreeNode());
            }
            
            lcNode = lcChild;
            --log;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    private void Fill()
    {
        while (_insertRemoveStack.TryPop(out var frame))
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

        var result = RemoveCore(RootCode, Log, tile);

        if (result)
        {
            Trim();
            ++Version;
        }

        _insertRemoveStack.Clear();

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    private bool RemoveCore(ulong lcNode, byte log, Vector3di targetPos)
    {
        while (true)
        {
            var node = _nodes[lcNode];
            var octant = DescendOctant(ref targetPos, log);

            _insertRemoveStack.Push(new InsertRemoveFrame
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
                        _nodes.Add(ChildCode(lcNode, i), new HashedBitOctreeNode
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

    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    private void Trim()
    {
        while (_insertRemoveStack.TryPop(out var frame) && _insertRemoveStack.Count > 0)
        {
            var node = _nodes[frame.Lc];

            if (node.IsFilled || node.HasChildren)
            {
                return;
            }

            var parentFrame = _insertRemoveStack.Peek();
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
        _traverseStack.Push(new TraverseFrame
        {
            Lc = RootCode,
            Position = Vector3di.Zero,
            Log = Log
        });

        while (_traverseStack.Count > 0)
        {
            var frame = _traverseStack.Pop();

            var lcNode = frame.Lc;
            var node = _nodes[lcNode];

            if (!traverse(node, frame.Position, frame.Log))
            {
                goto end;
            }

            if (frame.Log == 1)
            {
                for (byte i = 0; i < 8; i++)
                {
                    if (node.HasChild(i))
                    {
                        if (!traverse(new HashedBitOctreeNode { IsFilled = true }, frame.Position + ChildOffset(i, frame.Log), 0))
                        {
                            goto end;
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
                        _traverseStack.Push(new TraverseFrame
                        {
                            Lc = ChildCode(lcNode, i),
                            Position = frame.Position + ChildOffset(i, frame.Log),
                            Log = (byte)(frame.Log - 1)
                        });
                    }
                }
            }

           
        }

        end:

        _traverseStack.Clear();
    }

    // Assumes target is false by default
    public void ReadRange(Vector3di min, IGrid3d<bool> results)
    {
        _traverseStack.Push(new TraverseFrame
        {
            Lc = RootCode,
            Position = Vector3di.Zero,
            Log = Log
        });

        var targetBounds = new BoundingBox3di(min, min + results.Size);

        while (_traverseStack.TryPop(out var frame))
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
                    var childPos = position + ChildOffset(i, log);

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

                    var childPos = position + ChildOffset(i, log);
                    var childBounds = new BoundingBox3di(childPos, childPos + childSize);
                  
                    if (childBounds.Intersects(targetBounds))
                    {
                        _traverseStack.Push(new TraverseFrame
                        {
                            Lc = ChildCode(frame.Lc, i),
                            Position = childPos,
                            Log = childLog
                        });
                    }
                }
            }
        }

        _traverseStack.Clear();
    }

    #endregion

    #region Search

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

            var octant = DescendOctant(ref targetPos, log);

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
        HashedBitOctreeNode node,
        byte bit,
        Vector3di nodePosWorld,
        byte log,
        Vector3 targetPosWorld,
        out Vector3di bestPos)
    {
        var bestCost = float.MaxValue;
        bestPos = default;

        for (byte i = 0; i < 8; i++)
        {
            if (node.ChildBit(i) != bit)
            {
                continue;
            }

            var childPos = nodePosWorld + ChildOffset(i, log);

            var cost = Vector3.DistanceSquared(
                new Vector3(childPos.X + 0.5f, childPos.Y + 0.5f, childPos.Z + 0.5f),
                targetPosWorld
            );

            if (cost < bestCost)
            {
                bestCost = cost;
                bestPos = childPos;
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
                ClosestLeaf(node, 1, nodePosWorld, log, targetPosWorld, out var bestPos);

                return new HashedOctreeNodeInfo(
                    null, 
                    new HashedBitOctreeNode { IsFilled = true }, 
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

                var childPos = nodePosWorld + ChildOffset(i, log);
                var childBox = new BoundingBox3di(childPos, childPos + new Vector3di(1 << (log - 1)));
                var lc = ChildCode(info.Lc!.Value, i);

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
            NodeData = new KeyValuePair<ulong, HashedBitOctreeNode>(RootCode, _nodes[RootCode]),
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
                ClosestLeaf(node, 0, nodePosWorld, log, targetPosWorld, out var bestPos);

                return new OctreeNodeBounds(bestPos, 0);
            }

            var lcNode = nodeData.Key;
            var childLog = (byte)(log - 1);

            for (byte i = 0; i < 8; i++)
            {
                KeyValuePair<ulong, HashedBitOctreeNode>? childNodeData;

                if (node.HasChild(i))
                {
                    var lc = ChildCode(lcNode, i);
                    childNodeData = new KeyValuePair<ulong, HashedBitOctreeNode>(lc, _nodes[lc]);
                }
                else
                {
                    childNodeData = null;
                }

                var childPos = nodePosWorld + ChildOffset(i, log);
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

    #endregion

    public void Clear(bool fill = false)
    {
        _nodes.Clear();
        _nodes.Add(RootCode, new HashedBitOctreeNode { IsFilled = fill });
        ++Version;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong ChildCode(ulong lc, byte child)
    {
        Debug.Assert(lc != 0);
        Debug.Assert(BitOperations.LeadingZeroCount(lc) >= 3);
        Debug.Assert(child < 8);

        return (lc << 3) | child;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static byte DescendOctant(ref Vector3di position, byte parentLog)
    {
        var lsz = 1 << (parentLog - 1);
        var cell = position / lsz;

        Debug.Assert(cell.X is 0 or 1 && cell.Y is 0 or 1 && cell.Z is 0 or 1);

        position -= cell * lsz;

        return (byte)(cell.X | cell.Y << 1 | cell.Z << 2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3di ChildOffset(byte child, byte parentLog)
    {
        var k = (1 << (parentLog - 1));
       
        return new Vector3di(
            (child & 1) * k, 
            ((child >> 1) & 1) * k,
            ((child >> 2) & 1) * k
        );
    }

    private readonly struct InsertRemoveFrame
    {
        public required ulong Lc { get; init; }
        public required byte Log { get; init; }
        public required byte Octant { get; init; }
    }

    private readonly struct TraverseFrame
    {
        public required ulong Lc { get; init; }
        public required Vector3di Position { get; init; }
        public required byte Log { get; init; }
    }

    private readonly struct NodeInfo
    {
        public ulong? Lc { get; init; }
        public HashedBitOctreeNode Node { get; init; }
        public Vector3di Position { get; init; }
        public byte Log { get; init; }
    }

    private readonly struct RegionInfo
    {
        public KeyValuePair<ulong, HashedBitOctreeNode>?  NodeData { get; init; }
        public Vector3di Position { get; init; }
        public byte Log { get; init; }
    }
}