using System.Diagnostics;
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

    public HashedBitOctree(int log)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(log, 1, nameof(log));
        ArgumentOutOfRangeException.ThrowIfGreaterThan(log, 20, nameof(log));

        Log = (byte)log;

        _nodes.Add(RootCode, new HashedBitOctreeNode());
    }

    public byte Log { get; }
    public int EdgeSize => 1 << Log;
    public Vector3di Min => Vector3di.Zero;
    public Vector3di Max => new(EdgeSize);
    public BoundingBox3di Bounds => new(Min, Max);
    public int NodeCount => _nodes.Count;

    public bool IsWithinBounds(Vector3di point) => Bounds.Contains(point);

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
        }

        _insertRemoveStack.Clear();

        return result;
    }

    private bool InsertCore(ulong lcNode, byte log, Vector3di targetPos)
    {
        while (true)
        {
            var node = _nodes[lcNode];

            if (node.IsFilled)
            {
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
                _nodes.Add(lcChild, new HashedBitOctreeNode());
            }

            lcNode = lcChild;
            --log;
        }
    }

    private void Fill()
    {
        while (_insertRemoveStack.TryPop(out var frame))
        {
            var lcNode = frame.Lc;
            var node = _nodes[lcNode];

            Debug.Assert(!node.IsFilled);

            if (node.ChildMask != byte.MaxValue)
            {
                break;
            }

            if (frame.Log > 1)
            {
                for (byte i = 0; i < 8; i++)
                {
                    Debug.Assert(node.HasChild(i));
#if DEBUG
                    Debug.Assert(_nodes.Remove(ChildCode(lcNode, i)));
#else
                    _nodes.Remove(ChildCode(lcNode, i));
#endif
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
        }

        _insertRemoveStack.Clear();

        return result;
    }

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
            
            var node = frame.Lc.HasValue
                ? _nodes[frame.Lc.Value] 
                : new HashedBitOctreeNode { IsFilled = true };

            if (!traverse(node, frame.Position, frame.Log))
            {
                break;
            }

            if (frame.Log == 0)
            {
                continue;
            }

            for (byte i = 0; i < 8; i++)
            {
                if (node.HasChild(i))
                {
                    _traverseStack.Push(new TraverseFrame
                    {
                        Lc = frame.Log == 1 ? null : ChildCode(frame.Lc!.Value, i),
                        Position = frame.Position + ChildOffset(i, frame.Log),
                        Log = (byte)(frame.Log - 1)
                    });
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
                return node.HasChild(octant);
            }

            lcNode = ChildCode(lcNode, octant);
            --log;
        }
    }

    #endregion

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong ChildCode(ulong lc, byte child) => (lc << 3) | child;

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
    private static Vector3di ChildOffset(byte child, byte log)
    {
        var k = (1 << (log - 1));
       
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
        public required ulong? Lc { get; init; }
        public required Vector3di Position { get; init; }
        public required byte Log { get; init; }
    }
}