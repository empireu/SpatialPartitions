using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Common;
using Rectangle = System.Drawing.Rectangle;

namespace QuadTreeVisualization;

public enum Quadrant : byte
{
    // Do not change
    TopLeft = 0,
    TopRight = 1,
    BottomLeft = 2,
    BottomRight = 3
}

public sealed class BitQuadTree
{
    public Vector2di Position { get; }

    private readonly byte _log;

    private BitQuadTree? _bl;
    private BitQuadTree? _br;
    private BitQuadTree? _tl;
    private BitQuadTree? _tr;

    public BitQuadTree(Vector2di position, int size)
    {
        if (size <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(size), $"Quadtree size cannot be {size}");
        }

        Position = position;

        _log = (byte)Math.Log(NextPow2(size), 2);
    }

    private BitQuadTree(Vector2di position, byte log)
    {
        Position = position;
        _log = log;
    }

    public bool IsFilled { get; private set; }

    public ushort Size => (ushort)(1 << _log);

    public Rectangle NodeRectangle => new(Position.X, Position.Y, Size, Size);

    public bool HasChildren
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => !IsFilled && (_bl != null || _br != null || _tl != null || _tr != null);
    }

    public bool HasTiles
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => IsFilled || (_bl != null || _br != null || _tl != null || _tr != null);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public BitQuadTree? GetChild(Quadrant quadrant) => quadrant switch
    {
        Quadrant.BottomLeft => _bl,
        Quadrant.BottomRight => _br,
        Quadrant.TopLeft => _tl,
        Quadrant.TopRight => _tr,
        _ => throw new ArgumentOutOfRangeException(nameof(quadrant), quadrant, null)
    };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private BitQuadTree? SetChild(Quadrant quadrant, BitQuadTree? child) => quadrant switch
    {
        Quadrant.BottomLeft => _bl = child,
        Quadrant.BottomRight => _br = child,
        Quadrant.TopLeft => _tl = child,
        Quadrant.TopRight => _tr = child,
        _ => throw new ArgumentOutOfRangeException(nameof(quadrant), quadrant, null)
    };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public BitQuadTree? GetChild(Vector2di position) => GetChild(GetQuadrant(position));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Quadrant GetQuadrant(Vector2di position)
    {
        var isLeft = position.X < Position.X + Size / 2;
        var isBottom = position.Y < Position.Y + Size / 2;

        if (isBottom)
        {
            return isLeft ? Quadrant.BottomLeft : Quadrant.BottomRight;
        }

        return isLeft ? Quadrant.TopLeft : Quadrant.TopRight;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private BitQuadTree CreateNode(Quadrant quadrant)
    {
        var x = Position.X;
        var y = Position.Y;

        var childSize = (ushort)(Size / 2);
        var childSizeLog = (byte)(_log - 1);

        return quadrant switch
        {
            Quadrant.BottomLeft => _bl = new BitQuadTree(new Vector2di(x, y), childSizeLog),
            Quadrant.BottomRight => _br = new BitQuadTree(new Vector2di(x + childSize, y), childSizeLog),
            Quadrant.TopLeft => _tl = new BitQuadTree(new Vector2di(x, y + childSize), childSizeLog),
            Quadrant.TopRight => _tr = new BitQuadTree(new Vector2di(x + childSize, y + childSize), childSizeLog),
            _ => throw new ArgumentOutOfRangeException(nameof(quadrant), quadrant, null)
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void KillChildren()
    {
        _bl = null;
        _br = null;
        _tl = null;
        _tr = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private BitQuadTree GetOrCreateChild(Vector2di position)
    {
        var subNodeIndex = GetQuadrant(position);
        var subNode = GetChild(subNodeIndex) ?? CreateNode(subNodeIndex);
        return subNode;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Insert(Vector2di tile)
    {
        if (!NodeRectangle.Contains(tile))
        {
            throw new InvalidOperationException("Cannot insert outside of bounds");
        }

        InsertCore(tile, 0);
    }

    private void InsertCore(Vector2di position, byte sizeExp)
    {
        if (IsFilled)
        {
            return;
        }

        if (_log == sizeExp)
        {
            if (IsFilled)
            {
                return;
            }

            KillChildren();
            IsFilled = true;
            return;
        }

        if (_log == 0)
        {
            throw new InvalidOperationException("Tried to insert in leaf node");
        }

        var child = GetOrCreateChild(position);

        child.InsertCore(position, sizeExp);

        Optimize();
    }

    public bool Remove(Vector2di tile)
    {
        if (!NodeRectangle.Contains(tile))
        {
            return false;
        }

        return RemoveCore(tile);
    }

    private bool RemoveCore(Vector2di tile)
    {
        if (IsFilled)
        {
            IsFilled = false;

            for (byte i = 0; i < 4; i++)
            {
                CreateNode((Quadrant)i).IsFilled = true;
            }
        }

        var quadrant = GetQuadrant(tile);
        var child = GetChild(quadrant);

        if (child == null)
        {
            return false;
        }

        if (child._log == 0)
        {
            SetChild(quadrant, null);
            return true;
        }

        var removed = child.RemoveCore(tile);

        if (removed && !child.HasTiles)
        {
            SetChild(quadrant, null);
        }

        return removed;
    }

    private void Optimize()
    {
        for (byte subNodeIndex = 0; subNodeIndex < 4; subNodeIndex++)
        {
            var node = GetChild((Quadrant)subNodeIndex);

            if (node is not { IsFilled: true })
            {
                return;
            }
        }

        KillChildren();

        IsFilled = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector2di position)
    {
        if (!NodeRectangle.Contains(position))
        {
            return false;
        }

        if (!HasChildren)
        {
            return IsFilled;
        }

        var node = GetChild(position);

        return node != null && node.Contains(position);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int NextPow2(int v)
    {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v++;
        return v;
    }
}

public struct QuadTreeNode
{
    private const byte Tl = 1 << 1;
    private const byte Tr = 1 << 2;
    private const byte Bl = 1 << 3;
    private const byte Br = 1 << 4;
    private const byte ChildMaskFull = Tl | Tr | Bl | Br;
    private const ulong IndexMask = uint.MaxValue >> 3;

    private static readonly byte[] ChildCounts;

    static QuadTreeNode()
    {
        ChildCounts = new byte[16];

        for (var i = 0; i < 16; i++)
        {
            var count = 0;

            if ((i & (Tl >> 1)) != 0) count++;
            if ((i & (Tr >> 1)) != 0) count++;
            if ((i & (Bl >> 1)) != 0) count++;
            if ((i & (Br >> 1)) != 0) count++;

            ChildCounts[i] = (byte)count;
        }
    }

    public ulong Data;

    public bool IsFilled
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => (Data & (1 << 0)) != 0;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            if (value) Data |= 1 << 0;
            else Data &= ulong.MaxValue ^ (1 << 0);
        }
    }
    
    public void ClearChildren()
    {
        Data &= ulong.MaxValue ^ ChildMaskFull;
        ChildList = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool HasChild(Quadrant quadrant) => (Data & (uint)(1 << (1 + (byte)quadrant))) != 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ActivateChild(Quadrant quadrant) => Data |= (uint)(1 << (1 + (byte)quadrant));

    public readonly byte ChildMask
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (byte)((Data & ChildMaskFull) >> 1);
    }

    public readonly int ChildCount
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ChildCounts[ChildMask];
    }

    public uint ChildList
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => (uint)((Data >> 5) & IndexMask);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            if (value > IndexMask)
            {
                throw new Exception($"Invalid child index {value}");
            }

            Data &= ulong.MaxValue ^ (IndexMask << 5);
            Data |= ((ulong)value) << 5;
        }
    }

    public uint Sibling
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => (uint)((Data >> 34) & IndexMask);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            if (value > IndexMask)
            {
                throw new Exception($"Invalid child index {value}");
            }

            Data &= ulong.MaxValue ^ (IndexMask << 34);
            Data |= ((ulong)value) << 34;
        }
    }
}

public sealed class SPQ<T> where T : struct
{
    public delegate bool TraverseDelegate(int index, Vector2di position, byte log, in QuadTreeNode node);

    private readonly IEqualityComparer<T> _comparer;
    public byte Log { get; }
    public int Size => 1 << Log;

    private QuadTreeNode[] _nodes = new QuadTreeNode[16];

    private T[] _data = new T[16];
    
    private readonly Queue<int> _free = new();

    // Root is reserved
    private int _nodeCount = 1;

    private readonly Stack<int> _path = new();

    public SPQ(byte log, IEqualityComparer<T>? comparer = null)
    {
        _comparer = comparer ?? EqualityComparer<T>.Default;
        Log = log;
    }

    public T GetData(int node) => _data[node];

    private void EnsureCapacity()
    {
        if (_nodes.Length == _nodeCount)
        {
            var size = _nodes.Length * 2;
            Array.Resize(ref _nodes, size);
            Array.Resize(ref _data, size);
        }
    }

    private int Allocate()
    {
        if (_free.TryDequeue(out var node))
        {
            return node;
        }

        EnsureCapacity();

        return _nodeCount++;
    }

    private void Free(int node)
    {
        if (node is 0 or -1)
        {
            throw new InvalidOperationException();
        }

        _nodes[node] = default;
        _free.Enqueue(node);
    }

    private int AllocateList(int count, QuadTreeNode value, T data)
    {
        if (count < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(count));
        }

        int? previous = null;

        var head = 0;

        for (var i = 0; i < count; i++)
        {
            var node = Allocate();

            _nodes[node] = value;
            _data[node] = data;

            if (previous.HasValue)
            {
                _nodes[previous.Value].Sibling = (uint)node;
            }
            else
            {
                head = node;
            }

            previous = node;
        }

        return head;
    }

    public void Traverse(TraverseDelegate traverse)
    {
        TraverseCore(traverse, Vector2di.Zero, 0, Log);
    }

    private bool TraverseCore(TraverseDelegate traverse, Vector2di position, int idx, byte log)
    {
        ref var node = ref _nodes[idx];

        if (!traverse(idx, position, log, in node))
        {
            return false;
        }

        if (node.ChildCount == 0)
        {
            return true;
        }

        var indices = new IndexList(_nodes, node);

        for (var i = 0; i < 4; i++)
        {
            var quad = (Quadrant)i;

            if (indices.Has(quad))
            {
                if (!TraverseCore(traverse, GetChildPos(position, 1 << log, quad), indices[quad], (byte)(log - 1)))
                {
                    return false;
                }
            }
        }

        return true;
    }

    public bool IsWithinBounds(Vector2di position) => position.X >= 0 && position.Y <= 0 && position.X < Size && position.Y > -Size;

    public bool Insert(Vector2di position, T data)
    {
        if (!IsWithinBounds(position))
        {
            throw new ArgumentOutOfRangeException(nameof(position));
        }

        var result = InsertCore(Vector2di.Zero, Log, 0, position, data);

        if (result)
        {
            Console.WriteLine("Solidifying...");
            Solidify();
            Console.WriteLine("-----\n");
        }

        Console.WriteLine($"Insert: {result}");

        _path.Clear();

        return result;
    }

    private void AllocateChild(ref QuadTreeNode parent, Quadrant targetQuad)
    {
        // Allocate new node and reorder list:

        var childCount = parent.ChildCount;

        if (childCount == 4)
        {
            throw new InvalidOperationException();
        }

        Span<(int index, Quadrant q)> list = stackalloc (int index, Quadrant q)[childCount + 1];

        var actualList = new IndexList(_nodes, parent);
        var listIndex = 0;

        for (var quadIndex = 0; quadIndex < 4; quadIndex++)
        {
            var quad = (Quadrant)quadIndex;

            if (actualList.Has(quad))
            {
                list[listIndex++] = (actualList[quad], quad);
            }
        }

        Debug.Assert(listIndex == childCount);

        var newNode = Allocate();

        list[childCount] = (newNode, targetQuad);

        static int Comparer((int, Quadrant) a, (int, Quadrant) b) => ((int)a.Item2).CompareTo((int)b.Item2);
      
        list.Sort(Comparer);

        var previous = list[0];

        parent.ChildList = (uint)previous.index;
        parent.ActivateChild(previous.q);

        for (var i = 1; i < list.Length; i++)
        {
            var current = list[i];
            parent.ActivateChild(current.q);
            _nodes[previous.index].Sibling = (uint)current.index;
            previous = current;
        }
    }

    private bool AreEqual(T a, T b) => _comparer.Equals(a, b);

    private bool InsertCore(Vector2di parentPos, byte parentLog, int parentIdx, Vector2di targetPos, T targetData)
    {
        _path.Push(parentIdx);

        ref var parent = ref _nodes[parentIdx];
        var parentSize = 1 << parentLog;
        var targetQuad = GetQuadrant(parentPos, parentSize, targetPos);

        if (parent.IsFilled)
        {
            var parentData = _data[parentIdx];

            if (AreEqual(parentData, targetData))
            {
                // No need to do anything
                return false;
            }

            // We need to allocate children for this node.
            // We'll allocate children, make them filled with the data stored in the parent,
            // then we'll continue descending into the child that will get the new data.

            parent.IsFilled = false;
            parent.ChildList = (uint)AllocateList(4, new QuadTreeNode { IsFilled = true }, parentData);
            
            return InsertCore(
                GetChildPos(parentPos, parentSize, targetQuad),
                (byte)(parentLog - 1),
                new IndexList(_nodes, parent)[targetQuad],
                targetPos,
                targetData
            );
        }

        // Used to determine if we inserted a node.
        // We need that because we can't rely on comparing the inserted data with the default value, stored in the fresh slot.
        var isNewChild = false;

        if (!parent.HasChild(targetQuad))
        {
            AllocateChild(ref parent, targetQuad);
            isNewChild = true;
        }

        var child = new IndexList(_nodes, parent)[targetQuad];

        Debug.Assert(child != -1);

        if (parentLog == 1)
        {   
            // We'll make sure the size 1 node is filled:
            _nodes[child].IsFilled = true;
            // Also, it is put before checking if the child data is target data, because the default value of the data
            // may be equal to the data being inserted and, if the child is freshly allocated, the flag will not be set already.

            if (AreEqual(_data[child], targetData))
            {
                return isNewChild;
            }

            // If parent log is 1, then the next level will be size 1 nodes.
            // We will set the child node to the new data and start solidifying, starting from this 2x2 node. We'll do that in the Insert method.

            _data[child] = targetData;

            return true;
        }

        return InsertCore(
            GetChildPos(parentPos, parentSize, targetQuad),
            (byte)(parentLog - 1),
            child,
            targetPos,
            targetData
        );
    }

    private void Solidify()
    {
        Console.WriteLine($"Start at {_path.Peek()}");
        while (_path.Count > 0)
        {
            var nodeIdx = _path.Pop();

            ref var node = ref _nodes[nodeIdx];

            if (node.IsFilled)
            {
                Console.WriteLine($"  {nodeIdx} filled!");
                continue;
            }

            if (node.ChildCount != 4)
            {
                Console.WriteLine($"  {nodeIdx} CC!");

                return;
            }

            var indices = new IndexList(_nodes, node);
            var data = _data[indices.TopLeft];

            for (byte i = 0; i < 4; i++)
            {
                var idx = indices[i];

                if (!_nodes[idx].IsFilled || !AreEqual(_data[idx], data))
                {
                    Console.WriteLine($"  {nodeIdx} child is not!");

                    return;
                }
            }

            for (byte i = 0; i < 4; i++)
            {
                // They are filled, so they don't have children and we don't have to free anything else.
                Free(indices[i]);
            }

            Console.WriteLine($"  Solidy {nodeIdx}");

            node.ClearChildren();
            node.IsFilled = true;

            _data[nodeIdx] = data;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Quadrant GetQuadrant(Vector2di parentPos, int parentSize, Vector2di childPos)
    {
        var isLeft = childPos.X < parentPos.X + parentSize / 2;
        var isBottom = childPos.Y <= parentPos.Y - parentSize / 2;

        if (isBottom)
        {
            return isLeft ? Quadrant.BottomLeft : Quadrant.BottomRight;
        }

        return isLeft ? Quadrant.TopLeft : Quadrant.TopRight;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2di GetChildPos(Vector2di parentPos, int parentSize, Quadrant quad)
    {
        return quad switch
        {
            Quadrant.TopLeft => parentPos,
            Quadrant.TopRight => new Vector2di(parentPos.X + parentSize / 2, parentPos.Y),
            Quadrant.BottomLeft => new Vector2di(parentPos.X, parentPos.Y - parentSize / 2),
            Quadrant.BottomRight => new Vector2di(parentPos.X + parentSize / 2, parentPos.Y - parentSize / 2),
            _ => throw new ArgumentOutOfRangeException(nameof(quad), quad, null)
        };
    }

    private readonly ref struct IndexList
    {
        public readonly int TopLeft;
        public readonly int TopRight;
        public readonly int BottomLeft;
        public readonly int BottomRight;

        public int this[Quadrant q] => q switch
        {
            Quadrant.TopLeft => TopLeft,
            Quadrant.TopRight => TopRight,
            Quadrant.BottomLeft => BottomLeft,
            Quadrant.BottomRight => BottomRight,
            _ => throw new ArgumentOutOfRangeException(nameof(q), q, null)
        };

        public int this[int i] => this[(Quadrant)i];

        public bool Has(Quadrant q) => this[q] != -1;

        public IndexList(QuadTreeNode[] storage, QuadTreeNode parent)
        {
            var quadIndex = 0;

            var node = parent.ChildList;

            TopLeft = -1;
            TopRight = -1;
            BottomLeft = -1;
            BottomRight = -1;

            for (var i = 0; i < 4; i++)
            {
                var quad = (Quadrant)quadIndex;
                var target = parent.HasChild(quad) ? ((int)node) : -1;

                switch (quad)
                {
                    case Quadrant.TopLeft:
                        TopLeft = target;
                        break;
                    case Quadrant.TopRight:
                        TopRight = target;
                        break;
                    case Quadrant.BottomLeft:
                        BottomLeft = target;
                        break;
                    case Quadrant.BottomRight:
                        BottomRight = target;
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }

                if (target != -1)
                {
                    node = storage[node].Sibling;
                }

                quadIndex++;
            }
        }
    }
}