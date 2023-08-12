using System.Diagnostics;
using System.Runtime.CompilerServices;
using Common;

namespace QuadTreeVisualization;

public enum Quadrant : byte
{
    // Do not change
    TopLeft = 0,
    TopRight = 1,
    BottomLeft = 2,
    BottomRight = 3
}

public struct LinkedQuadTreeNode
{
    private const byte Tl = 1 << 1;
    private const byte Tr = 1 << 2;
    private const byte Bl = 1 << 3;
    private const byte Br = 1 << 4;
    private const byte ChildMaskFull = Tl | Tr | Bl | Br;
    private const ulong IndexMask = uint.MaxValue >> 3;

    private static readonly byte[] ChildCounts;

    static LinkedQuadTreeNode()
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
   
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void DeactivateChild(Quadrant quadrant) => Data &= ulong.MaxValue ^ (uint)(1 << (1 + (byte)quadrant));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ActivateChildren() => Data |= ChildMaskFull;

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

public sealed class LinkedQuadTree<T> where T : struct
{
    public delegate bool TraverseDelegate(int index, Vector2di position, byte log, in LinkedQuadTreeNode node);

    private readonly IEqualityComparer<T> _comparer;
    private LinkedQuadTreeNode[] _nodes = new LinkedQuadTreeNode[16];
    private T[] _data = new T[16];

    private readonly Queue<int> _free = new();
    private readonly Stack<int> _insertRemovePath = new();
    private readonly Stack<TraverseFrame> _traverseStack = new();

    public LinkedQuadTree(byte log, IEqualityComparer<T>? comparer = null)
    {
        _comparer = comparer ?? EqualityComparer<T>.Default;
        Log = log;
    }

    public byte Log { get; }

    public int NodeCount { get; private set; } = 1; // Root is reserved as 0

    public int Size => 1 << Log;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool AreEqual(T a, T b) => _comparer.Equals(a, b);

    public LinkedQuadTreeNode GetNode(int node) => _nodes[node];

    public T GetData(int node) => _data[node];

    public void Traverse(TraverseDelegate visit)
    {
        _traverseStack.Push(new TraverseFrame
        {
            Position = Vector2di.Zero,
            Index = 0,
            Log = Log
        });

        TraverseCore(visit);
        
        _traverseStack.Clear();
    }

    private void TraverseCore(TraverseDelegate visit)
    {
        while (_traverseStack.Count > 0)
        {
            var frame = _traverseStack.Pop();

            ref var node = ref _nodes[frame.Index];

            if (!visit(frame.Index, frame.Position, frame.Log, in node))
            {
                return;
            }

            if (node.ChildCount == 0)
            {
                Debug.Assert(node.ChildList == 0);
                continue;
            }

            var indices = new IndexList(_nodes, node);

            for (var i = 0; i < 4; i++)
            {
                var quad = (Quadrant)i;

                if (indices.Has(quad))
                {
                    _traverseStack.Push(new TraverseFrame
                    {
                        Position = GetChildPosition(frame.Position, 1 << frame.Log, quad),
                        Index = indices[quad],
                        Log = (byte)(frame.Log - 1)
                    });
                }
            }
        }
    }

    public int Find(Vector2di position)
    {
        if (!IsWithinBounds(position))
        {
            return -1;
        }

        var parentIdx = 0;
        var parentLog = Log;
        var parentPos = Vector2di.Zero;

        while (true)
        {
            ref var parent = ref _nodes[parentIdx];

            var parentSize = 1 << parentLog;

            if (parent.IsFilled)
            {
                return parentIdx;
            }

            if (parent.ChildCount == 0)
            {
                return -1;
            }

            var indices = new IndexList(_nodes, parent);
            var quad = GetChildQuadrant(parentPos, parentSize, position);

            var childIdx = indices[quad];

            if (childIdx == -1)
            {
                return -1;
            }

            parentPos = GetChildPosition(parentPos, parentSize, quad);
            parentIdx = childIdx;
            parentLog = (byte)(parentLog - 1);
        }
    }

    private int Allocate()
    {
        if (_free.TryDequeue(out var node))
        {
            NodeCount++;
            return node;
        }

        if (_nodes.Length == NodeCount)
        {
            var size = _nodes.Length * 2;
            Array.Resize(ref _nodes, size);
            Array.Resize(ref _data, size);
        }

        return NodeCount++;
    }
   
    private int AllocateList(int count, LinkedQuadTreeNode value, T data)
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

    private int AllocateChild(int parentIdx, Quadrant targetQuad)
    {
        var parent = _nodes[parentIdx];
        // Allocate new node and reorder list:

        Debug.Assert(!parent.HasChild(targetQuad));

        var childCount = parent.ChildCount;

        Debug.Assert(childCount < 4);

        Span<(int index, Quadrant q)> list = stackalloc (int index, Quadrant q)[childCount + 1];

        var actualList = new IndexList(_nodes, parent);
        var listIndex = 0;

        for (var quadIndex = 0; quadIndex < 4; quadIndex++)
        {
            var quad = (Quadrant)quadIndex;
            var idx = actualList[quad];

            if (idx != -1)
            {
                list[listIndex++] = (idx, quad);
            }
        }

        Debug.Assert(listIndex == childCount);

        var newNode = Allocate();

        list[childCount] = (newNode, targetQuad);

        Debug.Assert(list[^1].index == newNode);

        list.Sort(CompareChildren);

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

        _nodes[parentIdx] = parent;

        return newNode;
    }

    private void Free(int node)
    {
        if (node is 0 or -1)
        {
            throw new InvalidOperationException();
        }

        _nodes[node] = default;
        _data[node] = default;
        _free.Enqueue(node);
        NodeCount--;
    }

    private void FreeLeaf(ref LinkedQuadTreeNode parent, Quadrant targetQuad)
    {
        Debug.Assert(parent.HasChild(targetQuad));

        var indices = new IndexList(_nodes, parent);
        var targetIdx = indices[targetQuad];

        Debug.Assert(targetIdx != -1);
        Debug.Assert(_nodes[targetIdx].ChildCount == 0);

        Free(targetIdx);
        parent.DeactivateChild(targetQuad);

        var count = parent.ChildCount;

        if (count == 0)
        {
            parent.ChildList = 0;
            return;
        }

        indices = indices.Without(targetQuad);

        // Reorder children:

        Span<(int index, Quadrant q)> children = stackalloc (int node, Quadrant q)[count];

        var childIndex = 0;
        for (var quadIndex = 0; quadIndex < 4; quadIndex++)
        {
            var quad = (Quadrant)quadIndex;

            if (indices.Has(quad))
            {
                children[childIndex++] = (indices[quad], quad);
            }
        }

        Debug.Assert(childIndex == count);

        children.Sort(CompareChildren);

        var previous = children[0];
        parent.ChildList = (uint)previous.index;

        for (var i = 1; i < count; i++)
        {
            var current = children[i];
            _nodes[previous.index].Sibling = (uint)current.index;
            previous = current;
        }
    }

    public bool IsWithinBounds(Vector2di position) => IsWithinBounds(Vector2di.Zero, position, Log);

    public bool Insert(Vector2di position, T data)
    {
        if (!IsWithinBounds(position))
        {
            throw new ArgumentOutOfRangeException(nameof(position));
        }

        var result = InsertCore(Vector2di.Zero, Log, 0, position, data);

        if (result)
        {
            Fill();
        }
        
        _insertRemovePath.Clear();

        return result;
    }

    private bool InsertCore(Vector2di parentPos, byte parentLog, int parentIdx, Vector2di targetPos, T targetData)
    {
        while (true)
        {
            Debug.Assert(parentLog > 0);

            _insertRemovePath.Push(parentIdx);

            ref var parent = ref _nodes[parentIdx];
            var parentSize = 1 << parentLog;
            var targetQuad = GetChildQuadrant(parentPos, parentSize, targetPos);

            if (parent.IsFilled)
            {
                Debug.Assert(parent.ChildList == 0);

                var parentData = _data[parentIdx];

                if (AreEqual(parentData, targetData))
                {
                    return false;
                }

                // We need to allocate children for this node.
                // We'll allocate children, make them filled with the data stored in the parent,
                // then we'll continue descending into the child that will get the new data.
                // If the parent log is 1, then we'll just set the data here instead of descending.

                parent.IsFilled = false;
                parent.ChildList = (uint)AllocateList(4, new LinkedQuadTreeNode { IsFilled = true }, parentData);
                parent = ref _nodes[parentIdx]; // Re-acquire reference (resize array)
                parent.ActivateChildren();

                Debug.Assert(parent.ChildCount == 4);

                var indices = new IndexList(_nodes, parent);
                
                var childIdx = indices[targetQuad];

                Debug.Assert(childIdx != -1);

                if (parentLog == 1)
                {
                    // Set the child here
                    _data[childIdx] = targetData;
                    return true;
                }

                parentPos = GetChildPosition(parentPos, parentSize, targetQuad);
                parentLog--;
                parentIdx = childIdx;
                continue;
            }

            // Used to determine if we inserted a node.
            // We need that because we can't rely on comparing the inserted data with the default value, stored in the fresh slot, to determine if we inserted or not.
            var isNewChild = false;
            int child;

            if (!parent.HasChild(targetQuad))
            {
                child = AllocateChild(parentIdx, targetQuad);
                parent = ref _nodes[parentIdx]; // Re-acquire reference (resize array)
                isNewChild = true;
            }
            else
            {
                child = new IndexList(_nodes, parent)[targetQuad];
            }

            Debug.Assert(child != -1);

            if (parentLog == 1)
            {
                // We'll make sure the size 1 node is filled:
                _nodes[child].IsFilled = true;

                Debug.Assert(_nodes[child] is { ChildCount: 0, ChildList: 0 });

                if (AreEqual(_data[child], targetData))
                {
                    // We inserted if and only if this child didn't exist beforehand.
                    return isNewChild;
                }

                _data[child] = targetData;
                return true;
            }

            parentPos = GetChildPosition(parentPos, parentSize, targetQuad);
            parentLog--;
            parentIdx = child;
        }
    }

    private void Fill()
    {
        while (_insertRemovePath.Count > 0)
        {
            var nodeIdx = _insertRemovePath.Pop();

            ref var node = ref _nodes[nodeIdx];

            Debug.Assert(!node.IsFilled);

            if (node.ChildCount != 4)
            {
                return;
            }

            var indices = new IndexList(_nodes, node);

            var data = _data[indices.TopLeft];

            for (byte i = 0; i < 4; i++)
            {
                var idx = indices[i];

                Debug.Assert(idx != -1);

                if (!_nodes[idx].IsFilled || !AreEqual(_data[idx], data))
                {
                    return;
                }

                Debug.Assert(_nodes[idx] is { ChildCount: 0, ChildList: 0 });
            }

            // They are filled, so they don't have children and we don't have to free anything else.

            Free(indices.TopLeft);
            Free(indices.TopRight);
            Free(indices.BottomLeft);
            Free(indices.BottomRight);

            node.ClearChildren();
            node.IsFilled = true;

            _data[nodeIdx] = data;
        }
    }

    public bool Remove(Vector2di position)
    {
        if (!IsWithinBounds(position))
        {
            throw new ArgumentOutOfRangeException(nameof(position));
        }

        var result = RemoveCore(Vector2di.Zero, Log, 0, position);

        if (result)
        {
            Trim();
        }

        _insertRemovePath.Clear();

        return result;
    }

    private bool RemoveCore(Vector2di parentPos, byte parentLog, int parentIdx, Vector2di targetPos)
    {
        while (true)
        {
            _insertRemovePath.Push(parentIdx);

            ref var parent = ref _nodes[parentIdx];

            var parentSize = 1 << parentLog;
            var childQuadrant = GetChildQuadrant(parentPos, parentSize, targetPos);

            if (!parent.IsFilled && !parent.HasChild(childQuadrant))
            {
                return false;
            }

            if (parent.IsFilled)
            {
                // We'll have to split it up, like with insert

                var parentData = _data[parentIdx];

                parent.IsFilled = false;

                if (parentLog == 1)
                {
                    // We'll allocate only the 3 children and we're done
                    parent.ChildList = (uint)AllocateList(3, new LinkedQuadTreeNode { IsFilled = true }, parentData);
                    parent = ref _nodes[parentIdx]; // Re-acquire reference (resize array)
                    parent.ActivateChildren();
                    parent.DeactivateChild(childQuadrant);

                    return true;
                }

                parent.ChildList = (uint)AllocateList(4, new LinkedQuadTreeNode { IsFilled = true }, parentData);
                parent = ref _nodes[parentIdx]; // Re-acquire reference (resize array)
                parent.ActivateChildren();
            }

            if (parentLog == 1)
            {
                FreeLeaf(ref parent, childQuadrant);
                return true;
            }

            var indices = new IndexList(_nodes, parent);
            var childIdx = indices[childQuadrant];

            Debug.Assert(childIdx != -1);

            parentPos = GetChildPosition(parentPos, parentSize, childQuadrant);
            parentLog = (byte)(parentLog - 1);
            parentIdx = childIdx;
        }
    }

    private void Trim()
    {
        while (_insertRemovePath.Count > 0)
        {
            var nodeIdx = _insertRemovePath.Pop();

            if (nodeIdx == 0)
            {
                return;
            }

            Debug.Assert(nodeIdx != -1);

            ref var node = ref _nodes[nodeIdx];

            if (node.ChildCount != 0 || node.IsFilled)
            {
                return;
            }

            if (!_insertRemovePath.TryPeek(out var parentIdx))
            {
                return;
            }

            ref var parent = ref _nodes[parentIdx];
            var indices = new IndexList(_nodes, parent);

            Quadrant? childQuad = null;
            for (var i = 0; i < 4; i++)
            {
                var quad = (Quadrant)i;

                if (indices[quad] == nodeIdx)
                {
                    childQuad = quad;
                    break;
                }
            }

            if (childQuad == null)
            {
                throw new Exception();
            }

            FreeLeaf(ref parent, childQuad.Value);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CompareChildren((int, Quadrant) a, (int, Quadrant) b) => ((int)a.Item2).CompareTo((int)b.Item2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Quadrant GetChildQuadrant(Vector2di parentPos, int parentSize, Vector2di childPos)
    {
        var isLeft = childPos.X < parentPos.X + parentSize / 2;

        if (childPos.Y <= parentPos.Y - parentSize / 2)
        {
            return isLeft ? Quadrant.BottomLeft : Quadrant.BottomRight;
        }

        return isLeft ? Quadrant.TopLeft : Quadrant.TopRight;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2di GetChildPosition(Vector2di parentPos, int parentSize, Quadrant quad) => quad switch
    {
        Quadrant.TopLeft => parentPos,
        Quadrant.TopRight => new Vector2di(parentPos.X + parentSize / 2, parentPos.Y),
        Quadrant.BottomLeft => new Vector2di(parentPos.X, parentPos.Y - parentSize / 2),
        Quadrant.BottomRight => new Vector2di(parentPos.X + parentSize / 2, parentPos.Y - parentSize / 2),
        _ => throw new ArgumentOutOfRangeException(nameof(quad), quad, null)
    };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsWithinBounds(Vector2di nodePos, Vector2di targetPos, byte log)
    {
        var size = 1 << log;
        return targetPos.X >= nodePos.X && targetPos.Y <= nodePos.Y && targetPos.X < size && targetPos.Y > -size;
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

        public IndexList(LinkedQuadTreeNode[] storage, LinkedQuadTreeNode parent)
        {
            TopLeft = -1;
            TopRight = -1;
            BottomLeft = -1;
            BottomRight = -1;

            if (parent.ChildCount == 0)
            {
                return;
            }

            var currentChild = parent.ChildList;

#if DEBUG
            var count = 0;
#endif

            for (var i = 0; i < 4; i++)
            {
                var quad = (Quadrant)i;
                
                var target = parent.HasChild(quad) 
                    ? (int)currentChild 
                    : -1;

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
#if DEBUG
                    ++count;
#endif
                    currentChild = storage[currentChild].Sibling;
                }
            }

#if DEBUG
            Debug.Assert(count == parent.ChildCount);
#endif
        }

        private IndexList(int tl, int tr, int bl, int br)
        {
            TopLeft = tl;
            TopRight = tr;
            BottomLeft = bl;
            BottomRight = br;
        }

        public IndexList Without(Quadrant q)
        {
            var tl = TopLeft;
            var tr = TopRight;
            var bl = BottomLeft;
            var br = BottomRight;

            switch (q)
            {
                case Quadrant.TopLeft:
                    tl = -1;
                    break;
                case Quadrant.TopRight:
                    tr = -1;
                    break;
                case Quadrant.BottomLeft:
                    bl = -1;
                    break;
                case Quadrant.BottomRight:
                    br = -1;
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(q), q, null);
            }

            return new IndexList(tl, tr, bl, br);
        }
    }

    private readonly struct TraverseFrame
    {
        public Vector2di Position { get; init; }
        public int Index { get; init; }
        public byte Log { get; init; }
    }
}