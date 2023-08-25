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

public interface IQuadTree<in T> where T : struct
{
    int NodeCount { get; }
    bool Insert(Vector2di tile, T data);
    bool Remove(Vector2di tile);
    int Find(Vector2di tile);
}

#region Classic Quadtree

public unsafe struct ClassicQuadTreeNode
{
    public fixed int Children[4];
    public bool IsFilled;

    public bool HasChildren => Children[0] != -1 || Children[1] != -1 || Children[2] != -1 || Children[3] != -1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasChild(Quadrant q) => Children[(int)q] != -1;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasChild(int q) => Children[q] != -1;

    public void ClearChildren()
    {
        Children[0] = -1;
        Children[1] = -1;
        Children[2] = -1;
        Children[3] = -1;
    }
}

/*
 * Classic quadtree implementation, storing pointers to all children (but not parent pointers).
 * It stores 17 explicit bytes per node.
 */
public sealed class ClassicQuadTree<T> : IQuadTree<T> where T : unmanaged
{
    public delegate bool TraverseDelegate(int index, Vector2di position, byte log, in ClassicQuadTreeNode node);

    private readonly IEqualityComparer<T> _comparer;
    private readonly int _root;
    private readonly Stack<int> _insertRemovePath = new();
    private readonly Stack<TraverseFrame> _traverseStack = new();
    private readonly Queue<int> _free = new();
    private ClassicQuadTreeNode[] _nodes = new ClassicQuadTreeNode[16];
    private T[] _data = new T[16];

    public ClassicQuadTree(byte log, IEqualityComparer<T>? comparer = null)
    {
        _comparer = comparer ?? EqualityComparer<T>.Default;
        _root = Allocate();
        Debug.Assert(_root == 0);
        Log = log;
    }

    public byte Log { get; }

    public int NodeCount { get; private set; }

    public int Size => 1 << Log;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool AreEqual(T a, T b) => _comparer.Equals(a, b);

    public T GetData(int node) => _data[node];
    public ClassicQuadTreeNode GetNode(int node) => _nodes[node];

    public bool IsWithinBounds(Vector2di position) => IsWithinBounds(Vector2di.Zero, position, Log);

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

    #region Traversal

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

    private unsafe void TraverseCore(TraverseDelegate visit)
    {
        while (_traverseStack.Count > 0)
        {
            var frame = _traverseStack.Pop();

            ref var node = ref _nodes[frame.Index];

            if (!visit(frame.Index, frame.Position, frame.Log, in node))
            {
                return;
            }

            if (!node.HasChildren)
            {
                continue;
            }

            for (var i = 0; i < 4; i++)
            {
                var quad = (Quadrant)i;

                if (node.HasChild(quad))
                {
                    _traverseStack.Push(new TraverseFrame
                    {
                        Position = GetChildPosition(frame.Position, 1 << frame.Log, quad),
                        Index = node.Children[(int)quad],
                        Log = (byte)(frame.Log - 1)
                    });
                }
            }
        }
    }

    public unsafe int Find(Vector2di position)
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

            if (!parent.HasChildren)
            {
                return -1;
            }

            var quad = GetChildQuadrant(parentPos, parentSize, position);

            var childIdx = parent.Children[(int)quad];

            if (childIdx == -1)
            {
                return -1;
            }

            parentPos = GetChildPosition(parentPos, parentSize, quad);
            parentIdx = childIdx;
            parentLog = (byte)(parentLog - 1);
        }
    }


    #endregion

    #region Allocation

    private int Allocate()
    {
        if (!_free.TryDequeue(out var pNode))
        {
            if (_nodes.Length == NodeCount)
            {
                var size = _nodes.Length * 2;
                Array.Resize(ref _nodes, size);
                Array.Resize(ref _data, size);
            }

            pNode = NodeCount;
        }

        NodeCount++;

        _nodes[pNode].ClearChildren();

        return pNode;
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


    #endregion

    #region Insertion

    public bool Insert(Vector2di position, T data)
    {
        if (!IsWithinBounds(position))
        {
            throw new ArgumentOutOfRangeException(nameof(position));
        }

        var result = InsertCore(Vector2di.Zero, Log, _root, position, data);

        if (result)
        {
            Fill();
        }

        _insertRemovePath.Clear();

        return result;
    }

    private unsafe bool InsertCore(Vector2di parentPos, byte parentLog, int parentIdx, Vector2di targetPos, T targetData)
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
                var parentData = _data[parentIdx];

                if (AreEqual(parentData, targetData))
                {
                    return false;
                }

                parent.IsFilled = false;

                for (var i = 0; i < 4; i++)
                {
                    var pChild = Allocate();
                    parent = ref _nodes[parentIdx];
                    parent.Children[i] = pChild;
                    _nodes[pChild].IsFilled = true;
                    _data[pChild] = parentData;
                }

                var childIdx = parent.Children[(int)targetQuad];

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

            if (parent.HasChild(targetQuad))
            {
                child = parent.Children[(int)targetQuad];
            }
            else
            {
                child = Allocate();
                parent = ref _nodes[parentIdx];
                parent.Children[(int)targetQuad] = child;
                isNewChild = true;
            }

            Debug.Assert(child != -1);

            if (parentLog == 1)
            {
                // We'll make sure the size 1 node is filled:
                _nodes[child].IsFilled = true;

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

    private unsafe void Fill()
    {
        while (_insertRemovePath.Count > 0)
        {
            var nodeIdx = _insertRemovePath.Pop();

            ref var node = ref _nodes[nodeIdx];

            Debug.Assert(!node.IsFilled);

            if (!node.HasChild(0))
            {
                return;
            }

            var data = _data[node.Children[0]];

            for (var i = 0; i < 4; i++)
            {
                var pChild = node.Children[i];

                if (pChild == -1 || !_nodes[pChild].IsFilled || !AreEqual(_data[pChild], data))
                {
                    return;
                }
            }

            Free(node.Children[0]);
            Free(node.Children[1]);
            Free(node.Children[2]);
            Free(node.Children[3]);

            node.ClearChildren();
            node.IsFilled = true;
            
            _data[nodeIdx] = data;
        }
    }

    #endregion

    #region Removal

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

    private unsafe bool RemoveCore(Vector2di parentPos, byte parentLog, int parentIdx, Vector2di targetPos)
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
                    for (var i = 0; i < 4; i++)
                    {
                        if (i == (int)childQuadrant)
                        {
                            continue;
                        }

                        var pChild = Allocate();
                        parent = ref _nodes[parentIdx];
                        _nodes[pChild].IsFilled = true;
                        _data[pChild] = parentData;
                        parent.Children[i] = pChild;
                    }

                    return true;
                }

                for (var i = 0; i < 4; i++)
                {
                    var pChild = Allocate();
                    parent = ref _nodes[parentIdx];
                    _nodes[pChild].IsFilled = true;
                    _data[pChild] = parentData;
                    parent.Children[i] = pChild;
                }
            }

            if (parentLog == 1)
            {
                Free(parent.Children[(int)childQuadrant]);
                parent.Children[(int)childQuadrant] = -1;
                return true;
            }

            parentPos = GetChildPosition(parentPos, parentSize, childQuadrant);
            parentLog = (byte)(parentLog - 1);
            parentIdx = parent.Children[(int)childQuadrant];
        }
    }

    private unsafe void Trim()
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

            if (node.HasChildren || node.IsFilled)
            {
                return;
            }

            if (!_insertRemovePath.TryPeek(out var parentIdx))
            {
                return;
            }

            ref var parent = ref _nodes[parentIdx];

            for (var i = 0; i < 4; i++)
            {
                var quad = (Quadrant)i;
                var pChild = parent.Children[(int)quad];
                
                if (pChild == nodeIdx)
                {
                    Free(pChild);
                    parent.Children[(int)quad] = -1;
                    goto endIteration;
                }
            }

            throw new Exception();

            endIteration:
            continue;
        }
    }

    #endregion

    private readonly struct TraverseFrame
    {
        public Vector2di Position { get; init; }
        public int Index { get; init; }
        public byte Log { get; init; }
    }
}

#endregion

#region Hashed Quadtree

public abstract class HashedQuadTree
{
    public const ulong RootCode = 1;
    public const ulong NullCode = 0;
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Quadrant GetChildQuadrant(Vector2di parentPos, int parentSize, Vector2di childPos)
    {
        var isLeft = childPos.X < parentPos.X + parentSize / 2;

        if (childPos.Y <= parentPos.Y - parentSize / 2)
        {
            return isLeft ? Quadrant.BottomLeft : Quadrant.BottomRight;
        }

        return isLeft ? Quadrant.TopLeft : Quadrant.TopRight;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2di GetChildPosition(Vector2di parentPos, int parentSize, Quadrant quad) => quad switch
    {
        Quadrant.TopLeft => parentPos,
        Quadrant.TopRight => new Vector2di(parentPos.X + parentSize / 2, parentPos.Y),
        Quadrant.BottomLeft => new Vector2di(parentPos.X, parentPos.Y - parentSize / 2),
        Quadrant.BottomRight => new Vector2di(parentPos.X + parentSize / 2, parentPos.Y - parentSize / 2),
        _ => throw new ArgumentOutOfRangeException(nameof(quad), quad, null)
    };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsWithinBounds(Vector2di nodePos, Vector2di targetPos, byte log)
    {
        var size = 1 << log;
        return targetPos.X >= nodePos.X && targetPos.Y <= nodePos.Y && targetPos.X < size && targetPos.Y > -size;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ulong GetChildLocationCode(ulong parentLc, Quadrant child) => (parentLc << 2) | (byte)child;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ulong GetParentLocationCode(ulong childLc) => childLc >> 2;

    protected readonly struct TraverseFrame
    {
        public required Vector2di Position { get; init; }
        public required ulong Lc { get; init; }
        public required byte Log { get; init; }
    }

    protected readonly struct InsertRemoveFrame
    {
        public required ulong Code { get; init; }
        public required Quadrant ChildQuad { get; init; }
    }
}

/*
 * "Hashed" quadtree implementation. Nodes get unique codes based on their path from the root. The code is easy to calculate; both for children,
 * given the parent's code and the child quadrant, and for parents, given the child's code. As such, no pointers need to be stored, and the tree can be traversed down and up.
 * Nodes are stored in a map (Dictionary). To speed up lookups, nodes store a mask of the children they actually have.
 * It stores 1 explicit byte per node. There is some overhead added by the dictionary, though.
 * In testing, inserting is ~3 times slower than the classic implementation. Traversal is ~2 times slower, and removal is ~3 times slower.
 * A higher branching factor (octree) could benefit from this. However, the maximum depth would be cut (out of space for key):
 *  Quadtree:
 *      Depth 0 - 1 bit
 *      Depth 1 - 3 bit
 *      Depth 2 - 5 bit
 *      ...
 *      Depth 31 - 63 bit, which is very useful and enough for most purposes.
 *
 * Octree:
 *  - A "classical octree" would need 32 bytes to store child pointers.
 *      Depth 0 - 1 bit
 *      Depth 1 - 4 bit
 *      Depth 2 - 7 bit
 *      ...
 *      Depth 21 - 64 bit, which may actually be prohibitive, but it is still good for most applications.
 *  - To allow a depth of 31, 94 bits would be needed.
 *
 */
public sealed unsafe class HashedQuadTree<T> : HashedQuadTree, IQuadTree<T> where T : unmanaged
{
    public delegate bool TraverseDelegate(ulong lcNode, Vector2di position, byte log, in Node node);

    private readonly IEqualityComparer<T> _comparer;
    private readonly Stack<InsertRemoveFrame> _insertRemoveStack = new();
    private readonly Stack<TraverseFrame> _traverseStack = new();
    private readonly Dictionary<ulong, Node> _nodes = new();

    public HashedQuadTree(byte log, IEqualityComparer<T>? comparer = null)
    {
        Log = log;
        _comparer = comparer ?? EqualityComparer<T>.Default;
        _nodes.Add(RootCode, new Node());
    }

    public byte Log { get; }
    
    public int NodeCount => _nodes.Count;
   
    public int Size => 1 << Log;

    #region Insertion

    public bool Insert(Vector2di position, T data)
    {
        if (!IsWithinBounds(position))
        {
            throw new ArgumentOutOfRangeException(nameof(position));
        }

        var result = InsertCore(Vector2di.Zero, Log, RootCode, position, data);

        if (result)
        {
            Fill();
        }

        _insertRemoveStack.Clear();

        return result;
    }

    private bool InsertCore(Vector2di parentPos, byte parentLog, ulong lcParent, Vector2di targetPos, T targetData)
    {
        while (true)
        {
            Debug.Assert(parentLog > 0);

            var parent = _nodes[lcParent];
            var parentSize = 1 << parentLog;
            var targetQuad = GetChildQuadrant(parentPos, parentSize, targetPos);

            _insertRemoveStack.Push(new InsertRemoveFrame
            {
                Code = lcParent,
                ChildQuad = targetQuad
            });

            var lcChild = GetChildLocationCode(lcParent, targetQuad);

            if (parent.IsFilled)
            {
                var parentData = parent.Data;

                if (AreEqual(parentData, targetData))
                {
                    return false;
                }

                parent.ResetFilled();

                for (var i = 0; i < 4; i++)
                {
                    var quad = (Quadrant)i;
                    parent.ActivateChild(quad);

                    var node = new Node();
                    node.SetFilled();
                    node.Data = parentData;

                    _nodes.Add(GetChildLocationCode(lcParent, quad), node);
                }

                if (parentLog == 1)
                {
                    // Set the child here
                    _nodes[lcChild] = _nodes[lcChild] with { Data = targetData };
                    _nodes[lcParent] = parent; // !
                    return true;
                }

                parentPos = GetChildPosition(parentPos, parentSize, targetQuad);
                parentLog--;
                _nodes[lcParent] = parent; // !
                lcParent = lcChild;
                continue;
            }

            // Used to determine if we inserted a node.
            // We need that because we can't rely on comparing the inserted data with the default value, stored in the fresh slot, to determine if we inserted or not.
            var isNewChild = false;

            if (!parent.HasChild(targetQuad))
            {
                _nodes.Add(lcChild, new Node());
                isNewChild = true;
                parent.ActivateChild(targetQuad);
            }

            if (parentLog == 1)
            {
                var child = _nodes[lcChild];
                child.SetFilled();
                var originalData = child.Data;
                child.Data = targetData;
                _nodes[lcChild] = child;
                _nodes[lcParent] = parent; // !
                return isNewChild || !AreEqual(originalData, targetData);
            }

            parentPos = GetChildPosition(parentPos, parentSize, targetQuad);
            parentLog--;
            _nodes[lcParent] = parent; // !
            lcParent = lcChild;
        }
    }

    private void Fill()
    {
        Span<ulong> lcsChildren = stackalloc ulong[4];

        while (_insertRemoveStack.Count > 0)
        {
            var frame = _insertRemoveStack.Pop();
            var lcNode = frame.Code;
            var node = _nodes[lcNode];

            Debug.Assert(!node.IsFilled);

            if (!node.HasChild(0))
            {
                return;
            }

            var data = _nodes[GetChildLocationCode(lcNode, 0)].Data;

            for (var i = 0; i < 4; i++)
            {
                var q = (Quadrant)i;

                if (!node.HasChild(q))
                {
                    return;
                }

                var lc = GetChildLocationCode(lcNode, q);
                var child = _nodes[lc];

                if (!child.IsFilled || !AreEqual(child.Data, data))
                {
                    return;
                }

                lcsChildren[i] = lc;
            }

            for (var i = 0; i < 4; i++)
            {
                _nodes.Remove(lcsChildren[i]);
            }

            node.ResetChildren();
            node.SetFilled();
            node.Data = data;

            _nodes[lcNode] = node;
        }
    }

    #endregion

    #region Traversal

    public void Traverse(TraverseDelegate visit)
    {
        _traverseStack.Push(new TraverseFrame
        {
            Position = Vector2di.Zero,
            Lc = RootCode,
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
            var node = _nodes[frame.Lc];

            if (!visit(frame.Lc, frame.Position, frame.Log, in node))
            {
                return;
            }

            if (!node.HasChildren)
            {
                continue;
            }

            for (var i = 0; i < 4; i++)
            {
                var quad = (Quadrant)i;

                if (node.HasChild(quad))
                {
                    _traverseStack.Push(new TraverseFrame
                    {
                        Position = GetChildPosition(frame.Position, 1 << frame.Log, quad),
                        Lc = GetChildLocationCode(frame.Lc, quad),
                        Log = (byte)(frame.Log - 1)
                    });
                }
            }
        }
    }

    #endregion

    #region Search

    // todo cuts depth because we cast... oh well
    public int Find(Vector2di position)
    {
        if (!IsWithinBounds(position))
        {
            return -1;
        }

        var parentLog = Log;
        var parentPos = Vector2di.Zero;
        var parentLc = RootCode;

        while (true)
        {
            var parent = _nodes[parentLc];

            var parentSize = 1 << parentLog;

            if (parent.IsFilled)
            {
                return (int)parentLc;
            }

            if (!parent.HasChildren)
            {
                return -1;
            }

            var quad = GetChildQuadrant(parentPos, parentSize, position);

            if (!parent.HasChild(quad))
            {
                return -1;
            }

            parentPos = GetChildPosition(parentPos, parentSize, quad);
            parentLc = GetChildLocationCode(parentLc, quad);
            parentLog = (byte)(parentLog - 1);
        }
    }

    #endregion

    #region Removal

    public bool Remove(Vector2di position)
    {
        if (!IsWithinBounds(position))
        {
            throw new ArgumentOutOfRangeException(nameof(position));
        }

        var result = RemoveCore(Vector2di.Zero, Log, RootCode, position);

        if (result)
        {
            Trim();
        }

        _insertRemoveStack.Clear();

        return result;
    }

    private bool RemoveCore(Vector2di parentPos, byte parentLog, ulong lcParent, Vector2di targetPos)
    {
        while (true)
        {
            var parent = _nodes[lcParent];
            var parentSize = 1 << parentLog;
            var targetQuad = GetChildQuadrant(parentPos, parentSize, targetPos);
            
            if (!parent.IsFilled && !parent.HasChild(targetQuad))
            {
                return false;
            }

            _insertRemoveStack.Push(new InsertRemoveFrame
            {
                Code = lcParent,
                ChildQuad = targetQuad
            });

            if (parent.IsFilled)
            {
                // We'll have to split it up, like with insert
                var parentData = parent.Data;
                parent.ResetFilled();
                parent.Data = default;

                if (parentLog == 1)
                {
                    for (var i = 0; i < 4; i++)
                    {
                        if (i == (int)targetQuad)
                        {
                            continue;
                        }

                        var q = (Quadrant)i;
                        
                        var child = new Node();
                        child.SetFilled();
                        child.Data = parentData;
                        
                        _nodes.Add(GetChildLocationCode(lcParent, q), child);
                        parent.ActivateChild(q);
                    }

                    _nodes[lcParent] = parent; // !

                    return true;
                }

                for (var i = 0; i < 4; i++)
                {
                    var q = (Quadrant)i;

                    var child = new Node();
                    child.SetFilled();
                    child.Data = parentData;

                    _nodes.Add(GetChildLocationCode(lcParent, q), child);
                    parent.ActivateChild(q);
                }
            }

            var lcChild = GetChildLocationCode(lcParent, targetQuad);

            if (parentLog == 1)
            {
#if DEBUG
                Debug.Assert(_nodes.Remove(lcChild));
#else
                _nodes.Remove(lcChild);
#endif
                parent.ResetChild(targetQuad);
                _nodes[lcParent] = parent; // !
                return true;
            }

            parentPos = GetChildPosition(parentPos, parentSize, targetQuad);
            parentLog = (byte)(parentLog - 1);
            _nodes[lcParent] = parent; // !
            lcParent = lcChild;
        }
    }

    private void Trim()
    {
        while (_insertRemoveStack.Count > 0)
        {
            var frame = _insertRemoveStack.Pop();
            var lcNode = frame.Code;

            if (lcNode == RootCode)
            {
                return;
            }

            var node = _nodes[lcNode];

            if (node.HasChildren || node.IsFilled)
            {
                return;
            }

            var parentFrame = _insertRemoveStack.Peek();
            var lcParent = parentFrame.Code;
            var parent = _nodes[lcParent];

            _nodes.Remove(lcNode);
            parent.ResetChild(parentFrame.ChildQuad);
            _nodes[lcParent] = parent; // ! Only parent is mutated
        }
    }

    #endregion

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Node GetNode(ulong node) => _nodes[node];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool AreEqual(T a, T b) => _comparer.Equals(a, b);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsWithinBounds(Vector2di position) => IsWithinBounds(Vector2di.Zero, position, Log);

    public struct Node
    {
        public T Data;
        public byte Mask;

        public readonly bool HasChildren => (Mask & 15) > 0;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly bool HasChild(Quadrant q) => ((Mask >> (int)q) & 1) > 0;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ActivateChild(Quadrant q) => Mask |= (byte)(1 << (byte)q);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ResetChild(Quadrant q) => Mask &= (byte)(31 ^ (1 << (int)q));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ResetChildren() => Mask &= 1 << 4;

        public readonly bool IsFilled => (Mask & (1 << 4)) > 0;
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetFilled() => Mask |= 1 << 4;
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ResetFilled() => Mask &= 31 ^ (1 << 4);
    }
}

#endregion