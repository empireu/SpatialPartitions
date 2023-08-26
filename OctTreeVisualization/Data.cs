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
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ActivateChild(int childIdx)
    {
        Debug.Assert(childIdx is >= 0 and < 8);
        ChildMask |= (byte)(1 << childIdx);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ResetChild(int childIdx)
    {
        Debug.Assert(childIdx is >= 0 and < 8);
        ChildMask &= (byte)(byte.MaxValue ^ (1 << childIdx));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ResetChildren()
    {
        ChildMask = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool HasChild(int childIdx)
    {
        Debug.Assert(childIdx is >= 0 and < 8);
        return ((ChildMask >> childIdx) & 1) > 0;
    }
}

// Bit Octree, as in, 2 states per voxel.
// I plan to use this in the future to represent open and closed volumes for fast nearest queries, etc.

public sealed class HashedBitOctree
{
    public const ulong RootCode = 1;
    public const ulong NullCode = 0;

    private readonly Dictionary<ulong, HashedBitOctreeNode> _nodes = new();

    public HashedBitOctree(byte log)
    {
        Log = log;
    }

    public byte Log { get; }

    public int EdgeSize => 1 << Log;

    public Vector3di Min => new(-EdgeSize / 2);
    public Vector3di Max => new(EdgeSize / 2);
}