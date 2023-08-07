﻿using System.Drawing;
using System.Runtime.CompilerServices;
using Common;

namespace QuadTreeVisualization;

public sealed class BitQuadTree
{
    public enum Quadrant : byte
    {
        BottomLeft = 0,
        BottomRight = 1,
        TopLeft = 2,
        TopRight = 3
    }

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