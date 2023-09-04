using Newtonsoft.Json.Linq;
using SixLabors.Fonts;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Common;

public interface IReadOnlyGrid2d<out T>
{
    Vector2ds Size { get; }
    T this[int x, int y] { get; }
    T this[Vector2ds tile] { get; }
    bool IsWithinBounds(int x, int y);
    bool IsWithinBounds(Vector2ds tile);
}

public interface IGrid2d<T> : IReadOnlyGrid2d<T>
{
    new T this[int x, int y] { set; }
    new T this[Vector2ds tile] { set; }
}

public sealed class Grid2d<T> : IGrid2d<T>
{
    public Vector2ds Size { get; }

    public T[] Storage { get; }

    public Grid2d(Vector2ds size)
    {
        if (size.X * size.Y == 0)
        {
            throw new ArgumentException("Grid surface is 0", nameof(size));
        }

        Size = size;
        Storage = new T[size.X * size.Y];
    }

    public Grid2d(int width, int height) : this(new Vector2ds(width, height))
    {

    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GridIndex(int x, int y) => x + y * Size.X;

    T IReadOnlyGrid2d<T>.this[int tileX, int tileY]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Storage[GridIndex(tileX, tileY)];
    }

    T IReadOnlyGrid2d<T>.this[Vector2ds tile]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Storage[GridIndex(tile.X, tile.Y)];
    }

    T IGrid2d<T>.this[int tileX, int tileY]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Storage[GridIndex(tileX, tileY)] = value;
    }

    T IGrid2d<T>.this[Vector2ds tile]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Storage[GridIndex(tile.X, tile.Y)] = value;
    }

    public ref T this[int tileX, int tileY]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ref Storage[GridIndex(tileX, tileY)];
    }

    public ref T this[Vector2ds tile]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ref Storage[GridIndex(tile.X, tile.Y)];
    }

    public bool IsWithinBounds(int x, int y) => x >= 0 && x < Size.X && y >= 0 && y < Size.Y;
    public bool IsWithinBounds(Vector2ds tile) => tile.X >= 0 && tile.X < Size.X && tile.Y >= 0 && tile.Y < Size.Y;

    public Grid2d<T> Bind()
    {
        var result = new Grid2d<T>(Size);

        Array.Copy(Storage, result.Storage, result.Storage.Length);

        return result;
    }
}

public interface IGrid3d<T>
{
    Vector3di Size { get; }
    T this[int x, int y, int z] { get; set; }
    T this[Vector3di tile] { get; set; }
}

public sealed class Grid3d<T> : IGrid3d<T>
{
    public Vector3di Size { get; }

    public T[] Storage { get; }

    public Grid3d(Vector3di size)
    {
        if (size.X * size.Y * size.Z == 0)
        {
            throw new ArgumentException("Grid volume is 0", nameof(size));
        }

        Size = size;
        Storage = new T[size.X * size.Y * size.Z];
    }

    public int Count => Storage.Length;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GridIndex(int x, int y, int z) => x + y * Size.X + z * Size.X * Size.Y;

    T IGrid3d<T>.this[int tileX, int tileY, int tileZ]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Storage[GridIndex(tileX, tileY, tileZ)];
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Storage[GridIndex(tileX, tileY, tileZ)] = value;
    }

    T IGrid3d<T>.this[Vector3di tile]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Storage[GridIndex(tile.X, tile.Y, tile.Z)];
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Storage[GridIndex(tile.X, tile.Y, tile.Z)] = value;
    }

    public ref T this[int tileX, int tileY, int tileZ]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ref Storage[GridIndex(tileX, tileY, tileZ)];
    }

    public ref T this[Vector3di tile]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => ref Storage[GridIndex(tile.X, tile.Y, tile.Z)];
    }
}

public readonly struct GridPageKey3d
{
    public int X { get; }
    public int Y { get; }
    public int Z { get; }

    public GridPageKey3d(int x, int y, int z)
    {
        X = x;
        Y = y;
        Z = z;
    }
    public Vector3di ToTile(int edgeSize) => new(X * edgeSize, Y * edgeSize, Z * edgeSize);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int MapAxis(int tileCoordinate, int size) => (int)Math.Floor(tileCoordinate / (double)size);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static GridPageKey3d FromTile(int tileX, int tileY, int tileZ, int edgeSize) => new(
        MapAxis(tileX, edgeSize), 
        MapAxis(tileY, edgeSize), 
        MapAxis(tileZ, edgeSize)
    );

    public override bool Equals(object? obj) => obj is GridPageKey3d other && Equals(other);

    public bool Equals(GridPageKey3d other) => X == other.X && Y == other.Y && Z == other.Z;

    public override int GetHashCode() => HashCode.Combine(X, Y, Z);

    public override string ToString() => $"{nameof(X)}: {X}, {nameof(Y)}: {Y}, {nameof(Z)}: {Z}";

    public static bool operator ==(GridPageKey3d a, GridPageKey3d b) => a.Equals(b);

    public static bool operator !=(GridPageKey3d a, GridPageKey3d b) => !a.Equals(b);
}

public sealed class GridPage3d<T> : IGrid3d<T>
{
    public GridPageKey3d Key3d { get; }
    public int Log { get; }
    public T[] Storage { get; }

    public GridPage3d(GridPageKey3d key, int log)
    {
        Key3d = key;
        Log = log;
        Storage = new T[EdgeSize * EdgeSize * EdgeSize];
    }

    public int EdgeSize => 1 << Log;
    public int TileX => Key3d.X * EdgeSize;
    public int TileY => Key3d.Y * EdgeSize;
    public int TileZ => Key3d.Z * EdgeSize;
    public int TileRight => TileX + EdgeSize;
    public int TileTop => TileY + EdgeSize;
    public int TileFront => TileZ + EdgeSize;
    public int Count => Storage.Length;
    public Vector3di TileMin => new(TileX, TileY, TileZ);
    public Vector3di TileMax => new(TileRight, TileTop, TileFront);
    public Vector3 WorldCenter => new(TileX + EdgeSize / 2f, TileY + EdgeSize / 2f, TileZ + EdgeSize / 2f);
    public BoundingBox3di Bounds => new(TileMin, TileMax);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int ReduceLocal(int world) => Mathx.ReduceLocal(world, Log);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GridIndex(int xWorld, int yWorld, int zWorld)
    {
        var xGrid = ReduceLocal(xWorld);
        var yGrid = ReduceLocal(yWorld);
        var zGrid = ReduceLocal(zWorld);
        var edgeSize = EdgeSize;
        return xGrid + edgeSize * (yGrid + edgeSize * zGrid);
    }

    public T this[int tileX, int tileY, int tileZ]
    {
        get => Storage[GridIndex(tileX, tileY, tileZ)];
        set => Storage[GridIndex(tileX, tileY, tileZ)] = value;
    }

    public Vector3di Size => new(EdgeSize);

    public T this[Vector3di tile]
    {
        get => Storage[GridIndex(tile.X, tile.Y, tile.Z)];
        set => Storage[GridIndex(tile.X, tile.Y, tile.Z)] = value;
    }

    public Vector3di WorldPosition(int gridIndex) => new(
        gridIndex % EdgeSize + TileX,
        (gridIndex / EdgeSize) % EdgeSize + TileY,
        gridIndex / EdgeSize / EdgeSize + TileZ
    );
}

public interface IReadOnlyMultiMap<TKey, TValue>
{
    int Count { get; }
    IReadOnlyCollection<TKey> Keys { get; }
    IReadOnlySet<TValue> this[TKey k] { get; }
    bool ContainsKey(TKey k);
}

public interface IMultiMap<TKey, TValue> : IReadOnlyMultiMap<TKey, TValue>
{
    bool Add(TKey k, TValue v);
    bool Remove(TKey k);
    bool Remove(TKey k, TValue v);
    void Clear();
}

public sealed class HashMultiMap<TKey, TValue> : IMultiMap<TKey, TValue> where TKey : notnull
{
    public readonly Dictionary<TKey, HashSet<TValue>> Map = new();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private HashSet<TValue> Get(TKey k) => 
        Map.TryGetValue(k, out var set) 
            ? set 
            : new HashSet<TValue>().Also(s => Map.Add(k, s));

    public int Count => Map.Count;

    public IReadOnlyCollection<TKey> Keys => Map.Keys;

    public HashSet<TValue> this[TKey k] => Get(k);

    IReadOnlySet<TValue> IReadOnlyMultiMap<TKey, TValue>.this[TKey k] => Get(k);

    public bool Add(TKey k, TValue v) => this[k].Add(v);

    public HashSet<TValue>? Place(TKey key, HashSet<TValue> set)
    {
        if (!Map.Remove(key, out var old))
        {
            old = null;
        }

        Map.Add(key, set);

        return old;
    }

    public bool ContainsKey(TKey k)
    {
        if (!Map.TryGetValue(k, out var set))
        {
            return false;
        }

        return set.Count > 0;
    }

    public bool Remove(TKey k) => Map.Remove(k);

    public bool Remove(TKey k, [NotNullWhen(true)] out HashSet<TValue>? set) => Map.Remove(k, out set);

    public bool Remove(TKey k, TValue v) => Map.TryGetValue(k, out var set) && set.Remove(v);

    public void Clear()
    {
        Map.Clear();
    }
}

public interface IReadOnlyBiMap<TForward, TBackward>
{
    IReadOnlyDictionary<TForward, TBackward> Forward { get; }
    IReadOnlyDictionary<TBackward, TForward> Backward { get; }
    bool ContainsForward(TForward f);
    bool ContainsBackward(TBackward b);
}

public interface IBiMap<TForward, TBackward> : IReadOnlyBiMap<TForward, TBackward>
{
    void Associate(TForward f, TBackward b);
    bool Disassociate(TForward f, TBackward b);
    void Clear();
}

public class HashBiMap<TForward, TBackward> : IBiMap<TForward, TBackward> where TForward : notnull where TBackward : notnull
{
    private readonly Dictionary<TForward, TBackward> _forward = new();
    private readonly Dictionary<TBackward, TForward> _backward = new();

    public IReadOnlyDictionary<TForward, TBackward> Forward => _forward;
    public IReadOnlyDictionary<TBackward, TForward> Backward => _backward;
    
    public bool ContainsForward(TForward f) => Forward.ContainsKey(f);

    public bool ContainsBackward(TBackward b) => Backward.ContainsKey(b);

    public void Associate(TForward f, TBackward b)
    {
        _forward.Add(f, b);
        _backward.Add(b, f);
    }

    public bool Disassociate(TForward f, TBackward b)
    {
        var removedF = _forward.Remove(f, out var actualBackward);
        var removedB = _backward.Remove(b, out var actualForward);

#if DEBUG
        Debug.Assert(removedF == removedB);

        if (removedF)
        {
            Debug.Assert(f.Equals(actualForward));
            Debug.Assert(b.Equals(actualBackward));
        }
#endif

        return removedF;
    }

    public void Clear()
    {
        _forward.Clear();
        _backward.Clear();
    }
}

public sealed class Histogram<TKey> where TKey : notnull
{
    public readonly Dictionary<TKey, int> Map = new();

    public Dictionary<TKey, int>.KeyCollection Keys => Map.Keys;

    public int Count => Map.Count;

    public int this[TKey k]
    {
        get => Map.TryGetValue(k, out var v) ? v : 0;
        set => Map[k] = value;
    }
}

public struct Average
{
    public int Count { get; private set; }
    public double Value { get; private set; }

    public void Add(double sample)
    {
        Value += (sample - Value) / (Count + 1);
        ++Count;
    }
}

public struct Average2d
{
    public int Count { get; private set; }
    public double ValueX { get; private set; }
    public double ValueY { get; private set; }
  
    public readonly Vector2d Value => new(ValueX, ValueY);

    public void Add(double x, double y)
    {
        ValueX += (x - ValueX) / (Count + 1);
        ValueY += (y - ValueY) / (Count + 1);
        ++Count;
    }

    public void Add(Vector2d value) => Add(value.X, value.Y);
}