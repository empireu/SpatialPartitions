using System.Drawing;
using System.Numerics;
using System.Runtime.Intrinsics.X86;

namespace Common;

public enum Direction2d : byte
{
    U = 0,
    D = 1,
    L = 2,
    R = 3
}

public readonly struct Vector2di : IComparable<Vector2di>
{
    public short X { get; init; }
    public short Y { get; init; }
    public int NormSqr => X * X + Y * Y;
    public double Norm => Math.Sqrt(NormSqr);
    public float NormF => MathF.Sqrt(NormSqr);

    private static readonly Direction2d[] Directions = Enum.GetValues<Direction2d>(); 

    public Vector2di(short x, short y)
    {
        X = x;
        Y = y;
    }

    public Vector2di(int x, int y)
    {
        X = (short)x;
        Y = (short)y;
    }

    public override string ToString() => $"X={X}, Y={Y}";

    public override bool Equals(object? obj) => obj is Vector2di v && this.Equals(v);

    public bool Equals(Vector2di other) => X == other.X && Y == other.Y;

    public override int GetHashCode() => HashCode.Combine(X, Y);

    public Direction2d DirectionTo(Vector2di b)
    {
        if (this == b)
        {
            throw new ArgumentException("Cannot get direction to same point", nameof(b));
        }

        var a = this;

        return Directions.MinBy(n => DistanceSqr(a + n, b));
    }

    public static Vector2di operator +(Vector2di a, Vector2di b) => new(a.X + b.X, a.Y + b.Y);
    public static Vector2di operator +(Vector2di a, Direction2d d) => a + d.Step();
    public static Vector2di operator -(Vector2di a, Vector2di b) => new(a.X - b.X, a.Y - b.Y);
    public static Vector2di operator -(Vector2di a, Direction2d d) => a - d.Step();
    public static Vector2di operator /(Vector2di a, Vector2di b) => new(a.X / b.X, a.Y / b.Y);
    public static Vector2di operator /(Vector2di a, int s) => new(a.X / s, a.Y / s);
    public static Vector2di operator *(Vector2di a, Vector2di b) => new(a.X * b.X, a.Y * b.Y);
    public static Vector2di operator *(Vector2di a, int s) => new(a.X * s, a.Y * s);
    public static bool operator ==(Vector2di a, Vector2di b) => a.Equals(b);
    public static bool operator !=(Vector2di a, Vector2di b) => !a.Equals(b);

    public static readonly Vector2di Zero = new(0, 0);
    public static readonly Vector2di UnitX = new(1, 0);
    public static readonly Vector2di UnitY = new(0, 1);
    public static readonly Vector2di One = new(1, 1);

    public static int DistanceSqr(Vector2di a, Vector2di b) => (a - b).NormSqr;
    public static double Distance(Vector2di a, Vector2di b) => (a - b).Norm;
    public static float DistanceF(Vector2di a, Vector2di b) => (a - b).NormF;
    public static int Manhattan(Vector2di a, Vector2di b) => Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);

    public int CompareTo(Vector2di other) => this.NormSqr.CompareTo(other.NormSqr);

    public static Vector2d BarycenterMany(IEnumerable<Vector2di> points)
    {
        var x = 0d;
        var y = 0d;

        var i = 0;

        foreach (var p in points)
        {
            x += (p.X - x) / (i + 1);
            y += (p.Y - y) / (i + 1);
            i++;
        }

        return new Vector2d(x, y);
    }
}

public readonly struct Vector2d
{
    public static readonly Vector2d Zero = new(0, 0);
    public static readonly Vector2d One = new(1, 1);
    public static readonly Vector2d UnitX = new(1, 0);
    public static readonly Vector2d UnitY = new(0, 1);

    public double X { get; }
    public double Y { get; }

    public Vector2d(double x, double y)
    {
        X = x;
        Y = y;
    }

    public Vector2d(double value) : this(value, value)
    {

    }

    // We may consider using functions here. Normalized() is already using it.
    public double NormSqr => X * X + Y * Y;
    public double Norm => Math.Sqrt(NormSqr);
    public Vector2d Normalized() => this / Norm;

    public override bool Equals(object? obj)
    {
        if (obj is not Vector2d other)
        {
            return false;
        }

        return Equals(other);
    }

    public bool Equals(Vector2d other) => X.Equals(other.X) && Y.Equals(other.Y);

    public bool ApproxEq(Vector2d other, double eps = 1e-7) => X.ApproxEq(other.X, eps) && Y.ApproxEq(other.Y, eps);

    public override int GetHashCode() => HashCode.Combine(X, Y);

    public override string ToString() => $"X={X}, Y={Y}";

    public Vector2di Floor() => new((int)Math.Floor(X), (int)Math.Floor(Y));
    public Vector2di Round() => new((int)Math.Round(X), (int)Math.Round(Y));
    public Vector2di Ceiling() => new((int)Math.Ceiling(X), (int)Math.Ceiling(Y));

    public static bool operator ==(Vector2d a, Vector2d b) => a.Equals(b);
    public static bool operator !=(Vector2d a, Vector2d b) => !a.Equals(b);
    public static Vector2d operator +(Vector2d v) => new(+v.X, +v.Y);
    public static Vector2d operator -(Vector2d v) => new(-v.X, -v.Y);
    public static Vector2d operator +(Vector2d a, Vector2d b) => new(a.X + b.X, a.Y + b.Y);
    public static Vector2d operator -(Vector2d a, Vector2d b) => new(a.X - b.X, a.Y - b.Y);
    public static Vector2d operator *(Vector2d a, Vector2d b) => new(a.X * b.X, a.Y * b.Y);
    public static Vector2d operator /(Vector2d a, Vector2d b) => new(a.X / b.X, a.Y / b.Y);
    public static Vector2d operator *(Vector2d v, double scalar) => new(v.X * scalar, v.Y * scalar);
    public static Vector2d operator /(Vector2d v, double scalar) => new(v.X / scalar, v.Y / scalar);
    public static implicit operator Vector2(Vector2d v) => new((float)v.X, (float)v.Y);
    public static implicit operator Vector2d(Vector2 v) => new(v.X, v.Y);
}