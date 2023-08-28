using System.Numerics;

namespace Common;

public enum Direction2d : byte
{
    U = 0,
    D = 1,
    L = 2,
    R = 3
}

public enum Direction3d : byte
{
    L,
    R,
    U,
    D,
    F,
    B
}

[Flags]
public enum Direction3dMask : byte
{
    None = 0,
    L = 1 << 0,
    R = 1 << 1,
    U = 1 << 2,
    D = 1 << 3,
    F = 1 << 4,
    B = 1 << 5,
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

    public Vector2ds Floor() => new((int)Math.Floor(X), (int)Math.Floor(Y));
    public Vector2ds Round() => new((int)Math.Round(X), (int)Math.Round(Y));
    public Vector2ds Ceiling() => new((int)Math.Ceiling(X), (int)Math.Ceiling(Y));

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

    public static implicit operator Vector2d(Vector2 v) => new(v.X, v.Y);
    public static implicit operator Vector2(Vector2d v) => new((float)v.X, (float)v.Y);
}

public readonly struct Vector2ds : IComparable<Vector2ds>
{
    public static readonly Vector2ds Zero = new(0, 0);
    public static readonly Vector2ds UnitX = new(1, 0);
    public static readonly Vector2ds UnitY = new(0, 1);
    public static readonly Vector2ds One = new(1, 1);

    public short X { get; init; }
    public short Y { get; init; }
    public int NormSqr => X * X + Y * Y;
    public double Norm => Math.Sqrt(NormSqr);
    public float NormF => MathF.Sqrt(NormSqr);

    private static readonly Direction2d[] Directions = Enum.GetValues<Direction2d>();

    public Vector2ds(short x, short y)
    {
        X = x;
        Y = y;
    }

    public Vector2ds(int x, int y)
    {
        X = (short)x;
        Y = (short)y;
    }

    public override string ToString() => $"X={X}, Y={Y}";

    public override bool Equals(object? obj) => obj is Vector2ds v && this.Equals(v);

    public bool Equals(Vector2ds other) => X == other.X && Y == other.Y;

    public override int GetHashCode() => HashCode.Combine(X, Y);

    public Direction2d DirectionTo(Vector2ds b)
    {
        if (this == b)
        {
            throw new ArgumentException("Cannot get direction to same point", nameof(b));
        }

        var a = this;

        return Directions.MinBy(n => DistanceSqr(a + n, b));
    }

    public static Vector2ds operator +(Vector2ds a, Vector2ds b) => new(a.X + b.X, a.Y + b.Y);
    public static Vector2ds operator +(Vector2ds a, Direction2d d) => a + d.Step();
    public static Vector2ds operator -(Vector2ds a, Vector2ds b) => new(a.X - b.X, a.Y - b.Y);
    public static Vector2ds operator -(Vector2ds a, Direction2d d) => a - d.Step();
    public static Vector2ds operator /(Vector2ds a, Vector2ds b) => new(a.X / b.X, a.Y / b.Y);
    public static Vector2ds operator /(Vector2ds a, int s) => new(a.X / s, a.Y / s);
    public static Vector2ds operator *(Vector2ds a, Vector2ds b) => new(a.X * b.X, a.Y * b.Y);
    public static Vector2ds operator *(Vector2ds a, int s) => new(a.X * s, a.Y * s);
   
    public static bool operator ==(Vector2ds a, Vector2ds b) => a.Equals(b);
    public static bool operator !=(Vector2ds a, Vector2ds b) => !a.Equals(b);

    public static int DistanceSqr(Vector2ds a, Vector2ds b) => (a - b).NormSqr;
    public static double Distance(Vector2ds a, Vector2ds b) => (a - b).Norm;
    public static float DistanceF(Vector2ds a, Vector2ds b) => (a - b).NormF;
    public static int Manhattan(Vector2ds a, Vector2ds b) => Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);

    public int CompareTo(Vector2ds other) => this.NormSqr.CompareTo(other.NormSqr);

    public static Vector2d BarycenterMany(IEnumerable<Vector2ds> points)
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

public readonly struct Vector3di : IComparable<Vector3di>
{
    public static readonly Vector3di Zero = new(0, 0, 0);
    public static readonly Vector3di UnitX = new(1, 0, 0);
    public static readonly Vector3di UnitY = new(0, 1, 0);
    public static readonly Vector3di UnitZ = new(0, 0, 1);
    public static readonly Vector3di One = new(1, 1, 1);

    public int X { get; init; }
    public int Y { get; init; }
    public int Z { get; init; }

    public int NormSqr => X * X + Y * Y + Z * Z;
    public double Norm => Math.Sqrt(NormSqr);
    public float NormF => MathF.Sqrt(NormSqr);

    public Vector3di(int x, int y, int z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    public Vector3di(int value)
    {
        X = value;
        Y = value;
        Z = value;
    }

    public override string ToString() => $"X={X}, Y={Y}, Z={Z}";

    public override bool Equals(object? obj) => obj is Vector3di v && this.Equals(v);

    public bool Equals(Vector3di other) => X == other.X && Y == other.Y && Z == other.Z;

    public override int GetHashCode() => HashCode.Combine(X, Y, Z);

    public int CompareTo(Vector3di other) => NormSqr.CompareTo(other.NormSqr);

    public static Vector3di operator +(Vector3di a) => a;
    public static Vector3di operator -(Vector3di a) => new(-a.X, -a.Y, -a.Z);
    public static Vector3di operator ~(Vector3di a) => new(~a.X, ~a.Y, ~a.Z);
    public static Vector3di operator +(Vector3di a, Vector3di b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static Vector3di operator +(Vector3di a, int b) => new(a.X + b, a.Y + b, a.Z + b);
    public static Vector3di operator -(Vector3di a, Vector3di b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static Vector3di operator -(Vector3di a, int b) => new(a.X - b, a.Y - b, a.Z - b);
    public static Vector3di operator *(Vector3di a, Vector3di b) => new(a.X * b.X, a.Y * b.Y, a.Z * b.Z);
    public static Vector3di operator *(Vector3di a, int scalar) => new(a.X * scalar, a.Y * scalar, a.Z * scalar);
    public static Vector3di operator /(Vector3di a, Vector3di b) => new(a.X / b.X, a.Y / b.Y, a.Z / b.Z);
    public static Vector3di operator /(Vector3di a, int scalar) => new(a.X / scalar, a.Y / scalar, a.Z / scalar);
    public static Vector3di operator <<(Vector3di a, int shl) => new(a.X << shl, a.Y << shl, a.Z << shl);
    public static Vector3di operator >>(Vector3di a, int shr) => new(a.X >> shr, a.Y >> shr, a.Z >> shr);
    public static Vector3di operator <<(Vector3di a, Vector3di b) => new(a.X << b.X, a.Y << b.Y, a.Z << b.Z);
    public static Vector3di operator >>(Vector3di a, Vector3di b) => new(a.X >> b.X, a.Y >> b.Y, a.Z >> b.Z);
    public static Vector3di operator &(Vector3di a, int b) => new(a.X & b, a.Y & b, a.Z & b);
    public static Vector3di operator &(Vector3di a, Vector3di b) => new(a.X & b.X, a.Y & b.Y, a.Z & b.Z);
    public static Vector3di operator |(Vector3di a, int b) => new(a.X | b, a.Y | b, a.Z | b);
    public static Vector3di operator |(Vector3di a, Vector3di b) => new(a.X | b.X, a.Y | b.Y, a.Z | b.Z);
    public static Vector3di operator ^(Vector3di a, int b) => new(a.X ^ b, a.Y ^ b, a.Z ^ b);
    public static Vector3di operator ^(Vector3di a, Vector3di b) => new(a.X ^ b.X, a.Y ^ b.Y, a.Z ^ b.Z);
    public static Vector3di operator %(Vector3di a, int b) => new(a.X % b, a.Y % b, a.Z % b);
    public static Vector3di operator %(Vector3di a, Vector3di b) => new(a.X % b.X, a.Y % b.Y, a.Z % b.Z);
    public static bool operator <(Vector3di a, int b) => a.X < b && a.Y < b && a.Z < b;
    public static bool operator >(Vector3di a, int b) => a.X > b && a.Y > b && a.Z > b;
    public static bool operator <=(Vector3di a, int b) => a.X <= b && a.Y <= b && a.Z <= b;
    public static bool operator >=(Vector3di a, int b) => a.X >= b && a.Y >= b && a.Z >= b;

    public static bool operator ==(Vector3di a, Vector3di b) => a.Equals(b);
    public static bool operator !=(Vector3di a, Vector3di b) => !a.Equals(b);

    public static implicit operator Vector3(Vector3di v) => new(v.X, v.Y, v.Z);
    public static explicit operator Vector3di(Vector3 v) => new((int)v.X, (int)v.Y, (int)v.Z);

    public static int DistanceSqr(Vector3di a, Vector3di b) => (a - b).NormSqr;
    public static double Distance(Vector3di a, Vector3di b) => (a - b).Norm;
    public static float DistanceF(Vector3di a, Vector3di b) => (a - b).NormF;
    public static int Manhattan(Vector3di a, Vector3di b) => Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y) + Math.Abs(a.Z - b.Z);
}

public readonly struct BoundingBox3di
{
    public Vector3di Min { get; init; }
    public Vector3di Max { get; init; }

    public BoundingBox3di(Vector3di min, Vector3di max)
    {
        Min = min;
        Max = max;
    }

    public bool Contains(Vector3di point) =>
        point.X >= Min.X && point.X <= Max.X &&
        point.Y >= Min.Y && point.Y <= Max.Y &&
        point.Z >= Min.Z && point.Z <= Max.Z;
}