using System.Numerics;
using GameFramework.Extensions;
using GameFramework.Renderer;
using GameFramework.Renderer.Batch;
using Veldrid;
using Vortice.Mathematics;
using Rectangle = System.Drawing.Rectangle;

namespace Common;

public static class Extensions
{
    public static T Also<T>(this T t, Action<T> action)
    {
        action(t);
        return t;
    }

    public static T2 Map<T1, T2>(this T1 t, Func<T1, T2> m)
    {
        return m(t);
    }

    public static Vector2ds Step(this Base4Direction2d dir) => dir switch
    {
        Base4Direction2d.L => new Vector2ds(-1, 0),
        Base4Direction2d.R => new Vector2ds(1, 0),
        Base4Direction2d.U => new Vector2ds(0, -1),
        Base4Direction2d.D => new Vector2ds(0, 1),
        _ => throw new ArgumentOutOfRangeException(nameof(dir), dir, $"Unexpected direction {dir}")
    };

    public static Vector2ds Step(this Base8Direction2d dir) => dir switch
    {
        Base8Direction2d.L => new Vector2ds(-1, 0),
        Base8Direction2d.R => new Vector2ds(1, 0),
        Base8Direction2d.U => new Vector2ds(0, -1),
        Base8Direction2d.D => new Vector2ds(0, 1),
        Base8Direction2d.LU => new Vector2ds(-1, -1),
        Base8Direction2d.RU => new Vector2ds(1, -1),
        Base8Direction2d.LD => new Vector2ds(-1, 1),
        Base8Direction2d.RD => new Vector2ds(1, 1),
        _ => throw new ArgumentOutOfRangeException(nameof(dir), dir, $"Unexpected direction {dir}")
    };

    public static Base6Direction3d Opposite(this Base6Direction3d dir) => dir switch
    {
        Base6Direction3d.L => Base6Direction3d.R,
        Base6Direction3d.R => Base6Direction3d.L,
        Base6Direction3d.U => Base6Direction3d.D,
        Base6Direction3d.D => Base6Direction3d.U,
        Base6Direction3d.F => Base6Direction3d.B,
        Base6Direction3d.B => Base6Direction3d.F,
        _ => throw new ArgumentOutOfRangeException(nameof(dir), dir, null)
    };

    public static Vector3di Step(this Base6Direction3d dir) => dir switch
    {
        Base6Direction3d.L => new Vector3di(-1, 0, 0),
        Base6Direction3d.R => new Vector3di(1, 0, 0),
        Base6Direction3d.U => new Vector3di(0, 1, 0),
        Base6Direction3d.D => new Vector3di(0, -1, 0),
        Base6Direction3d.F => new Vector3di(0, 0, -1),
        Base6Direction3d.B => new Vector3di(0, 0, 1),
        _ => throw new ArgumentOutOfRangeException(nameof(dir), dir, null)
    };

    public static Base6Direction3dMask Mask(this Base6Direction3d dir) => (Base6Direction3dMask)(1 << (int)dir);

    public static bool Contains(this Rectangle rect, Vector2ds p) => rect.Contains(p.X, p.Y);
    public static Vector2ds CenterI(this Rectangle rect) => new(rect.X + rect.Width / 2, rect.Y + rect.Height / 2);
    public static Vector2 Center(this Rectangle rect) => new(rect.X + rect.Width / 2f, rect.Y + rect.Height / 2f);

    public static bool ApproxEq(this float f, float other, float threshold = 1e-7f)
    {
        return Math.Abs(f - other) < threshold;
    }

    public static bool ApproxEq(this double d, double other, double threshold = 1e-7)
    {
        return Math.Abs(d - other) < threshold;
    }

    public static void Quad(this QuadBatch batch, Rectangle rect, RgbaFloat4 color)
    {
        batch.Quad(new Vector2(rect.X, -rect.Y), new Vector2(rect.Width, rect.Height), color, align: AlignMode.TopLeft);
    }

    private static readonly Matrix4x4 CubeQuadForward = Matrix4x4.CreateRotationY(0) * Matrix4x4.CreateTranslation(0, 0, 0.5f);
    private static readonly Matrix4x4 CubeQuadBackward = Matrix4x4.CreateRotationY(MathF.PI) * Matrix4x4.CreateTranslation(0, 0, -0.5f);
    private static readonly Matrix4x4 CubeQuadLeft = Matrix4x4.CreateRotationY(-MathF.PI / 2) * Matrix4x4.CreateTranslation(-0.5f, 0, 0);
    private static readonly Matrix4x4 CubeQuadRight = Matrix4x4.CreateRotationY(MathF.PI / 2) * Matrix4x4.CreateTranslation(0.5f, 0, 0);
    private static readonly Matrix4x4 CubeQuadUp = Matrix4x4.CreateRotationX(-MathF.PI / 2) * Matrix4x4.CreateTranslation(0, 0.5f, 0);
    private static readonly Matrix4x4 CubeQuadDown = Matrix4x4.CreateRotationX(MathF.PI / 2) * Matrix4x4.CreateTranslation(0, -0.5f, 0);

    // Uses 6 quads
    public static void ColoredQuadBox(
        this QuadBatch batch, 
        Matrix4x4 transform,
        QuadColors forward,
        QuadColors backward,
        QuadColors left,
        QuadColors right,
        QuadColors up,
        QuadColors down)
    {
        batch.Quad(CubeQuadForward * transform, forward);
        batch.Quad(CubeQuadBackward * transform, backward);
        batch.Quad(CubeQuadLeft * transform, left);
        batch.Quad(CubeQuadRight * transform, right);
        batch.Quad(CubeQuadUp * transform, up);
        batch.Quad(CubeQuadDown * transform, down);
    }

    public static void ColoredQuadBox(
        this QuadBatch batch,
        Matrix4x4 transform,
        QuadColors forward,
        QuadColors backward,
        QuadColors left,
        QuadColors right,
        QuadColors up,
        QuadColors down, 
        Base6Direction3dMask mask)
    {
        if (mask.HasFlag(Base6Direction3dMask.F))
        {
            batch.Quad(CubeQuadForward * transform, forward);
        }

        if (mask.HasFlag(Base6Direction3dMask.B))
        {
            batch.Quad(CubeQuadBackward * transform, backward);
        }

        if (mask.HasFlag(Base6Direction3dMask.L))
        {
            batch.Quad(CubeQuadLeft * transform, left);
        }

        if (mask.HasFlag(Base6Direction3dMask.R))
        {
            batch.Quad(CubeQuadRight * transform, right);
        }

        if (mask.HasFlag(Base6Direction3dMask.U))
        {
            batch.Quad(CubeQuadUp * transform, up);
        }

        if (mask.HasFlag(Base6Direction3dMask.D))
        {
            batch.Quad(CubeQuadDown * transform, down);
        }
    }

    public static void ColoredQuadBox(this QuadBatch batch, Matrix4x4 transform, QuadColors color)
    {
        ColoredQuadBox(batch, transform, color, color, color, color, color, color);
    }

    public static void ColoredQuadBox(this QuadBatch batch, Vector3 min, Vector3 max, QuadColors color)
    {
        ColoredQuadBox(batch, Matrix4x4.CreateScale(max - min) with { Translation = (min + max) / 2f }, color, color, color, color, color, color);
    }


    public static void ColoredQuadBox(this QuadBatch batch, Matrix4x4 transform, QuadColors color, Base6Direction3dMask mask)
    {
        ColoredQuadBox(batch, transform, color, color, color, color, color, color, mask);
    }

    // Uses 56 quads
    public static void ColoredQuadBoxFrame(this QuadBatch batch, Vector3 min, Vector3 max, float thickness, QuadColors colors)
    {
        var a = Matrix4x4.CreateScale(max.X - min.X - thickness, thickness, thickness) * 
                Matrix4x4.CreateTranslation(min with { X = (min.X + max.X) / 2f });
        
        var b = Matrix4x4.CreateScale(thickness, thickness, max.Z - min.Z - thickness) * 
                Matrix4x4.CreateTranslation(min with { Z = (min.Z + max.Z) / 2f });
        
        var c = Matrix4x4.CreateScale(max.X - min.X, thickness, thickness) *
                Matrix4x4.CreateTranslation(min with { X = (min.X + max.X) / 2f, Z = max.Z });
      
        var d = Matrix4x4.CreateScale(thickness, thickness, max.Z - min.Z - thickness) *
                Matrix4x4.CreateTranslation(min with { Z = (min.Z + max.Z) / 2f, X = max.X });

        const Base6Direction3dMask ex = Base6Direction3dMask.F | Base6Direction3dMask.B | Base6Direction3dMask.U | Base6Direction3dMask.D;
        const Base6Direction3dMask ez = Base6Direction3dMask.L | Base6Direction3dMask.R | Base6Direction3dMask.U | Base6Direction3dMask.D;

        // Lower horizontal edges:
        ColoredQuadBox(batch, a, colors, ex);
        ColoredQuadBox(batch, b, colors, ez);
        ColoredQuadBox(batch, c, colors, ex);
        ColoredQuadBox(batch, d, colors, ez);

        // Upper horizontal edges:
        ColoredQuadBox(batch, a with { Translation = a.Translation with { Y = max.Y } }, colors, ex);
        ColoredQuadBox(batch, b with { Translation = b.Translation with { Y = max.Y } }, colors, ez);
        ColoredQuadBox(batch, c with { Translation = c.Translation with { Y = max.Y } }, colors, ex);
        ColoredQuadBox(batch, d with { Translation = d.Translation with { Y = max.Y } }, colors, ez);

        // Vertical edges:
        var edgeScale = Matrix4x4.CreateScale(thickness, max.Y - min.Y + thickness, thickness);
        var edgeY = (min.Y + max.Y) / 2f;
        ColoredQuadBox(batch, edgeScale * Matrix4x4.CreateTranslation(min with { Y = edgeY }), colors);
        ColoredQuadBox(batch, edgeScale * Matrix4x4.CreateTranslation(min with { Y = edgeY, Z = max.Z }), colors);
        ColoredQuadBox(batch, edgeScale * Matrix4x4.CreateTranslation(min with { Y = edgeY, X = max.X }), colors);
        ColoredQuadBox(batch, edgeScale * Matrix4x4.CreateTranslation(min with { Y = edgeY, X = max.X, Z = max.Z }), colors);
    }

    // Debug only
    public static string Bits(this ulong u)
    {
        var str = "";

        while (u > 0)
        {
            str = (u & 1) + str;
            u >>= 1;
        }

        return str;
    }

    public static IEnumerable<Vector3di> EnumerateFace(this BoundingBox3di box, Base6Direction3d face)
    {
        switch (face)
        {
            case Base6Direction3d.L:
                for (var z = box.Min.Z; z <= box.Max.Z; z++)
                {
                    for (var y = box.Min.Y; y <= box.Max.Y; y++)
                    {
                        yield return new Vector3di(box.Min.X, y, z);
                    }
                }
                break;
            case Base6Direction3d.R:
                for (var z = box.Min.Z; z <= box.Max.Z; z++)
                {
                    for (var y = box.Min.Y; y <= box.Max.Y; y++)
                    {
                        yield return new Vector3di(box.Max.X, y, z);
                    }
                }
                break;
            case Base6Direction3d.U:
                for (var z = box.Min.Z; z <= box.Max.Z; z++)
                {
                    for (var x = box.Min.X; x <= box.Max.X; x++)
                    {
                        yield return new Vector3di(x, box.Max.Y, z);
                    }
                }
                break;
            case Base6Direction3d.D:
                for (var z = box.Min.Z; z <= box.Max.Z; z++)
                {
                    for (var x = box.Min.X; x <= box.Max.X; x++)
                    {
                        yield return new Vector3di(x, box.Min.Y, z);
                    }
                }
                break;
            case Base6Direction3d.F:
                for (var y = box.Min.Y; y <= box.Max.Y; y++)
                {
                    for (var x = box.Min.X; x <= box.Max.X; x++)
                    {
                        yield return new Vector3di(x, y, box.Min.Z);
                    }
                }
                break;
            case Base6Direction3d.B:
                for (var y = box.Min.Y; y <= box.Max.Y; y++)
                {
                    for (var x = box.Min.X; x <= box.Max.X; x++)
                    {
                        yield return new Vector3di(x, y, box.Max.Z);
                    }
                }
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(face), face, null);
        }
    }

    public static IEnumerable<Vector3di> EnumerateFaceExclusive(this BoundingBox3di box, Base6Direction3d face)
    {
        switch (face)
        {
            case Base6Direction3d.L:
                for (var z = box.Min.Z; z < box.Max.Z; z++)
                {
                    for (var y = box.Min.Y; y < box.Max.Y; y++)
                    {
                        yield return new Vector3di(box.Min.X, y, z);
                    }
                }
                break;
            case Base6Direction3d.R:
                for (var z = box.Min.Z; z < box.Max.Z; z++)
                {
                    for (var y = box.Min.Y; y < box.Max.Y; y++)
                    {
                        yield return new Vector3di(box.Max.X - 1, y, z);
                    }
                }
                break;
            case Base6Direction3d.U:
                for (var z = box.Min.Z; z < box.Max.Z; z++)
                {
                    for (var x = box.Min.X; x < box.Max.X; x++)
                    {
                        yield return new Vector3di(x, box.Max.Y - 1, z);
                    }
                }
                break;
            case Base6Direction3d.D:
                for (var z = box.Min.Z; z < box.Max.Z; z++)
                {
                    for (var x = box.Min.X; x < box.Max.X; x++)
                    {
                        yield return new Vector3di(x, box.Min.Y, z);
                    }
                }
                break;
            case Base6Direction3d.F:
                for (var y = box.Min.Y; y < box.Max.Y; y++)
                {
                    for (var x = box.Min.X; x < box.Max.X; x++)
                    {
                        yield return new Vector3di(x, y, box.Min.Z);
                    }
                }
                break;
            case Base6Direction3d.B:
                for (var y = box.Min.Y; y < box.Max.Y; y++)
                {
                    for (var x = box.Min.X; x < box.Max.X; x++)
                    {
                        yield return new Vector3di(x, y, box.Max.Z - 1);
                    }
                }
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(face), face, null);
        }
    }
    
    public static BoundingBox3di ExtrudeNormal(this BoundingBox3di box, Base6Direction3d direction, int height = 1)
    {
        var min = box.Min;
        var max = box.Max;

        return direction switch
        {
            Base6Direction3d.L => new BoundingBox3di(
                new Vector3di(min.X - height, min.Y, min.Z),
                new Vector3di(min.X, max.Y, max.Z)),
            Base6Direction3d.R => new BoundingBox3di(
                new Vector3di(max.X, min.Y, min.Z),
                new Vector3di(max.X + height, max.Y, max.Z)),
            Base6Direction3d.U => new BoundingBox3di(
                new Vector3di(min.X, max.Y, min.Z),
                new Vector3di(max.X, max.Y + height, max.Z)),
            Base6Direction3d.D => new BoundingBox3di(
                new Vector3di(min.X, min.Y - height, min.Z),
                new Vector3di(max.X, min.Y, max.Z)),
            Base6Direction3d.F => new BoundingBox3di(
                new Vector3di(min.X, min.Y, min.Z - height),
                new Vector3di(max.X, max.Y, min.Z)),
            Base6Direction3d.B => new BoundingBox3di(
                new Vector3di(min.X, min.Y, max.Z),
                new Vector3di(max.X, max.Y, max.Z + height)),
            _ => throw new ArgumentOutOfRangeException()
        };
    }

    public static BoundingBox3di SliceNormal(this BoundingBox3di box, Base6Direction3d direction, int height = 1)
    {
        var min = box.Min;
        var max = box.Max;

        return direction switch
        {
            Base6Direction3d.L => new BoundingBox3di(
                new Vector3di(max.X - height, min.Y, min.Z),
                new Vector3di(max.X, max.Y, max.Z)),
            Base6Direction3d.R => new BoundingBox3di(
                new Vector3di(min.X, min.Y, min.Z),
                new Vector3di(min.X + height, max.Y, max.Z)),
            Base6Direction3d.U => new BoundingBox3di(
                new Vector3di(min.X, min.Y, min.Z),
                new Vector3di(max.X, min.Y + height, max.Z)),
            Base6Direction3d.D => new BoundingBox3di(
                new Vector3di(min.X, max.Y - height, min.Z),
                new Vector3di(max.X, max.Y, max.Z)),
            Base6Direction3d.F => new BoundingBox3di(
                new Vector3di(min.X, min.Y, max.Z - height),
                new Vector3di(max.X, max.Y, max.Z)),
            Base6Direction3d.B => new BoundingBox3di(
                new Vector3di(min.X, min.Y, min.Z),
                new Vector3di(max.X, max.Y, min.Z + height)),
            _ => throw new ArgumentOutOfRangeException()
        };
    }

    public static BoundingBox3di InnerFace(this BoundingBox3di box, Base6Direction3d direction)
    {
        var min = box.Min;
        var max = box.Max;

        return direction switch
        {
            Base6Direction3d.L => new BoundingBox3di(min, new Vector3di(min.X + 1, max.Y, max.Z)),
            Base6Direction3d.R => new BoundingBox3di(new Vector3di(max.X - 1, min.Y, min.Z), max),
            Base6Direction3d.U => new BoundingBox3di(new Vector3di(min.X, max.Y - 1, min.Z), max),
            Base6Direction3d.D => new BoundingBox3di(min, new Vector3di(max.X, min.Y + 1, max.Z)),
            Base6Direction3d.F => new BoundingBox3di(min, new Vector3di(max.X, max.Y, min.Z + 1)),
            Base6Direction3d.B => new BoundingBox3di(new Vector3di(min.X, min.Y, max.Z - 1), max),
            _ => throw new ArgumentOutOfRangeException()
        };
    }


    public static BoundingBox3di TranslateNormal(this BoundingBox3di box, Base6Direction3d direction, int distance = 1)
    {
        var step = direction.Step() * distance;
        return new BoundingBox3di(box.Min + step, box.Max + step);
    }

    public static int SizeAlong(this BoundingBox3di box, Base6Direction3d direction) => direction switch
    {
        Base6Direction3d.L => box.Max.X - box.Min.X,
        Base6Direction3d.R => box.Max.X - box.Min.X,
        Base6Direction3d.U => box.Max.Y - box.Min.Y,
        Base6Direction3d.D => box.Max.Y - box.Min.Y,
        Base6Direction3d.F => box.Max.Z - box.Min.Z,
        Base6Direction3d.B => box.Max.Z - box.Min.Z,
        _ => throw new ArgumentOutOfRangeException(nameof(direction), direction, null)
    };
}