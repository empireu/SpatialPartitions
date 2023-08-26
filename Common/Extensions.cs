using System.Numerics;
using GameFramework.Extensions;
using GameFramework.Renderer;
using GameFramework.Renderer.Batch;
using Veldrid;
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

    public static Vector2di Step(this Direction2d dir) => dir switch
    {
        Direction2d.L => new Vector2di(-1, 0),
        Direction2d.R => new Vector2di(1, 0),
        Direction2d.U => new Vector2di(0, -1),
        Direction2d.D => new Vector2di(0, 1),
        _ => throw new ArgumentOutOfRangeException(nameof(dir), dir, $"Unexpected direction {dir}")
    };

    public static bool Contains(this Rectangle rect, Vector2di p) => rect.Contains(p.X, p.Y);
    public static Vector2di CenterI(this Rectangle rect) => new(rect.X + rect.Width / 2, rect.Y + rect.Height / 2);
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

    public static void ColoredQuadCube(
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

    public static void ColoredQuadCube(this QuadBatch batch, Matrix4x4 transform, QuadColors color)
    {
        ColoredQuadCube(batch, transform, color, color, color, color, color, color);
    }
}