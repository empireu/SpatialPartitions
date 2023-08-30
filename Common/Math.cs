using System.Runtime.CompilerServices;

namespace Common;

public static class Mathx
{
    public const double SnzEpsilonDouble = 2.2e-15;
    public const float SnzEpsilonFloat = 1.2e-6f;

    public static int Snz(double a) => a >= 0d ? 1 : -1;
    public static int Snz(float a) => a >= 0f ? 1 : -1;
    public static int Snz(int a) => a >= 0 ? 1 : -1;
    public static int NSnz(double a) => a <= 0d ? -1 : 1;
    public static int NSnz(float a) => a <= 0f ? -1 : 1;
    public static int NSnz(int a) => a <= 0 ? -1 : 1;
    public static double SnzE(double a) => a >= 0.0 ? SnzEpsilonDouble : -SnzEpsilonDouble;
    public static float SnzE(float a) => a >= 0.0 ? SnzEpsilonFloat : -SnzEpsilonFloat;
    public static int MatchSign(int a, int targetSign) => Math.Sign(a) == targetSign ? 1 : 0;

    public static int Rsb(int a) => (a & -1) >>> 31;

    public static byte Rsb2(int a)
    {
        unchecked
        {
            var x = (uint)a & -1;
            return (byte)(x >> 31);
        }
    }


    public static double Lerp(double a, double b, double t) => a + ((b - a) * t);

    public static int ReduceLocal(int world, int log)
    {
        var size = 1 << log;
        var v = world - size * (world >> log);
        return (v + (v >> 31)) ^ (v >> 31);
    }
}