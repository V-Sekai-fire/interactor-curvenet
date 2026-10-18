// Minimal stand-ins for the engine math types CASSIE's graph and curve code use, in 32-bit float.
using System;
using System.Collections.Generic;

namespace UnityEngine
{
    public struct Vector3 : IEquatable<Vector3>
    {
        public float x, y, z;
        public Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public Vector3(float x, float y) { this.x = x; this.y = y; z = 0f; }
        public static Vector3 zero => new Vector3(0f, 0f, 0f);
        public static Vector3 one => new Vector3(1f, 1f, 1f);
        public static Vector3 up => new Vector3(0f, 1f, 0f);
        public static Vector3 down => new Vector3(0f, -1f, 0f);
        public static Vector3 right => new Vector3(1f, 0f, 0f);
        public static Vector3 left => new Vector3(-1f, 0f, 0f);
        public static Vector3 forward => new Vector3(0f, 0f, 1f);
        public static Vector3 back => new Vector3(0f, 0f, -1f);
        public static Vector3 positiveInfinity => new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
        public float this[int i]
        {
            get { return i == 0 ? x : i == 1 ? y : z; }
            set { if (i == 0) x = value; else if (i == 1) y = value; else z = value; }
        }
        public float sqrMagnitude => x * x + y * y + z * z;
        public float magnitude => (float)Math.Sqrt(x * x + y * y + z * z);
        public static float Magnitude(Vector3 v) => v.magnitude;
        public static float SqrMagnitude(Vector3 v) => v.sqrMagnitude;
        public Vector3 normalized => Normalize(this);
        public void Normalize() { this = Normalize(this); }
        public static Vector3 Normalize(Vector3 v)
        {
            float m = v.magnitude;
            return m > 1e-5f ? v / m : zero;
        }
        public static float Dot(Vector3 a, Vector3 b) => a.x * b.x + a.y * b.y + a.z * b.z;
        public static Vector3 Cross(Vector3 a, Vector3 b) =>
            new Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
        public static float Distance(Vector3 a, Vector3 b) => (a - b).magnitude;
        public static Vector3 Lerp(Vector3 a, Vector3 b, float t) { t = Mathf.Clamp01(t); return a + (b - a) * t; }
        public static Vector3 LerpUnclamped(Vector3 a, Vector3 b, float t) => a + (b - a) * t;
        public static Vector3 Project(Vector3 v, Vector3 n)
        {
            float s = Dot(n, n);
            return s < Mathf.Epsilon ? zero : n * (Dot(v, n) / s);
        }
        public static Vector3 ProjectOnPlane(Vector3 v, Vector3 n)
        {
            float s = Dot(n, n);
            return s < Mathf.Epsilon ? v : v - n * (Dot(v, n) / s);
        }
        public static float Angle(Vector3 a, Vector3 b)
        {
            float d = (float)Math.Sqrt(a.sqrMagnitude * b.sqrMagnitude);
            if (d < 1e-15f) return 0f;
            float dot = Mathf.Clamp(Dot(a, b) / d, -1f, 1f);
            return (float)Math.Acos(dot) * Mathf.Rad2Deg;
        }
        public static float SignedAngle(Vector3 a, Vector3 b, Vector3 axis)
        {
            float u = Angle(a, b);
            float s = Math.Sign(Dot(axis, Cross(a, b)));
            return u * (s == 0 ? 1f : s);
        }
        public void Set(float nx, float ny, float nz) { x = nx; y = ny; z = nz; }
        public static Vector3 Reflect(Vector3 d, Vector3 n) => -2f * Dot(n, d) * n + d;
        public static Vector3 Min(Vector3 a, Vector3 b) => new Vector3(Math.Min(a.x, b.x), Math.Min(a.y, b.y), Math.Min(a.z, b.z));
        public static Vector3 Max(Vector3 a, Vector3 b) => new Vector3(Math.Max(a.x, b.x), Math.Max(a.y, b.y), Math.Max(a.z, b.z));
        public static Vector3 Scale(Vector3 a, Vector3 b) => new Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
        public static Vector3 operator +(Vector3 a, Vector3 b) => new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
        public static Vector3 operator -(Vector3 a, Vector3 b) => new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
        public static Vector3 operator -(Vector3 a) => new Vector3(-a.x, -a.y, -a.z);
        public static Vector3 operator *(Vector3 a, float d) => new Vector3(a.x * d, a.y * d, a.z * d);
        public static Vector3 operator *(float d, Vector3 a) => new Vector3(a.x * d, a.y * d, a.z * d);
        public static Vector3 operator /(Vector3 a, float d) => new Vector3(a.x / d, a.y / d, a.z / d);
        // Equality is within a tolerance, as the original engine defines it.
        public static bool operator ==(Vector3 a, Vector3 b) => (a - b).sqrMagnitude < 9.99999944E-11f;
        public static bool operator !=(Vector3 a, Vector3 b) => !(a == b);
        public bool Equals(Vector3 o) => x.Equals(o.x) && y.Equals(o.y) && z.Equals(o.z);
        public override bool Equals(object o) => o is Vector3 v && Equals(v);
        public override int GetHashCode() => x.GetHashCode() ^ (y.GetHashCode() << 2) ^ (z.GetHashCode() >> 2);
        public override string ToString() => $"({x:F5}, {y:F5}, {z:F5})";
        public string ToString(string f) => $"({x.ToString(f)}, {y.ToString(f)}, {z.ToString(f)})";
    }

    public struct Vector2
    {
        public float x, y;
        public Vector2(float x, float y) { this.x = x; this.y = y; }
        public static Vector2 operator -(Vector2 a, Vector2 b) => new Vector2(a.x - b.x, a.y - b.y);
        public static Vector2 operator +(Vector2 a, Vector2 b) => new Vector2(a.x + b.x, a.y + b.y);
        public static Vector2 operator *(Vector2 a, float d) => new Vector2(a.x * d, a.y * d);
        public float magnitude => (float)Math.Sqrt(x * x + y * y);
        public static float Dot(Vector2 a, Vector2 b) => a.x * b.x + a.y * b.y;
    }

    public struct Quaternion
    {
        public float x, y, z, w;
        public Quaternion(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }
        public static Quaternion identity => new Quaternion(0f, 0f, 0f, 1f);
        public static Quaternion AngleAxis(float angle, Vector3 axis)
        {
            axis = axis.normalized;
            float h = angle * Mathf.Deg2Rad * 0.5f;
            float s = (float)Math.Sin(h);
            return new Quaternion(axis.x * s, axis.y * s, axis.z * s, (float)Math.Cos(h));
        }
        public static Quaternion Inverse(Quaternion q) => new Quaternion(-q.x, -q.y, -q.z, q.w);
        public static Vector3 operator *(Quaternion r, Vector3 p)
        {
            float x = r.x * 2f, y = r.y * 2f, z = r.z * 2f;
            float xx = r.x * x, yy = r.y * y, zz = r.z * z, xy = r.x * y, xz = r.x * z, yz = r.y * z;
            float wx = r.w * x, wy = r.w * y, wz = r.w * z;
            return new Vector3((1f - (yy + zz)) * p.x + (xy - wz) * p.y + (xz + wy) * p.z,
                    (xy + wz) * p.x + (1f - (xx + zz)) * p.y + (yz - wx) * p.z,
                    (xz - wy) * p.x + (yz + wx) * p.y + (1f - (xx + yy)) * p.z);
        }
    }

    public static class Mathf
    {
        public const float PI = (float)Math.PI;
        public const float Infinity = float.PositiveInfinity;
        public const float NegativeInfinity = float.NegativeInfinity;
        public const float Deg2Rad = PI / 180f;
        public const float Rad2Deg = 57.29578f;
        public static readonly float Epsilon = 1.401298E-45f;
        public static float Abs(float f) => Math.Abs(f);
        public static int Abs(int f) => Math.Abs(f);
        public static float Sqrt(float f) => (float)Math.Sqrt(f);
        public static float Pow(float f, float p) => (float)Math.Pow(f, p);
        public static float Sin(float f) => (float)Math.Sin(f);
        public static float Cos(float f) => (float)Math.Cos(f);
        public static float Acos(float f) => (float)Math.Acos(f);
        public static float Atan2(float y, float x) => (float)Math.Atan2(y, x);
        public static float Min(float a, float b) => a < b ? a : b;
        public static float Max(float a, float b) => a > b ? a : b;
        public static int Min(int a, int b) => a < b ? a : b;
        public static int Max(int a, int b) => a > b ? a : b;
        public static float Min(params float[] v) { float m = v[0]; foreach (float f in v) if (f < m) m = f; return m; }
        public static float Max(params float[] v) { float m = v[0]; foreach (float f in v) if (f > m) m = f; return m; }
        public static float Clamp(float v, float a, float b) => v < a ? a : (v > b ? b : v);
        public static int Clamp(int v, int a, int b) => v < a ? a : (v > b ? b : v);
        public static float Clamp01(float v) => v < 0f ? 0f : (v > 1f ? 1f : v);
        public static int CeilToInt(float f) => (int)Math.Ceiling(f);
        public static int FloorToInt(float f) => (int)Math.Floor(f);
        public static int RoundToInt(float f) => (int)Math.Round(f);
        public static float Floor(float f) => (float)Math.Floor(f);
        public static float Ceil(float f) => (float)Math.Ceiling(f);
        public static float Round(float f) => (float)Math.Round(f);
        public static float Sign(float f) => f >= 0f ? 1f : -1f;
        public static float Lerp(float a, float b, float t) => a + (b - a) * Clamp01(t);
        public static float Log(float f) => (float)Math.Log(f);
        public static bool Approximately(float a, float b) =>
            Math.Abs(b - a) < Math.Max(1E-06f * Math.Max(Math.Abs(a), Math.Abs(b)), Epsilon * 8f);
    }

    public static class Debug
    {
        public static bool Verbose = false;
        public static void Log(object m) { if (Verbose) Console.Error.WriteLine(m); }
        public static void LogWarning(object m) { if (Verbose) Console.Error.WriteLine("W " + m); }
        public static void LogError(object m) { Console.Error.WriteLine("E " + m); }
        public static void DrawLine(Vector3 a, Vector3 b) { }
        public static void DrawLine(Vector3 a, Vector3 b, Color c) { }
        public static void DrawLine(Vector3 a, Vector3 b, Color c, float d) { }
        public static void DrawRay(Vector3 a, Vector3 b, Color c, float d) { }
    }

    public struct Color
    {
        public float r, g, b, a;
        public Color(float r, float g, float b, float a = 1f) { this.r = r; this.g = g; this.b = b; this.a = a; }
        public static Color blue => new Color(0, 0, 1);
        public static Color red => new Color(1, 0, 0);
        public static Color green => new Color(0, 1, 0);
        public static Color yellow => new Color(1, 1, 0);
        public static Color white => new Color(1, 1, 1);
        public static Color black => new Color(0, 0, 0);
        public static Color magenta => new Color(1, 0, 1);
        public static Color cyan => new Color(0, 1, 1);
    }

    public static class Time { public static float time = 0f; public static float deltaTime = 0f; }
    public static class Application { public static string dataPath = "."; public static string persistentDataPath = "."; public static string streamingAssetsPath = "."; }
    public class Transform { public Vector3 InverseTransformPoint(Vector3 p) => p; public Vector3 TransformPoint(Vector3 p) => p; }
    public class GameObject { public Transform transform = new Transform(); }
    public class Object { }
    public class Component : Object { }
    public class MonoBehaviour : Component { public GameObject gameObject = new GameObject(); public Transform transform = new Transform(); }
}
