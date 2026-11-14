// Monster - small vector, quaternion and matrix types
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include <cmath>

// Struct Vec3
struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;
    Vec3(double x, double y, double z)
        :
        x(x)
        , y(y)
        , z(z)
    {
    }

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator-() const { return Vec3(-x, -y, -z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
};

inline double dot(const Vec3& a, const Vec3& b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b)
{
    return Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

inline double length(const Vec3& v)
{
    return std::sqrt(dot(v, v));
}

// Returns the fallback when v is too short to normalize
inline Vec3 normalize(const Vec3& v, const Vec3& fallback = Vec3(0.0, 1.0, 0.0))
{
    const double len = length(v);
    return len > 1.0e-9 ? v * (1.0 / len) : fallback;
}

inline double clamp(double value, double lo, double hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

// Struct Quat (unit quaternion, rotates body coordinates to world coordinates)
struct Quat
{
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Quat() = default;
    Quat(double w, double x, double y, double z)
        :
        w(w)
        , x(x)
        , y(y)
        , z(z)
    {
    }

    Quat operator*(const Quat& q) const
    {
        return Quat(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w);
    }

    Quat conjugate() const { return Quat(w, -x, -y, -z); }

    Quat normalized() const
    {
        const double n = std::sqrt(w*w + x*x + y*y + z*z);
        return n > 1.0e-12 ? Quat(w / n, x / n, y / n, z / n) : Quat();
    }

    Vec3 rotate(const Vec3& v) const
    {
        const Vec3 qv(x, y, z);
        const Vec3 t = cross(qv, v) * 2.0;
        return v + t * w + cross(qv, t);
    }

    // Body axes given in world coordinates; they must be orthonormal with cross(x, y) = z
    static Quat fromAxes(const Vec3& x, const Vec3& y, const Vec3& z)
    {
        const double trace = x.x + y.y + z.z;
        if (trace > 0.0)
        {
            const double s = 2.0 * std::sqrt(trace + 1.0);
            return Quat(0.25 * s, (y.z - z.y) / s, (z.x - x.z) / s, (x.y - y.x) / s).normalized();
        }
        if (x.x > y.y && x.x > z.z)
        {
            const double s = 2.0 * std::sqrt(1.0 + x.x - y.y - z.z);
            return Quat((y.z - z.y) / s, 0.25 * s, (y.x + x.y) / s, (z.x + x.z) / s).normalized();
        }
        if (y.y > z.z)
        {
            const double s = 2.0 * std::sqrt(1.0 + y.y - x.x - z.z);
            return Quat((z.x - x.z) / s, (y.x + x.y) / s, 0.25 * s, (z.y + y.z) / s).normalized();
        }
        const double s = 2.0 * std::sqrt(1.0 + z.z - x.x - y.y);
        return Quat((x.y - y.x) / s, (z.x + x.z) / s, (z.y + y.z) / s, 0.25 * s).normalized();
    }

    static Quat fromAxisAngle(const Vec3& axis, double angle)
    {
        const Vec3 a = normalize(axis);
        const double s = std::sin(0.5 * angle);
        return Quat(std::cos(0.5 * angle), a.x*s, a.y*s, a.z*s);
    }
};

// Struct Mat4 (column-major, single precision, as WebGL expects)
struct Mat4
{
    float m[16] = { 1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f };

    Mat4 operator*(const Mat4& b) const
    {
        Mat4 c;
        for (int col = 0; col < 4; ++col)
        {
            for (int row = 0; row < 4; ++row)
            {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k)
                {
                    sum += m[k*4 + row] * b.m[col*4 + k];
                }
                c.m[col*4 + row] = sum;
            }
        }
        return c;
    }

    static Mat4 perspective(double fovY, double aspect, double zNear, double zFar)
    {
        const double f = 1.0 / std::tan(0.5 * fovY);
        Mat4 p;
        p.m[0] = (float) (f / aspect);
        p.m[5] = (float) f;
        p.m[10] = (float) ((zFar + zNear) / (zNear - zFar));
        p.m[11] = -1.0f;
        p.m[14] = (float) (2.0 * zFar * zNear / (zNear - zFar));
        p.m[15] = 0.0f;
        return p;
    }

    static Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up)
    {
        const Vec3 f = normalize(target - eye, Vec3(1.0, 0.0, 0.0));
        const Vec3 s = normalize(cross(f, up), Vec3(0.0, 0.0, 1.0));
        const Vec3 u = cross(s, f);
        Mat4 v;
        v.m[0] = (float) s.x;  v.m[4] = (float) s.y;  v.m[8]  = (float) s.z;  v.m[12] = (float) -dot(s, eye);
        v.m[1] = (float) u.x;  v.m[5] = (float) u.y;  v.m[9]  = (float) u.z;  v.m[13] = (float) -dot(u, eye);
        v.m[2] = (float) -f.x; v.m[6] = (float) -f.y; v.m[10] = (float) -f.z; v.m[14] = (float) dot(f, eye);
        return v;
    }
};
