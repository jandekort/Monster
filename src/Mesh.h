// Monster - flat shaded triangle soup, ready to upload as a vertex buffer
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Math.h"

#include <cstddef>
#include <vector>

// Struct Color
struct Color
{
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;

    Color() = default;
    Color(float r, float g, float b)
        :
        r(r)
        , g(g)
        , b(b)
    {
    }

    Color scaled(float s) const { return Color(r*s, g*s, b*s); }
};

// Class Mesh
// Vertex layout: x, y, z, r, g, b (six floats). Lighting is baked into the colour.
class Mesh
{
public:
    Mesh();

    // Mutator
    void clear() { m_vertices.clear(); }
    void addTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Color& color, const Vec3& normal);
    void addQuad(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Color& color, const Vec3& normal);
    void addBox(const Vec3& center, const Vec3& axisX, const Vec3& axisY, const Vec3& axisZ,
        const Vec3& halfSize, const Color& color);
    void addPyramid(const Vec3& baseCenter, double halfBase, double height, const Color& color);

    // Accessor
    const float* data() const { return m_vertices.data(); }
    size_t vertexCount() const { return m_vertices.size() / m_floatsPerVertex; }

private:
    void addVertex(const Vec3& p, const Color& color);
    Color shade(const Color& color, const Vec3& normal) const;

    std::vector<float> m_vertices;
    size_t m_floatsPerVertex;
    Vec3 m_light;     // direction towards the sun
    double m_ambient; // light on faces turned away from the sun
};
