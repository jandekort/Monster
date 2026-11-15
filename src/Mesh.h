// Monster - flat shaded triangle soup, ready to upload as a vertex buffer
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Color.h"
#include "Math.h"

#include <cstddef>
#include <vector>

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

    // A convex outline in the plane of axisX and axisY (z is ignored), drawn out along axisZ to
    // halfWidth either side. Where the outline meets the two end faces the edge is cut back by
    // bevel, which takes the squareness off.
    // A round bar of eight flat sides from one end to the other, its ends in their own colour
    void addCylinder(const Vec3& from, const Vec3& to, double radius, const Color& side, const Color& end);

    void addPrism(const Vec3& origin, const Vec3& axisX, const Vec3& axisY, const Vec3& axisZ,
        const std::vector<Vec3>& outline, double halfWidth, double bevel, const Color& color);

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
