// Monster - flat shaded triangle soup, ready to upload as a vertex buffer
// Author: Jan de Kort
// Created: 2026-11-14
#include "Mesh.h"

#include "Check.h"

#include <algorithm>
#include <numeric>

Mesh::Mesh()
    :
    m_vertices()
    , m_floatsPerVertex(6)
    , m_light(normalize(Vec3(-0.35, 0.8, 0.5)))
    , m_ambient(0.55)
{
}

void Mesh::addVertex(const Vec3& p, const Color& color)
{
    m_vertices.push_back((float) p.x);
    m_vertices.push_back((float) p.y);
    m_vertices.push_back((float) p.z);
    m_vertices.push_back(color.r);
    m_vertices.push_back(color.g);
    m_vertices.push_back(color.b);
}

Color Mesh::shade(const Color& color, const Vec3& normal) const
{
    const double diffuse = std::max(0.0, dot(normal, m_light));
    return color.scaled((float) (m_ambient + (1.0 - m_ambient) * diffuse));
}

void Mesh::addTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Color& color, const Vec3& normal)
{
    const Color lit = shade(color, normal);
    addVertex(a, lit);
    addVertex(b, lit);
    addVertex(c, lit);
}

void Mesh::addQuad(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Color& color, const Vec3& normal)
{
    const Color lit = shade(color, normal);
    addVertex(a, lit);
    addVertex(b, lit);
    addVertex(c, lit);
    addVertex(a, lit);
    addVertex(c, lit);
    addVertex(d, lit);
}

void Mesh::addBox(const Vec3& center, const Vec3& axisX, const Vec3& axisY, const Vec3& axisZ,
    const Vec3& halfSize, const Color& color)
{
    const Vec3 x = axisX * halfSize.x;
    const Vec3 y = axisY * halfSize.y;
    const Vec3 z = axisZ * halfSize.z;
    addQuad(center + x - y - z, center + x + y - z, center + x + y + z, center + x - y + z, color, axisX);
    addQuad(center - x - y - z, center - x + y - z, center - x + y + z, center - x - y + z, color, -axisX);
    addQuad(center + y - x - z, center + y + x - z, center + y + x + z, center + y - x + z, color, axisY);
    addQuad(center - y - x - z, center - y + x - z, center - y + x + z, center - y - x + z, color, -axisY);
    addQuad(center + z - x - y, center + z + x - y, center + z + x + y, center + z - x + y, color, axisZ);
    addQuad(center - z - x - y, center - z + x - y, center - z + x + y, center - z - x + y, color, -axisZ);
}

void Mesh::addPyramid(const Vec3& baseCenter, double halfBase, double height, const Color& color)
{
    const Vec3 apex = baseCenter + Vec3(0.0, height, 0.0);
    const Vec3 a = baseCenter + Vec3(-halfBase, 0.0, -halfBase);
    const Vec3 b = baseCenter + Vec3( halfBase, 0.0, -halfBase);
    const Vec3 c = baseCenter + Vec3( halfBase, 0.0,  halfBase);
    const Vec3 d = baseCenter + Vec3(-halfBase, 0.0,  halfBase);
    addTriangle(a, b, apex, color, normalize(Vec3( 0.0, halfBase, -height)));
    addTriangle(b, c, apex, color, normalize(Vec3( height, halfBase, 0.0)));
    addTriangle(c, d, apex, color, normalize(Vec3( 0.0, halfBase,  height)));
    addTriangle(d, a, apex, color, normalize(Vec3(-height, halfBase, 0.0)));
}

void Mesh::addPrism(const Vec3& origin, const Vec3& axisX, const Vec3& axisY, const Vec3& axisZ,
    const std::vector<Vec3>& outline, double halfWidth, double bevel, const Color& color)
{
    const size_t count = outline.size();
    require(count >= 3, "Mesh::addPrism: an outline needs three corners");
    require(bevel >= 0.0 && bevel < halfWidth, "Mesh::addPrism: bevel must be smaller than halfWidth");

    const Vec3 middle = std::accumulate(outline.begin(), outline.end(), Vec3()) * (1.0 / count);

    // Outward normal of each edge, in the plane of the outline
    std::vector<Vec3> normals;
    normals.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        const Vec3& p = outline[i];
        const Vec3& q = outline[(i + 1) % count];
        const Vec3 normal = normalize(Vec3(q.y - p.y, p.x - q.x, 0.0));
        normals.push_back(dot(normal, (p + q) * 0.5 - middle) < 0.0 ? -normal : normal);
    }

    // The outline of the end faces: every corner moved in by bevel from both edges it lies on
    std::vector<Vec3> inner;
    inner.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        const Vec3& before = normals[(i + count - 1) % count];
        const Vec3& after = normals[i];
        inner.push_back(outline[i] - (before + after) * (bevel / (1.0 + dot(before, after))));
    }

    const auto world = [&](const Vec3& corner, double z)
    {
        return origin + axisX * corner.x + axisY * corner.y + axisZ * z;
    };
    const double flat = halfWidth - bevel;
    for (size_t i = 0; i < count; ++i)
    {
        const size_t j = (i + 1) % count;
        const Vec3 normal = axisX * normals[i].x + axisY * normals[i].y;
        addQuad(world(outline[i], -flat), world(outline[j], -flat), world(outline[j], flat), world(outline[i], flat),
            color, normal);
        for (double side : { -1.0, 1.0 })
        {
            addQuad(world(outline[i], side * flat), world(outline[j], side * flat),
                world(inner[j], side * halfWidth), world(inner[i], side * halfWidth),
                color, normalize(normal + axisZ * side));
        }
    }
    for (double side : { -1.0, 1.0 })
    {
        for (size_t i = 1; i + 1 < count; ++i)
        {
            addTriangle(world(inner[0], side * halfWidth), world(inner[i], side * halfWidth),
                world(inner[i + 1], side * halfWidth), color, axisZ * side);
        }
    }
}

void Mesh::addCylinder(const Vec3& from, const Vec3& to, double radius, const Color& side, const Color& end)
{
    const int sides = 8;
    const Vec3 axis = normalize(to - from);
    const Vec3 upward(0.0, 1.0, 0.0);
    const Vec3 level(1.0, 0.0, 0.0);
    const Vec3 lift = upward - axis * dot(upward, axis);
    const Vec3 high = length(lift) > 0.2 ? normalize(lift) : normalize(level - axis * dot(level, axis));
    const Vec3 flank = cross(high, axis);
    const auto spoke = [&](double turn)
    {
        const double angle = 2.0 * M_PI * turn / sides;
        return high * std::cos(angle) + flank * std::sin(angle);
    };

    for (int k = 0; k < sides; ++k)
    {
        const Vec3 p = spoke(k + 0.5) * radius;
        const Vec3 q = spoke(k + 1.5) * radius;
        addQuad(from + p, to + p, to + q, from + q, side, spoke(k + 1.0));
        addTriangle(from, from + p, from + q, end, -axis);
        addTriangle(to, to + p, to + q, end, axis);
    }
}
