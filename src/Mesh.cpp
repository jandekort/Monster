// Monster - flat shaded triangle soup, ready to upload as a vertex buffer
// Author: Jan de Kort
// Created: 2026-11-14
#include "Mesh.h"

#include <algorithm>

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
