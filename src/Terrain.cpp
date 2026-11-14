// Monster - the collision world: ground plane, road surfaces, ramps and a crushable wreck
// Author: Jan de Kort
// Created: 2026-11-14
#include "Terrain.h"

#include "Check.h"

#include <algorithm>
#include <cmath>

// Struct Face
Face::Face(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, Surface surface, bool solid)
    :
    m_a(a)
    , m_b(b)
    , m_c(c)
    , m_d(d)
    , m_normal(normalize(cross(b - a, d - a) + cross(d - c, b - c)))
    , m_surface(surface)
    , m_solid(solid)
{
}

// Struct Slab
Slab::Slab(double x0, double x1, double h0, double h1, double halfWidth)
    :
    m_x0(x0)
    , m_x1(x1)
    , m_h0(h0)
    , m_h1(h1)
    , m_halfWidth(halfWidth)
{
    require(x1 > x0, "Slab: x1 must exceed x0");
    require(halfWidth > 0.0, "Slab: halfWidth must be positive");
}

// Class Wreck
Wreck::Wreck(double x, double z)
    :
    m_x(x)
    , m_z(z)
    , m_halfWidth(0.9)
    , m_halfLength(2.3)
    , m_slopeRun(0.9)
    , m_height(1.25)
    , m_minFraction(0.3)
    , m_loadThreshold(4000.0)
    , m_crushResistance(9000.0)
    , m_crush(0.0)
{
}

double Wreck::getCurrentHeight() const
{
    return m_height * (1.0 - (1.0 - m_minFraction) * m_crush);
}

double Wreck::getReach() const
{
    return std::sqrt(std::pow(m_halfWidth + m_slopeRun, 2.0) + std::pow(m_halfLength, 2.0));
}

std::array<Face, 3> Wreck::faces() const
{
    const double h = getCurrentHeight();
    const double foot = m_halfWidth + m_slopeRun;
    const auto corner = [&](double dx, double y, double side)
    {
        return Vec3(m_x + dx, y, m_z + side * m_halfLength);
    };
    return { {
        Face(corner(-m_halfWidth, h, -1.0), corner(m_halfWidth, h, -1.0),
            corner(m_halfWidth, h, 1.0), corner(-m_halfWidth, h, 1.0), Face::Surface::Deck, true),
        Face(corner(-foot, 0.0, -1.0), corner(-m_halfWidth, h, -1.0),
            corner(-m_halfWidth, h, 1.0), corner(-foot, 0.0, 1.0), Face::Surface::Deck, true),
        Face(corner(m_halfWidth, h, -1.0), corner(foot, 0.0, -1.0),
            corner(foot, 0.0, 1.0), corner(m_halfWidth, h, 1.0), Face::Surface::Deck, true)
    } };
}

void Wreck::applyLoad(double load, double dt)
{
    require(dt > 0.0, "Wreck::applyLoad: dt must be positive");
    if (load > m_loadThreshold)
    {
        m_crush = std::min(1.0, m_crush + (load - m_loadThreshold) * dt / m_crushResistance);
    }
}

// Class Terrain
Terrain::Terrain()
    :
    m_track()
    , m_slabs()
    , m_wreck(35.0, 0.0)
    , m_faces()
    , m_triangles()
    , m_cellSize(8.0)
    , m_gridX(0.0)
    , m_gridZ(0.0)
    , m_gridColumns(0)
    , m_gridRows(0)
    , m_cells()
    , m_stripeWidth(0.5)
    , m_deckThickness(0.6)
{
    const double rampHalfWidth = 4.5;
    m_slabs.reserve(3);
    m_slabs.push_back(Slab( 90.0, 104.0, 0.0, 3.5, rampHalfWidth)); // take-off ramp
    m_slabs.push_back(Slab(115.0, 118.0, 0.0, 2.5, rampHalfWidth)); // front of the landing
    m_slabs.push_back(Slab(118.0, 140.0, 2.5, 0.0, rampHalfWidth)); // landing slope

    buildRoad();
    buildSlabs();
    buildGrid();
}

void Terrain::buildRoad()
{
    const std::vector<PathPoint>& points = m_track.getPoints();
    const double w = m_track.getHalfWidth();
    const Vec3 drop(0.0, -m_deckThickness, 0.0);
    const auto ground = [](const Vec3& p)
    {
        return Vec3(p.x, 0.0, p.z);
    };

    for (size_t i = 0; i + 1 < points.size(); ++i)
    {
        const PathPoint& p = points[i];
        const PathPoint& q = points[i + 1];
        const Vec3 pl = p.m_position - p.m_cross * w;
        const Vec3 pr = p.m_position + p.m_cross * w;
        const Vec3 ql = q.m_position - q.m_cross * w;
        const Vec3 qr = q.m_position + q.m_cross * w;
        const bool raised = p.m_kind != PathPoint::Kind::Flat || q.m_kind != PathPoint::Kind::Flat;
        const bool loop = p.m_kind == PathPoint::Kind::Loop || q.m_kind == PathPoint::Kind::Loop;
        const bool open = p.m_kind == PathPoint::Kind::Bridge && q.m_kind == PathPoint::Kind::Bridge;

        // The ground plane carries flat road; only raised road needs its own surface
        addFace(Face(pl, pr, qr, ql, Face::Surface::Road, raised));

        if (raised)
        {
            const Vec3 pin = p.m_cross * m_stripeWidth;
            const Vec3 qin = q.m_cross * m_stripeWidth;
            addFace(Face(pl, pl + pin, ql + qin, ql, Face::Surface::Stripe, false));
            addFace(Face(pr - pin, pr, qr, qr - qin, Face::Surface::Stripe, false));
        }
        if (raised && !loop)
        {
            if (open)
            {
                addFace(Face(pl + drop, pl, ql, ql + drop, Face::Surface::Concrete, false));
                addFace(Face(pr + drop, qr + drop, qr, pr, Face::Surface::Concrete, false));
                addFace(Face(pl + drop, ql + drop, qr + drop, pr + drop, Face::Surface::Concrete, false));
            }
            else
            {
                addFace(Face(ground(pl), pl, ql, ground(ql), Face::Surface::Concrete, true));
                addFace(Face(ground(pr), ground(qr), qr, pr, Face::Surface::Concrete, true));
            }
        }

        // Tunnel portals: close the embankment where the open span begins and ends
        const bool portal = (p.m_kind == PathPoint::Kind::Embankment && q.m_kind == PathPoint::Kind::Bridge)
            || (p.m_kind == PathPoint::Kind::Bridge && q.m_kind == PathPoint::Kind::Embankment);
        if (portal)
        {
            const bool entering = p.m_kind == PathPoint::Kind::Embankment;
            const PathPoint& edge = entering ? p : q;
            const Vec3 el = edge.m_position - edge.m_cross * (entering ? -w : w);
            const Vec3 er = edge.m_position + edge.m_cross * (entering ? -w : w);
            addFace(Face(ground(el), ground(er), er, el, Face::Surface::Concrete, true));
        }
    }
}

void Terrain::buildSlabs()
{
    for (const Slab& slab : m_slabs)
    {
        const double w = slab.m_halfWidth;
        const Vec3 a0(slab.m_x0, 0.0, -w);
        const Vec3 a1(slab.m_x1, 0.0, -w);
        const Vec3 b0(slab.m_x0, 0.0, w);
        const Vec3 b1(slab.m_x1, 0.0, w);
        const Vec3 rise0(0.0, slab.m_h0, 0.0);
        const Vec3 rise1(0.0, slab.m_h1, 0.0);

        addFace(Face(a0 + rise0, b0 + rise0, b1 + rise1, a1 + rise1, Face::Surface::Deck, true));
        addFace(Face(a0, a0 + rise0, a1 + rise1, a1, Face::Surface::Flank, true));
        addFace(Face(b0, b1, b1 + rise1, b0 + rise0, Face::Surface::Flank, true));
        if (slab.m_h0 > 0.0)
        {
            addFace(Face(a0, b0, b0 + rise0, a0 + rise0, Face::Surface::Flank, true));
        }
        if (slab.m_h1 > 0.0)
        {
            addFace(Face(a1, a1 + rise1, b1 + rise1, b1, Face::Surface::Flank, true));
        }
    }
}

Terrain::Triangle Terrain::makeTriangle(const Vec3& a, const Vec3& b, const Vec3& c)
{
    Triangle triangle;
    triangle.m_a = a;
    triangle.m_ab = b - a;
    triangle.m_ac = c - a;
    triangle.m_normal = normalize(cross(triangle.m_ab, triangle.m_ac));
    return triangle;
}

void Terrain::buildGrid()
{
    for (const Face& face : m_faces)
    {
        if (face.m_solid)
        {
            m_triangles.push_back(makeTriangle(face.m_a, face.m_b, face.m_c));
            m_triangles.push_back(makeTriangle(face.m_a, face.m_c, face.m_d));
        }
    }
    require(!m_triangles.empty(), "Terrain::buildGrid: no solid faces");

    const auto corners = [](const Triangle& t)
    {
        return std::array<Vec3, 3>{ { t.m_a, t.m_a + t.m_ab, t.m_a + t.m_ac } };
    };

    double maxX = m_triangles.front().m_a.x;
    double maxZ = m_triangles.front().m_a.z;
    m_gridX = maxX;
    m_gridZ = maxZ;
    for (const Triangle& triangle : m_triangles)
    {
        for (const Vec3& p : corners(triangle))
        {
            m_gridX = std::min(m_gridX, p.x);
            m_gridZ = std::min(m_gridZ, p.z);
            maxX = std::max(maxX, p.x);
            maxZ = std::max(maxZ, p.z);
        }
    }
    m_gridColumns = (int) std::floor((maxX - m_gridX) / m_cellSize) + 1;
    m_gridRows = (int) std::floor((maxZ - m_gridZ) / m_cellSize) + 1;
    m_cells.assign((size_t) m_gridColumns * m_gridRows, std::vector<int>());

    for (size_t i = 0; i < m_triangles.size(); ++i)
    {
        const std::array<Vec3, 3> p = corners(m_triangles[i]);
        const int c0 = (int) std::floor((std::min({ p[0].x, p[1].x, p[2].x }) - m_gridX) / m_cellSize);
        const int c1 = (int) std::floor((std::max({ p[0].x, p[1].x, p[2].x }) - m_gridX) / m_cellSize);
        const int r0 = (int) std::floor((std::min({ p[0].z, p[1].z, p[2].z }) - m_gridZ) / m_cellSize);
        const int r1 = (int) std::floor((std::max({ p[0].z, p[1].z, p[2].z }) - m_gridZ) / m_cellSize);
        for (int row = r0; row <= r1; ++row)
        {
            for (int column = c0; column <= c1; ++column)
            {
                m_cells.at((size_t) row * m_gridColumns + column).push_back((int) i);
            }
        }
    }
}

// Moeller-Trumbore, two-sided. Updates hit only when this triangle is nearer than reach.
bool Terrain::intersect(const Triangle& triangle, const Vec3& origin, const Vec3& direction, double reach, Hit& hit)
{
    const double tolerance = 1.0e-9;
    const Vec3 p = cross(direction, triangle.m_ac);
    const double det = dot(triangle.m_ab, p);
    if (std::fabs(det) < tolerance)
    {
        return false;
    }
    const Vec3 s = origin - triangle.m_a;
    const double u = dot(s, p) / det;
    if (u < 0.0 || u > 1.0)
    {
        return false;
    }
    const Vec3 q = cross(s, triangle.m_ab);
    const double v = dot(direction, q) / det;
    if (v < 0.0 || u + v > 1.0)
    {
        return false;
    }
    const double t = dot(triangle.m_ac, q) / det;
    if (t < 0.0 || t > reach)
    {
        return false;
    }
    hit.m_distance = t;
    hit.m_point = origin + direction * t;
    hit.m_normal = dot(triangle.m_normal, direction) < 0.0 ? triangle.m_normal : -triangle.m_normal;
    hit.m_wreck = false;
    return true;
}

bool Terrain::raycast(const Vec3& origin, const Vec3& direction, double reach, Hit& hit) const
{
    bool found = false;
    double nearest = reach;

    { // Ground plane
        if (direction.y < 0.0 && origin.y >= 0.0 && -origin.y / direction.y <= nearest)
        {
            nearest = -origin.y / direction.y;
            hit.m_distance = nearest;
            hit.m_point = origin + direction * nearest;
            hit.m_normal = Vec3(0.0, 1.0, 0.0);
            hit.m_wreck = false;
            found = true;
        }
    }

    const Vec3 end = origin + direction * nearest;

    { // Road surfaces, walls and ramps
        const int c0 = std::max(0, (int) std::floor((std::min(origin.x, end.x) - m_gridX) / m_cellSize));
        const int c1 = std::min(m_gridColumns - 1, (int) std::floor((std::max(origin.x, end.x) - m_gridX) / m_cellSize));
        const int r0 = std::max(0, (int) std::floor((std::min(origin.z, end.z) - m_gridZ) / m_cellSize));
        const int r1 = std::min(m_gridRows - 1, (int) std::floor((std::max(origin.z, end.z) - m_gridZ) / m_cellSize));
        for (int row = r0; row <= r1; ++row)
        {
            for (int column = c0; column <= c1; ++column)
            {
                for (int index : m_cells[(size_t) row * m_gridColumns + column])
                {
                    if (intersect(m_triangles[index], origin, direction, nearest, hit))
                    {
                        nearest = hit.m_distance;
                        found = true;
                    }
                }
            }
        }
    }

    { // Wreck: its shape changes, so its triangles are made on the spot
        const double dx = origin.x - m_wreck.getX();
        const double dz = origin.z - m_wreck.getZ();
        const double range = m_wreck.getReach() + reach;
        if (dx*dx + dz*dz < range*range)
        {
            for (const Face& face : m_wreck.faces())
            {
                if (intersect(makeTriangle(face.m_a, face.m_b, face.m_c), origin, direction, nearest, hit)
                    || intersect(makeTriangle(face.m_a, face.m_c, face.m_d), origin, direction, nearest, hit))
                {
                    nearest = hit.m_distance;
                    hit.m_wreck = true;
                    found = true;
                }
            }
        }
    }

    return found;
}
