// Monster - the collision world: ground plane, road surfaces, ramps and a crushable wreck
// Author: Jan de Kort
// Created: 2026-11-14
#include "Terrain.h"

#include "Check.h"

#include <algorithm>
#include <cmath>
#include <numeric>

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

double Face::traction(Surface surface)
{
    switch (surface)
    {
    case Surface::Road:
    case Surface::Stripe:
    case Surface::Deck:
    case Surface::Flank:
    case Surface::Concrete:
        return 1.00;
    case Surface::DirtPale:
    case Surface::DirtDark:
        return 0.86;
    default:
        require(false, "Face::traction: unknown surface");
        return 1.0;
    }
}

// Struct DirtRoad
bool DirtRoad::contains(const Vec3& point) const
{
    const Vec3 offset = point - m_centre;
    return std::fabs(dot(offset, m_cross)) < halfWidth() && std::fabs(dot(offset, m_forward)) < m_halfLength;
}

double DirtRoad::height(const Vec3& point) const
{
    if (!contains(point))
    {
        return 0.0;
    }
    const Vec3 offset = point - m_centre;
    const double across = (halfWidth() - std::fabs(dot(offset, m_cross))) / m_slopeRun;
    const double along = (m_halfLength - std::fabs(dot(offset, m_forward))) / m_endRun;
    return -m_depth * clamp(std::min(across, along), 0.0, 1.0);
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
Wreck::Wreck(const PathPoint& at)
    :
    m_position(at.m_position)
    , m_forward(at.m_forward)
    , m_up(at.m_normal)
    , m_cross(at.m_cross)
    , m_halfWidth(0.9)
    , m_halfLength(2.3)
    , m_slopeRun(1.1)
    , m_height(1.45)
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
    const auto corner = [&](double along, double up, double side)
    {
        return m_position + m_forward * along + m_up * up + m_cross * (side * m_halfLength);
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
    , m_dirt()
    , m_dirtOffset(30.5)
    , m_dirtTile(3.5)
    , m_slabs()
    , m_wrecks()
    , m_wreckCount(3)
    , m_wreckLeadIn(10.0)
    , m_wreckSpacing(6.5)
    , m_faces()
    , m_triangles()
    , m_cellSize(8.0)
    , m_gridX(0.0)
    , m_gridZ(0.0)
    , m_gridColumns(0)
    , m_gridRows(0)
    , m_cells()
    , m_grassTraction(0.70)
    , m_stripeWidth(0.5)
    , m_deckThickness(0.6)
{
    const double rampHalfWidth = 4.5;
    m_slabs.reserve(3);
    m_slabs.push_back(Slab( 71.0,  86.0, 0.0, 3.9, rampHalfWidth)); // take-off ramp
    m_slabs.push_back(Slab( 99.0, 102.0, 0.0, 2.5, rampHalfWidth)); // front of the landing
    m_slabs.push_back(Slab(102.0, 124.0, 2.5, 0.0, rampHalfWidth)); // landing slope

    placeDirtRoad();
    placeWrecks();
    buildRoad();
    buildDirtRoad();
    buildSlabs();
    buildGrid();
}

// The dirt road runs alongside the bridge, to its right as the bridge is driven, and the road
// out of the tunnel jumps over it between the two ramps
void Terrain::placeDirtRoad()
{
    const PathPoint& gap = m_track.getGapCentre();
    m_dirt.m_forward = gap.m_forward;
    m_dirt.m_cross = gap.m_cross;
    m_dirt.m_centre = Vec3(gap.m_position.x, 0.0, gap.m_position.z) + gap.m_cross * m_dirtOffset;

    { // The ramps have to stand either side of it, with none of it left between them
        const double before = m_slabs.front().m_x1;
        const double after = m_slabs[1].m_x0;
        require(!m_dirt.contains(Vec3(before - 0.1, 0.0, 0.0)) && !m_dirt.contains(Vec3(after + 0.1, 0.0, 0.0)),
            "Terrain::placeDirtRoad: a ramp stands on the dirt road");
        require(m_dirt.contains(Vec3(0.5 * (before + after), 0.0, 0.0)),
            "Terrain::placeDirtRoad: the jump does not clear the dirt road");
    }
}

// Tiled in two shades, in squares smaller than the grass, so it reads as a dirt track
void Terrain::buildDirtRoad()
{
    const int across = (int) std::ceil(2.0 * m_dirt.halfWidth() / m_dirtTile);
    const int along = (int) std::ceil(2.0 * m_dirt.m_halfLength / m_dirtTile);
    const double stepAcross = 2.0 * m_dirt.halfWidth() / across;
    const double stepAlong = 2.0 * m_dirt.m_halfLength / along;
    const auto corner = [&](int i, int j)
    {
        const Vec3 flat = m_dirt.m_centre
            + m_dirt.m_cross * (-m_dirt.halfWidth() + stepAcross * i)
            + m_dirt.m_forward * (-m_dirt.m_halfLength + stepAlong * j);
        return flat + Vec3(0.0, m_dirt.height(flat), 0.0);
    };

    for (int i = 0; i < across; ++i)
    {
        for (int j = 0; j < along; ++j)
        {
            addFace(Face(corner(i, j), corner(i, j + 1), corner(i + 1, j + 1), corner(i + 1, j),
                ((i + j) & 1) != 0 ? Face::Surface::DirtPale : Face::Surface::DirtDark, true));
        }
    }
}

// A row of old cars to flatten, lying across the road on the run back from the last gate
void Terrain::placeWrecks()
{
    const std::vector<PathPoint>& points = m_track.getPoints();
    const double first = points[m_track.getCheckpoints().back()].m_distance + m_wreckLeadIn;

    m_wrecks.reserve(m_wreckCount);
    for (size_t i = 0; i < m_wreckCount; ++i)
    {
        const double target = first + m_wreckSpacing * i;
        const auto found = std::min_element(points.begin(), points.end(),
            [&](const PathPoint& a, const PathPoint& b)
            {
                return std::fabs(a.m_distance - target) < std::fabs(b.m_distance - target);
            });
        require(found->m_kind == PathPoint::Kind::Flat && !found->m_corner,
            "Terrain::placeWrecks: a wreck does not lie on level, straight road");
        m_wrecks.push_back(Wreck(*found));
    }
}

void Terrain::reset()
{
    for (Wreck& wreck : m_wrecks)
    {
        wreck.reset();
    }
}

// The solid ground under a point: the dirt road where it is sunk, the plane at zero elsewhere
double Terrain::groundFloor(const Vec3& point) const
{
    return m_dirt.height(point);
}

double Terrain::getCrush() const
{
    const double total = std::accumulate(m_wrecks.begin(), m_wrecks.end(), 0.0,
        [](double sum, const Wreck& wreck) { return sum + wreck.getCrush(); });
    return total / m_wrecks.size();
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
        if (p.m_kind == PathPoint::Kind::Gap || q.m_kind == PathPoint::Kind::Gap)
        {
            // The missing piece of bridge: nothing to drive on, only the cut ends of the deck
            const PathPoint& edge = p.m_kind == PathPoint::Kind::Gap ? q : p;
            if (edge.m_kind != PathPoint::Kind::Gap)
            {
                const Vec3 el = edge.m_position - edge.m_cross * w;
                const Vec3 er = edge.m_position + edge.m_cross * w;
                addFace(Face(el + drop, er + drop, er, el, Face::Surface::Concrete, false));
            }
            continue;
        }
        if (p.m_kind == PathPoint::Kind::Flat && m_dirt.contains(p.m_position))
        {
            continue; // the road is cut where it crosses the dirt road
        }

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

Terrain::Triangle Terrain::makeTriangle(const Vec3& a, const Vec3& b, const Vec3& c, double traction)
{
    Triangle triangle;
    triangle.m_traction = traction;
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
            const double grip = Face::traction(face.m_surface);
            m_triangles.push_back(makeTriangle(face.m_a, face.m_b, face.m_c, grip));
            m_triangles.push_back(makeTriangle(face.m_a, face.m_c, face.m_d, grip));
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
    hit.m_traction = triangle.m_traction;
    hit.m_wreck = -1;
    return true;
}

bool Terrain::raycast(const Vec3& origin, const Vec3& direction, double reach, Hit& hit) const
{
    bool found = false;
    double nearest = reach;

    { // Ground plane, with a hole in it where the dirt road is sunk
        const double t = direction.y < 0.0 ? -origin.y / direction.y : -1.0;
        if (t >= 0.0 && origin.y >= 0.0 && t <= nearest && !m_dirt.contains(origin + direction * t))
        {
            nearest = t;
            hit.m_distance = t;
            hit.m_point = origin + direction * t;
            hit.m_normal = Vec3(0.0, 1.0, 0.0);
            hit.m_traction = m_grassTraction;
            hit.m_wreck = -1;
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

    { // Wrecks: their shape changes, so their triangles are made on the spot
        for (size_t i = 0; i < m_wrecks.size(); ++i)
        {
            const Wreck& wreck = m_wrecks[i];
            if (length(origin - wreck.getPosition()) > wreck.getReach() + reach)
            {
                continue;
            }
            for (const Face& face : wreck.faces())
            {
                const double grip = Face::traction(face.m_surface);
                if (intersect(makeTriangle(face.m_a, face.m_b, face.m_c, grip), origin, direction, nearest, hit)
                    || intersect(makeTriangle(face.m_a, face.m_c, face.m_d, grip), origin, direction, nearest, hit))
                {
                    nearest = hit.m_distance;
                    hit.m_wreck = (int) i;
                    found = true;
                }
            }
        }
    }

    return found;
}
