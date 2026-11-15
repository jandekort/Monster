// Monster - the collision world: ground plane, road surfaces, ramps and a crushable wreck
// Author: Jan de Kort
// Created: 2026-11-14
#include "Terrain.h"

#include "Check.h"

#include <algorithm>
#include <cmath>
#include <iterator>
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
    case Surface::LogPale:
    case Surface::LogDark:
    case Surface::LogEnd:
    case Surface::Sunken:
    case Surface::Timber:
        return 0.86;
    case Surface::GravelPale:
    case Surface::GravelDark:
        return 0.70; // loose stones: better than grass, worse than the beaten track
    case Surface::Bank:
        return 0.55; // grass, the same going as the rest of the verge
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
    , m_riverbed()
    , m_ford()
    , m_dirtOffset(30.5)
    , m_dirtColumns(3)
    , m_slipGap(0.0)
    , m_slipSplay(0.45)
    , m_groundSlack(1.0e-4)
    , m_bypassAppeal(0.75)
    , m_shortcutAppeal(1.0 / 12.0)
    , m_shortcutLeadIn(10.0)
    , m_shortcutRunIn(34.0)
    , m_shortcutSweep(40.0)
    , m_riverTiles(13)
    , m_riverShift(5.5)
    , m_bedColumns(5)
    , m_waterDepth(0.9)
    , m_wadeSpeed(15.0 / 3.6)
    , m_shoreRise(0.45)
    , m_bridgeEdge(3.0)
    , m_logLength(7.0)
    , m_logRadius(0.25)
    , m_deckRise(0.75)
    , m_dipRun(4.0)
    , m_dipDepth(0.5)
    , m_laneWidth(4.5)
    , m_cautionRun(22.0)
    , m_cautionSpeed(7.0)
    , m_barrierLeadIn(6.0)
    , m_barrierClear(18.0)
    , m_barrierReach(2.5)
    , m_detours()
    , m_barrier()
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
    , m_grassTraction(0.55)
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
    placeRiverbed();
    buildRoad();
    buildBypass();
    buildShortcut();
    placeBarrier();
    buildSlabs();
    buildGrid();
}

// The dirt road runs alongside the bridge, to its right as the bridge is driven, and the road
// out of the tunnel jumps over it between the two ramps
void Terrain::placeDirtRoad()
{
    const PathPoint& gap = m_track.getBridgeCrest();
    m_dirt.m_forward = Vec3(gap.m_forward.x, 0.0, gap.m_forward.z);
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

size_t Terrain::pointAt(double distance) const
{
    const std::vector<PathPoint>& points = m_track.getPoints();
    const auto found = std::min_element(points.begin(), points.end(),
        [&](const PathPoint& a, const PathPoint& b)
        {
            return std::fabs(a.m_distance - distance) < std::fabs(b.m_distance - distance);
        });
    return (size_t) (found - points.begin());
}

// The shortcut leaves the road a little beyond the last wreck
size_t Terrain::shortcutLeave() const
{
    const std::vector<PathPoint>& points = m_track.getPoints();
    const double lastWreck = points[m_track.getCheckpoints().back()].m_distance + m_wreckLeadIn
        + m_wreckSpacing * (m_wreckCount - 1);
    return pointAt(lastWreck + m_shortcutLeadIn);
}

// and comes back to it just before the start and finish line, behind the starting grid
size_t Terrain::shortcutRejoin() const
{
    return pointAt(m_track.getLength() - m_shortcutRunIn);
}

// The dry riverbed lies inside the last two bends, square to the road the shortcut leaves. The
// shortcut swings out, fords it from bank to bank and swings back to the road coming the other way.
void Terrain::placeRiverbed()
{
    const std::vector<PathPoint>& points = m_track.getPoints();
    const PathPoint& leave = points[shortcutLeave()];
    const PathPoint& rejoin = points[shortcutRejoin()];
    const auto plain = [](const PathPoint& point)
    {
        return point.m_kind == PathPoint::Kind::Flat && !point.m_corner;
    };
    require(plain(leave) && plain(rejoin), "Terrain::placeRiverbed: the shortcut needs level, straight road at both ends");
    require(dot(leave.m_forward, rejoin.m_forward) < -0.99 && dot(rejoin.m_position - leave.m_position, leave.m_cross) < 0.0,
        "Terrain::placeRiverbed: the road does not come back on the left of where the shortcut leaves it");

    m_ford = (leave.m_position + rejoin.m_position) * 0.5 + leave.m_forward * m_shortcutSweep;
    m_ford.y = 0.0;
    m_riverbed.m_forward = -leave.m_forward; // the river runs alongside the road the shortcut leaves
    m_riverbed.m_cross = -leave.m_cross;     // and the shortcut crosses it from one bank to the other
    m_riverbed.m_halfFloor = 9.0;
    m_riverbed.m_slopeRun = 9.0;
    m_riverbed.m_depth = 3.0; // the water lies well below the road: there is a climb out of it
    m_riverbed.m_endRun = 12.0;
    m_riverbed.m_halfLength = m_riverTiles * tileSize();
    // The riverbed reaches further upstream of the ford than downstream, away from the junctions.
    // The shift is a whole number of tiles and a half, so the ford is three whole tiles wide.
    m_riverbed.m_centre = m_ford - m_riverbed.m_forward * (m_riverShift * tileSize());

    const double fordToEnd = m_riverbed.m_halfLength - m_riverShift * tileSize() - m_dirt.m_halfFloor;
    require(fordToEnd >= m_riverbed.m_endRun, "Terrain::placeRiverbed: the ford lies where the riverbed runs out");
}

// The barrier stands on the level road before the bridge, beyond the mouth of the dirt road
void Terrain::placeBarrier()
{
    const std::vector<PathPoint>& points = m_track.getPoints();
    const size_t leave = getBypass().m_leave;
    size_t climb = leave;
    while (climb + 1 < points.size() && points[climb].m_kind == PathPoint::Kind::Flat)
    {
        ++climb;
    }
    const PathPoint& at = points[pointAt(points[climb].m_distance - m_barrierLeadIn)];
    require(at.m_kind == PathPoint::Kind::Flat && at.m_distance - points[leave].m_distance >= m_barrierClear,
        "Terrain::placeBarrier: no level road between the mouth of the dirt road and the bridge");

    m_barrier.m_position = at.m_position;
    m_barrier.m_forward = at.m_forward;
    m_barrier.m_cross = at.m_cross;
    m_barrier.m_halfWidth = m_track.getHalfWidth();
    m_barrier.m_broken = false;
}

void Terrain::strike(const Vec3& position)
{
    const Vec3 offset = position - m_barrier.m_position;
    const double truckHalfWidth = 2.0;
    if (std::fabs(dot(offset, m_barrier.m_forward)) < m_barrierReach
        && std::fabs(dot(offset, m_barrier.m_cross)) < m_barrier.m_halfWidth + truckHalfWidth
        && std::fabs(offset.y) < 2.0 * m_barrierReach)
    {
        m_barrier.m_broken = true;
    }
}

// The cutting: a floor of dirt tiles in two shades, smaller than the grass tiles, between two
// grass banks. Returns its centreline.
std::vector<Vec3> Terrain::buildDirtRoad()
{
    const double tile = tileSize();
    const int rows = (int) std::ceil(2.0 * m_dirt.m_halfLength / tile);
    const double step = 2.0 * m_dirt.m_halfLength / rows;
    const auto corner = [&](double across, int row)
    {
        const Vec3 flat = m_dirt.m_centre + m_dirt.m_cross * across
            + m_dirt.m_forward * (-m_dirt.m_halfLength + step * row);
        return flat + Vec3(0.0, m_dirt.height(flat), 0.0);
    };
    const auto strip = [&](double from, double to, int row, Face::Surface surface)
    {
        addFace(Face(corner(from, row), corner(to, row), corner(to, row + 1), corner(from, row + 1), surface, true));
    };

    std::vector<Vec3> centreline;
    centreline.reserve(rows);
    for (int row = 0; row < rows; ++row)
    {
        for (int column = 0; column < m_dirtColumns; ++column)
        {
            const double from = -m_dirt.m_halfFloor + tile * column;
            strip(from, from + tile, row,
                ((column + row) & 1) != 0 ? Face::Surface::DirtPale : Face::Surface::DirtDark);
        }
        strip(-m_dirt.halfWidth(), -m_dirt.m_halfFloor, row, Face::Surface::Bank);
        strip(m_dirt.m_halfFloor, m_dirt.halfWidth(), row, Face::Surface::Bank);
        centreline.push_back(corner(0.0, row));
    }
    return centreline;
}

// Slip roads make the dirt road a way round the broken bridge. One leaves the right hand edge of
// the main road before the bridge starts to climb, swings out and runs straight into the end of
// the cutting; the other comes out of the far end and swings back to the edge of the main road
// beyond the bridge. They run alongside the main road where they meet it, never across it.
void Terrain::buildBypass()
{
    const std::vector<PathPoint>& points = m_track.getPoints();
    const size_t centre = (size_t) (&m_track.getBridgeCrest() - &points.front());
    const auto plain = [&](size_t index)
    {
        return points[index].m_kind == PathPoint::Kind::Flat && !points[index].m_corner;
    };

    // The straight, level road either side of the bridge
    size_t leave = centre;
    while (leave > 0 && !plain(leave))
    {
        --leave;
    }
    while (leave > 1 && plain(leave - 2))
    {
        --leave;
    }
    size_t rejoin = centre;
    while (rejoin + 1 < points.size() && !plain(rejoin))
    {
        ++rejoin;
    }
    while (rejoin + 3 < points.size() && plain(rejoin + 2))
    {
        ++rejoin;
    }
    require(leave < centre && rejoin > centre, "Terrain::buildBypass: no level road beside the bridge");

    Detour detour;
    detour.m_leave = leave;
    detour.m_rejoin = rejoin;
    detour.m_appeal = m_bypassAppeal;
    detour.m_cautious = true;

    // A slip road starts underneath the main road and splays out of its edge, like the arm of a junction
    const double aside = m_track.getHalfWidth() - m_dirt.m_halfFloor + m_slipGap;
    const Vec3 turnOff = normalize(points[leave].m_forward + points[leave].m_cross * m_slipSplay, points[leave].m_forward);
    const Vec3 turnIn = normalize(points[rejoin].m_forward - points[rejoin].m_cross * m_slipSplay, points[rejoin].m_forward);
    const std::vector<Vec3> cutting = buildDirtRoad();
    const int rows = (int) cutting.size();

    buildSlipRoad(detour.m_path, points[leave].m_position + points[leave].m_cross * aside, turnOff,
        m_dirt.m_centre - m_dirt.m_forward * m_dirt.m_halfLength, m_dirt.m_forward, 0, points[leave], 1.0);
    detour.m_path.insert(detour.m_path.end(), cutting.begin(), cutting.end());
    buildSlipRoad(detour.m_path, m_dirt.m_centre + m_dirt.m_forward * m_dirt.m_halfLength, m_dirt.m_forward,
        points[rejoin].m_position + points[rejoin].m_cross * aside, turnIn, rows, points[rejoin], 1.0);
    m_detours.push_back(detour);
}

// The shortcut cuts inside the last two bends. It leaves the left hand edge of the road beyond
// the wrecks, goes down to the river, out over what is left of a log bridge, through the water
// where the bridge has gone, up the far bank and back to the road just before the start and
// finish line.
void Terrain::buildShortcut()
{
    const std::vector<PathPoint>& points = m_track.getPoints();
    const PathPoint& leave = points[shortcutLeave()];
    const PathPoint& rejoin = points[shortcutRejoin()];

    Detour detour;
    detour.m_leave = shortcutLeave();
    detour.m_rejoin = shortcutRejoin();
    detour.m_appeal = m_shortcutAppeal;
    detour.m_cautious = false;

    const double aside = m_track.getHalfWidth() - m_dirt.m_halfFloor + m_slipGap;
    const Vec3 turnOff = normalize(leave.m_forward - leave.m_cross * m_slipSplay, leave.m_forward);
    const Vec3 turnIn = normalize(rejoin.m_forward + rejoin.m_cross * m_slipSplay, rejoin.m_forward);
    const Vec3 nearBank = m_ford - m_riverbed.m_cross * m_riverbed.halfWidth();
    const Vec3 farBank = m_ford + m_riverbed.m_cross * m_riverbed.halfWidth();

    buildSlipRoad(detour.m_path, leave.m_position - leave.m_cross * aside, turnOff,
        nearBank, m_riverbed.m_cross, 0, leave, -1.0);
    const std::vector<Vec3> ford = buildRiverbed();
    detour.m_path.insert(detour.m_path.end(), ford.begin(), ford.end());
    buildSlipRoad(detour.m_path, farBank, m_riverbed.m_cross,
        rejoin.m_position - rejoin.m_cross * aside, turnIn, 1, rejoin, -1.0);
    buildLogBridge();

    // The bridge is no place to arrive at speed: from a little before the river to the middle of
    // it, where the bridge ends, the route sets a speed
    detour.m_speeds.reserve(detour.m_path.size());
    std::transform(detour.m_path.begin(), detour.m_path.end(), std::back_inserter(detour.m_speeds),
        [&](const Vec3& point)
        {
            const Vec3 offset = point - m_ford;
            const bool care = dot(offset, m_riverbed.m_cross) < 0.0
                && std::hypot(offset.x, offset.z) < m_riverbed.halfWidth() + m_cautionRun;
            return care ? m_cautionSpeed : 0.0;
        });
    m_detours.push_back(detour);
}

// The riverbed: a floor of gravel between two banks, grass at the top and sand towards the water,
// with a band of dirt across it, banks and all, where the shortcut crosses. Returns the
// centreline of the crossing, which keeps to the edge of the bridge that is still dry.
std::vector<Vec3> Terrain::buildRiverbed()
{
    const DirtRoad& bed = m_riverbed;
    const double tile = tileSize();
    const int rows = 2 * m_riverTiles;
    const double fordAt = dot(m_ford - bed.m_centre, bed.m_forward);
    const auto corner = [&](double across, int row)
    {
        const Vec3 flat = bed.m_centre + bed.m_cross * across + bed.m_forward * (-bed.m_halfLength + tile * row);
        return flat + Vec3(0.0, bed.height(flat), 0.0);
    };

    // Across the riverbed: on each bank a strip of grass and a strip of sand, the floor in m_bedColumns
    const double shore = bed.halfWidth() + (getWaterLevel() + m_shoreRise) * bed.m_slopeRun / bed.m_depth;
    require(shore > bed.m_halfFloor && shore < bed.halfWidth(), "Terrain::buildRiverbed: the shore does not lie on the banks");
    std::vector<double> stations;
    stations.push_back(-bed.halfWidth());
    stations.push_back(-shore);
    for (int column = 0; column <= m_bedColumns; ++column)
    {
        stations.push_back(-bed.m_halfFloor + 2.0 * bed.m_halfFloor * column / m_bedColumns);
    }
    stations.push_back(shore);
    stations.push_back(bed.halfWidth());

    for (int row = 0; row < rows; ++row)
    {
        const double middle = -bed.m_halfLength + tile * (row + 0.5);
        const bool track = std::fabs(middle - fordAt) < m_dirt.m_halfFloor;
        for (size_t k = 0; k + 1 < stations.size(); ++k)
        {
            const bool grass = k == 0 || k + 2 == stations.size();
            const bool pale = ((row + k) & 1) != 0;
            const Face::Surface surface = track ? (pale ? Face::Surface::DirtPale : Face::Surface::DirtDark)
                : grass ? Face::Surface::Bank
                : (pale ? Face::Surface::GravelPale : Face::Surface::GravelDark);
            addFace(Face(corner(stations[k], row), corner(stations[k + 1], row),
                corner(stations[k + 1], row + 1), corner(stations[k], row + 1), surface, true));
        }
    }

    // The way across: over to the dry edge of the bridge on the way down the bank, and back to
    // the middle of the track by the far bank
    const Vec3 right = -bed.m_forward;
    const double lane = m_bridgeEdge - 0.5 * m_laneWidth;
    std::vector<Vec3> centreline;
    for (double across = -bed.halfWidth(); across < bed.halfWidth(); across += tile)
    {
        const double aside = lane * clamp(std::min((across + bed.halfWidth()) / 4.0, (bed.m_halfFloor - across) / 9.0), 0.0, 1.0);
        const Vec3 flat = m_ford + bed.m_cross * across + right * aside;
        centreline.push_back(flat + Vec3(0.0, bed.height(flat), 0.0));
    }
    return centreline;
}

// A round log of eight flat sides from one end to the other, its ends sawn off square. With
// solidTop set, the three sides that face upwards can be driven on.
void Terrain::addLog(const Vec3& from, const Vec3& to, double radius, Face::Surface bark, bool solidTop)
{
    const int sides = 8;
    const Vec3 axis = normalize(to - from);
    const Vec3 upward(0.0, 1.0, 0.0);
    const Vec3 level(1.0, 0.0, 0.0);
    const Vec3 lift = upward - axis * dot(upward, axis);
    const Vec3 high = length(lift) > 0.2 ? normalize(lift) : normalize(level - axis * dot(level, axis));
    const Vec3 flank = cross(high, axis);
    const auto rim = [&](int k)
    {
        const double angle = 2.0 * M_PI * (k + 0.5) / sides;
        return (high * std::cos(angle) + flank * std::sin(angle)) * radius;
    };

    for (int k = 0; k < sides; ++k)
    {
        const bool onTop = k == sides - 1 || k == 0 || k == sides - 2;
        addFace(Face(from + rim(k), to + rim(k), to + rim(k + 1), from + rim(k + 1), bark, solidTop && onTop));
    }
    addFace(Face(from + rim(0), from + rim(1), from + rim(2), from + rim(3), Face::Surface::LogEnd, false));
    addFace(Face(from + rim(0), from + rim(3), from + rim(4), from + rim(7), Face::Surface::LogEnd, false));
    addFace(Face(from + rim(4), from + rim(5), from + rim(6), from + rim(7), Face::Surface::LogEnd, false));
    addFace(Face(to + rim(3), to + rim(2), to + rim(1), to + rim(0), Face::Surface::LogEnd, false));
    addFace(Face(to + rim(7), to + rim(4), to + rim(3), to + rim(0), Face::Surface::LogEnd, false));
    addFace(Face(to + rim(7), to + rim(6), to + rim(5), to + rim(4), Face::Surface::LogEnd, false));
}

// What is left of a log bridge: logs laid side by side across the way a truck drives, from the
// near bank to the middle of the river. Its left hand side has collapsed and lies on the river
// floor, so the deck is tipped over sideways and only a lane along its right hand edge is above
// the water, little wider than a truck. At the broken end the deck dips under, and the second
// half of the bridge has gone altogether: from there a truck has to wade. Piles and ropes are
// for looking at, and are not solid.
void Terrain::buildLogBridge()
{
    const DirtRoad& bed = m_riverbed;
    const Vec3& along = bed.m_cross;   // the way the shortcut runs
    const Vec3 right = -bed.m_forward; // to the right of a truck driving it
    const Vec3 up(0.0, 1.0, 0.0);
    const double level = getWaterLevel();
    const double deck = level + m_deckRise; // top of the logs along the right hand edge
    const double start = -bed.halfWidth() - deck * bed.m_slopeRun / bed.m_depth; // the near bank at that height
    const int count = (int) std::lround(-start / (2.0 * m_logRadius));
    const double pitch = -start / count;
    const auto at = [&](double run, double side, double height)
    {
        return m_ford + along * run + right * side + up * height;
    };
    const auto bank = [&](double run)
    {
        return bed.height(m_ford + along * run);
    };
    // Top of the logs along the right hand edge: level, then down under the water at the broken end
    const auto edge = [&](double run)
    {
        return deck - (m_deckRise + m_dipDepth) * clamp((run + m_dipRun) / m_dipRun, 0.0, 1.0);
    };
    require(deck < 0.0 && level > -bed.m_depth && start < -m_dipRun,
        "Terrain::buildLogBridge: the bridge must lie between the water and the top of the bank");
    require(m_deckRise * m_logLength / (deck + bed.m_depth - 2.0 * m_logRadius) > m_laneWidth - 0.6,
        "Terrain::buildLogBridge: the dry lane is too narrow for a truck");

    { // The logs. Each lies from wherever the ground holds up its left hand end to the right hand edge.
        for (int log = 0; log < count; ++log)
        {
            const double run = start + pitch * (log + 0.5);
            const double radius = m_logRadius * (1.0 + 0.04 * ((log * 37) % 5 - 2));
            const double high = edge(run) - radius;
            const double low = std::min(high, bank(run) + radius);
            const double overhang = 0.08 * ((log * 53) % 5);
            const double tail = 0.12 * ((log * 29) % 4);
            const double slope = (high - low) / m_logLength;
            addLog(at(run, m_bridgeEdge - m_logLength - tail, low - slope * tail),
                at(run, m_bridgeEdge + overhang, high + slope * overhang), radius,
                (log * 37) % 3 == 0 ? Face::Surface::LogPale : Face::Surface::LogDark, true);

            // The part of it under water shows through as a darker shape on the surface
            const double dry = slope > 1.0e-9 ? clamp((high + radius - level) / slope, 0.0, m_logLength) : m_logLength;
            if (dry < m_logLength)
            {
                const double shimmer = 0.012;
                const double half = 0.8 * radius;
                addFace(Face(at(run - half, m_bridgeEdge - m_logLength - tail, level + shimmer),
                    at(run - half, m_bridgeEdge - dry, level + shimmer), at(run + half, m_bridgeEdge - dry, level + shimmer),
                    at(run + half, m_bridgeEdge - m_logLength - tail, level + shimmer), Face::Surface::Sunken, false));
            }
        }
    }

    const double pileSide = m_bridgeEdge + 0.55; // clear of the ends of the logs
    const double pileRadius = 0.16;
    { // The right hand edge still stands on its beam and piles; the piles carry a rope
        const double beam = 2.0 * m_logRadius + 0.22; // from the top of the logs down to the middle of the beam
        addLog(at(start + 0.4, m_bridgeEdge - 0.5, deck - beam), at(-m_dipRun, m_bridgeEdge - 0.5, deck - beam),
            0.2, Face::Surface::Timber, false);
        addLog(at(-m_dipRun, m_bridgeEdge - 0.5, deck - beam), at(0.6, m_bridgeEdge - 0.5, edge(0.0) - beam),
            0.2, Face::Surface::Timber, false);

        const int piles = 4;
        const double height = 0.95; // of a pile above the deck [m]
        Vec3 previous;
        for (int pile = 0; pile < piles; ++pile)
        {
            const double run = start + 0.9 + (-m_dipRun - 0.4 - start - 0.9) * pile / (piles - 1.0);
            const Vec3 top = at(run, pileSide + 0.03 * (pile % 2), deck + height - 0.08 * (pile % 3));
            addLog(at(run, pileSide, bank(run) - 0.3), top, pileRadius, Face::Surface::Timber, false);
            if (pile > 0)
            {
                // The rope hangs in two straight lengths from one pile to the next
                const Vec3 sag = (previous + top) * 0.5 - up * 0.32;
                for (const Vec3& end : { previous - up * 0.12, top - up * 0.12 })
                {
                    addBox((end + sag) * 0.5, normalize(sag - end), right,
                        Vec3(0.5 * length(sag - end), 0.03, 0.03), Face::Surface::LogEnd, false);
                }
            }
            previous = top;
        }
        // The last pile leans over where the deck goes under
        addLog(at(-0.9, pileSide, -bed.m_depth - 0.3), at(-0.3, pileSide + 0.45, level + 0.75), pileRadius,
            Face::Surface::Timber, false);
    }

    { // Stumps of the piles that gave way, on the sunken side and where the second half stood
        const double runs[]  = { -8.5, -4.2, 3.6, 3.9, 8.4, 8.7 };
        const double sides[] = { m_bridgeEdge - m_logLength + 0.4, m_bridgeEdge - m_logLength + 0.7, pileSide + 0.6,
            m_bridgeEdge - m_logLength + 0.5, pileSide + 0.6, m_bridgeEdge - m_logLength + 0.3 };
        const double tops[]  = { 0.25, 0.1, 0.9, 0.35, 0.55, 0.15 }; // above the water [m]
        const double leans[] = { 0.3, -0.2, 0.15, 0.4, -0.25, 0.1 }; // of the top, downstream [m]
        for (int i = 0; i < 6; ++i)
        {
            addLog(at(runs[i], sides[i], bank(runs[i]) - 0.3), at(runs[i] + 0.1, sides[i] - leans[i], level + tops[i]),
                pileRadius, Face::Surface::Timber, false);
        }
    }
}

// A box of faces for drawing, with along, up and side as its axes. Only its top can be driven on,
// and only when solidTop is set.
void Terrain::addBox(const Vec3& centre, const Vec3& along, const Vec3& side, const Vec3& halfSize,
    Face::Surface surface, bool solidTop)
{
    const Vec3 x = along * halfSize.x;
    const Vec3 z = side * halfSize.z;
    const Vec3 y = normalize(cross(side, along)) * halfSize.y;
    addFace(Face(centre + y - z - x, centre + y + z - x, centre + y + z + x, centre + y - z + x, surface, solidTop));
    addFace(Face(centre - y - z - x, centre - y - z + x, centre - y + z + x, centre - y + z - x, surface, false));
    addFace(Face(centre + x - y - z, centre + x + y - z, centre + x + y + z, centre + x - y + z, surface, false));
    addFace(Face(centre - x - y - z, centre - x - y + z, centre - x + y + z, centre - x + y - z, surface, false));
    addFace(Face(centre + z - x - y, centre + z + x - y, centre + z + x + y, centre + z - x + y, surface, false));
    addFace(Face(centre - z - x - y, centre - z - x + y, centre - z + x + y, centre - z + x - y, surface, false));
}

// A curve that leaves one point in one direction and arrives at the other in another, tiled like
// the floor of the cutting, with its centreline added to path. shade continues the chequer from
// whatever it joins on to. side says which edge of the main road the junction is on: 1->right.
void Terrain::buildSlipRoad(std::vector<Vec3>& path, const Vec3& from, const Vec3& leaving, const Vec3& to,
    const Vec3& arriving, int shade, const PathPoint& junction, double side)
{
    const double reach = length(to - from);
    const double tile = tileSize();
    const int rows = std::max(1, (int) std::ceil(1.1 * reach / tile));
    const double lift = 0.02; // clear of the grass it lies on

    // Cubic Hermite curve: position and direction at a fraction t of the way
    const auto station = [&](double t, Vec3& position, Vec3& across)
    {
        const double t2 = t * t;
        const double t3 = t2 * t;
        position = from * (2.0*t3 - 3.0*t2 + 1.0) + leaving * (reach * (t3 - 2.0*t2 + t))
            + to * (-2.0*t3 + 3.0*t2) + arriving * (reach * (t3 - t2));
        const Vec3 along = normalize(from * (6.0*t2 - 6.0*t) + leaving * (reach * (3.0*t2 - 4.0*t + 1.0))
            + to * (-6.0*t2 + 6.0*t) + arriving * (reach * (3.0*t2 - 2.0*t)), leaving);
        across = Vec3(-along.z, 0.0, along.x);
        position.y = lift;
    };

    for (int row = 0; row < rows; ++row)
    {
        Vec3 p;
        Vec3 pAcross;
        Vec3 q;
        Vec3 qAcross;
        station(row / (double) rows, p, pAcross);
        station((row + 1) / (double) rows, q, qAcross);
        for (int column = 0; column < m_dirtColumns; ++column)
        {
            const double near = -m_dirt.m_halfFloor + tile * column;
            // Where the tile still lies under the main road, the road is what the wheels feel
            const Vec3 middle = (p + q) * 0.5 + (pAcross + qAcross) * (0.5 * (near + 0.5 * tile));
            const bool clear = side * dot(middle - junction.m_position, junction.m_cross) > m_track.getHalfWidth();
            addFace(Face(p + pAcross * near, p + pAcross * (near + tile),
                q + qAcross * (near + tile), q + qAcross * near,
                ((column + row + shade + (shade == 0 ? rows : 0)) & 1) != 0
                    ? Face::Surface::DirtPale : Face::Surface::DirtDark, clear));
        }
        path.push_back(p);
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
    m_barrier.m_broken = false;
}

bool Terrain::isSunk(const Vec3& point) const
{
    return m_dirt.contains(point) || m_riverbed.contains(point);
}

bool Terrain::isWading(const Vec3& point) const
{
    return point.y < getWaterLevel() && m_riverbed.contains(point);
}

// The solid ground under a point: the floor of the cutting or the riverbed where the ground is
// sunk, the plane at zero elsewhere
double Terrain::groundFloor(const Vec3& point) const
{
    return std::min(m_dirt.height(point), m_riverbed.height(point));
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
        if (p.m_kind == PathPoint::Kind::Flat && isSunk(p.m_position))
        {
            continue; // the road is cut where it crosses the dirt road
        }

        const bool raised = p.m_kind != PathPoint::Kind::Flat || q.m_kind != PathPoint::Kind::Flat;
        const bool loop = p.m_kind == PathPoint::Kind::Loop || q.m_kind == PathPoint::Kind::Loop;
        const bool open = p.m_kind == PathPoint::Kind::Bridge && q.m_kind == PathPoint::Kind::Bridge;

        // Level road lies in the ground plane; it is solid all the same, so that it is tarmac to the wheels
        addFace(Face(pl, pr, qr, ql, Face::Surface::Road, true));

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

    { // Ground plane, with holes in it where the cutting and the riverbed are sunk
        const double t = direction.y < 0.0 ? -origin.y / direction.y : -1.0;
        if (t >= 0.0 && origin.y >= 0.0 && t <= nearest && !isSunk(origin + direction * t))
        {
            nearest = t + m_groundSlack; // whatever lies in the plane, level road say, is felt in its place
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

    hit.m_speedLimit = found && isWading(hit.m_point) ? m_wadeSpeed : 0.0;
    return found;
}
