// Monster - centreline of the circuit: a figure eight with a bridge and a loop
// Author: Jan de Kort
// Created: 2026-11-14
#include "Track.h"

#include "Check.h"

#include <algorithm>
#include <cmath>

Track::Track()
    :
    m_points()
    , m_halfWidth(5.0)
    , m_spacing(2.0)
    , m_arcStep(5.0 * M_PI / 180.0)
    , m_loopSegments(48)
    , m_bridgeHeight(6.0)
    , m_bridgeRamp(50.0)
    , m_bridgeDeck(12.0)
    , m_tunnelHalf(9.0)
    , m_checkpointCount(4)
    , m_gateMargin(0.8)
    , m_gateClearance(8.0)
    , m_cursor(20.0, 0.0, 0.0)
    , m_angle(0.0)
{
    // The two straights cross at (crossing, 0): the start straight runs along +x on the
    // ground, the bridge straight runs along -z over it. Each lobe of the eight is a
    // rectangle with three rounded corners.
    const double crossing = 62.0;   // x of the crossing; the start line is at x = 20, with the
                                    // starting grid on the straight behind it
    const double corner = 25.0;     // radius of every bend
    const double quarter = 0.5 * M_PI;
    const double startRun = 160.0;  // start straight beyond the crossing: jump and braking
    const double sideRun = 50.0;    // short sides of both lobes
    const double loopRunUp = 75.0;
    const double loopRadius = 10.0;
    const double loopShift = 10.5;  // exit lane lies this far right of the entry lane
    const double bridgeExit = 80.0; // bridge straight beyond the crossing

    addPoint(m_cursor, heading(), rightward(), Vec3(0.0, 1.0, 0.0), PathPoint::Kind::Flat, false, 0.0);

    { // Right lobe: wreck, tunnel, jump, then the loop on the back straight
        addStraight(crossing - m_cursor.x + startRun);
        addArc(corner, quarter);
        addStraight(sideRun + loopShift);
        addArc(corner, quarter);
        addStraight(loopRunUp);
        const double advance = addLoop(loopRadius, loopShift);
        addStraight(startRun - corner - loopRunUp - advance);
        addArc(corner, quarter);
    }

    { // Left lobe: over the bridge and back to the start line
        addBridge(corner + sideRun + bridgeExit, corner + sideRun);
        addArc(corner, -quarter);
        addStraight(sideRun);
        addArc(corner, -quarter);
        addStraight(bridgeExit - corner);
        addArc(corner, -quarter);
        addStraight(m_points.front().m_position.x - m_cursor.x);
    }

    require(length(m_cursor - m_points.front().m_position) < 1.0e-6, "Track: circuit does not close");

    chooseCheckpoints();
}

// A gate needs level, straight road around it, so that it stands upright beside the road and
// spans it squarely: never in a bend, on the bridge or inside the loop.
bool Track::canCarryGate(size_t index) const
{
    const double here = m_points[index].m_distance;
    return std::none_of(m_points.begin(), m_points.end(),
        [&](const PathPoint& point)
        {
            return std::fabs(point.m_distance - here) <= m_gateClearance
                && (point.m_kind != PathPoint::Kind::Flat || point.m_corner);
        });
}

// The gates are spread evenly over the lap, between one crossing of the start line and the
// next. Each one then moves to the nearest piece of road that can carry it.
void Track::chooseCheckpoints()
{
    const double spacing = getLength() / (m_checkpointCount + 1);
    const size_t none = m_points.size();

    m_checkpoints.clear();
    m_checkpoints.reserve(m_checkpointCount);
    for (size_t i = 0; i < m_checkpointCount; ++i)
    {
        const double target = spacing * (i + 1);
        size_t best = none;
        for (size_t index = 0; index + 1 < m_points.size(); ++index)
        {
            const bool after = m_checkpoints.empty() || index > m_checkpoints.back();
            const bool nearer = best == none
                || std::fabs(m_points[index].m_distance - target)
                    < std::fabs(m_points[best].m_distance - target);
            if (after && nearer && canCarryGate(index))
            {
                best = index;
            }
        }
        require(best < none, "Track::chooseCheckpoints: no road can carry a gate");
        m_checkpoints.push_back(best);
    }
}

Vec3 Track::heading() const
{
    return Vec3(std::cos(m_angle), 0.0, std::sin(m_angle));
}

Vec3 Track::rightward() const
{
    return Vec3(-std::sin(m_angle), 0.0, std::cos(m_angle));
}

void Track::addPoint(const Vec3& position, const Vec3& forward, const Vec3& cross, const Vec3& normal,
    PathPoint::Kind kind, bool corner, double step)
{
    PathPoint point;
    point.m_position = position;
    point.m_forward = forward;
    point.m_cross = cross;
    point.m_normal = normal;
    point.m_kind = kind;
    point.m_corner = corner;
    point.m_distance = m_points.empty() ? 0.0 : m_points.back().m_distance + step;
    m_points.push_back(point);
}

void Track::addStraight(double length)
{
    require(length > 0.0, "Track::addStraight: length must be positive");
    const int steps = std::max(1, (int) std::ceil(length / m_spacing));
    const Vec3 start = m_cursor;
    for (int i = 1; i <= steps; ++i)
    {
        m_cursor = start + heading() * (length * i / (double) steps);
        addPoint(m_cursor, heading(), rightward(), Vec3(0.0, 1.0, 0.0), PathPoint::Kind::Flat, false, length / steps);
    }
}

void Track::addArc(double radius, double angle)
{
    require(radius > 0.0, "Track::addArc: radius must be positive");
    const int steps = std::max(1, (int) std::ceil(std::fabs(angle) / m_arcStep));
    const double side = angle > 0.0 ? 1.0 : -1.0;
    const Vec3 center = m_cursor + rightward() * (side * radius);
    for (int i = 1; i <= steps; ++i)
    {
        m_angle += angle / steps;
        m_cursor = center - rightward() * (side * radius);
        addPoint(m_cursor, heading(), rightward(), Vec3(0.0, 1.0, 0.0), PathPoint::Kind::Flat, true,
            radius * std::fabs(angle) / steps);
    }
}

// A straight that rises over smooth ramps to a level deck centred on the crest
void Track::addBridge(double length, double crest)
{
    require(crest - m_bridgeDeck - m_bridgeRamp > 0.0 && crest + m_bridgeDeck + m_bridgeRamp < length,
        "Track::addBridge: ramps do not fit");
    const auto profile = [&](double s)
    {
        const double t = clamp((m_bridgeDeck + m_bridgeRamp - std::fabs(s - crest)) / m_bridgeRamp, 0.0, 1.0);
        return 0.5 * m_bridgeHeight * (1.0 - std::cos(M_PI * t));
    };

    const int steps = std::max(1, (int) std::ceil(length / m_spacing));
    const double step = length / steps;
    const Vec3 start = m_cursor;
    for (int i = 1; i <= steps; ++i)
    {
        const double s = step * i;
        const double h = profile(s);
        const double slope = (profile(s + 0.01) - profile(s - 0.01)) / 0.02;
        const Vec3 forward = normalize(heading() + Vec3(0.0, slope, 0.0));
        PathPoint::Kind kind = PathPoint::Kind::Flat;
        if (h > 0.0)
        {
            kind = std::fabs(s - crest) < m_tunnelHalf ? PathPoint::Kind::Bridge : PathPoint::Kind::Embankment;
        }
        m_cursor = start + heading() * s;
        addPoint(m_cursor + Vec3(0.0, h, 0.0), forward, rightward(), cross(rightward(), forward), kind, false, step);
    }
}

// A helix on a horizontal cylinder. The cylinder is turned slightly so that the road
// enters and leaves tangentially: a truck that keeps its wheels straight stays on it.
double Track::addLoop(double radius, double shift)
{
    require(radius > 0.0, "Track::addLoop: radius must be positive");
    require(std::fabs(shift) < 2.0 * M_PI * radius, "Track::addLoop: shift too large");
    const double circumference = 2.0 * M_PI * radius;
    const double skew = std::asin(std::fabs(shift) / circumference); // angle between road and loop plane
    const Vec3 lateral = rightward() * (shift > 0.0 ? 1.0 : -1.0);
    const Vec3 axis = heading() * std::sin(skew) + lateral * std::cos(skew);
    const Vec3 tangent = heading() * std::cos(skew) - lateral * std::sin(skew);
    const Vec3 upward(0.0, 1.0, 0.0);
    const double pitch = std::fabs(shift) / std::cos(skew); // travel along the axis per turn
    const double step = std::sqrt(circumference*circumference + pitch*pitch) / m_loopSegments;

    const Vec3 start = m_cursor;
    for (int i = 1; i <= m_loopSegments; ++i)
    {
        const double turn = i / (double) m_loopSegments;
        const double theta = 2.0 * M_PI * turn;
        const Vec3 position = start + tangent * (radius * std::sin(theta))
            + upward * (radius * (1.0 - std::cos(theta))) + axis * (pitch * turn);
        const Vec3 forward = normalize(tangent * (circumference * std::cos(theta))
            + upward * (circumference * std::sin(theta)) + axis * pitch);
        const Vec3 normal = upward * std::cos(theta) - tangent * std::sin(theta);
        addPoint(position, forward, normalize(cross(forward, normal)), normal, PathPoint::Kind::Loop, false, step);
    }
    m_cursor = start + axis * pitch;
    m_points.back().m_position = m_cursor;
    return pitch * std::sin(skew);
}
