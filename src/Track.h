// Monster - centreline of the circuit: a figure eight with a bridge and a loop
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Math.h"

#include <vector>

// Struct PathPoint: one cross-section of the road
struct PathPoint
{
    enum class Kind
    {
        Flat,       // painted on the ground
        Embankment, // raised, closed walls down to the ground
        Bridge,     // raised, open underneath
        Loop
    };

    Vec3 m_position;         // centre of the road
    Vec3 m_forward;          // unit tangent
    Vec3 m_cross;            // unit vector to the right edge
    Vec3 m_normal;           // unit vector out of the road surface
    Kind m_kind = Kind::Flat;
    bool m_corner = false;   // true->part of a bend, false->straight
    double m_distance = 0.0; // along the centreline from the start line [m]
};

// Class Track
// Plan view: x forward at the start line, z to the right.
class Track
{
public:
    Track();

    // Accessor
    const std::vector<PathPoint>& getPoints() const { return m_points; }
    double getHalfWidth() const { return m_halfWidth; }
    double getLength() const { return m_points.back().m_distance; }
    double getBridgeHeight() const { return m_bridgeHeight; }

private:
    void addPoint(const Vec3& position, const Vec3& forward, const Vec3& cross, const Vec3& normal,
        PathPoint::Kind kind, bool corner, double step);
    void addStraight(double length);
    void addArc(double radius, double angle); // angle in radians, positive->right
    void addBridge(double length, double crest);
    double addLoop(double radius, double shift); // shift positive->right; returns the forward advance
    Vec3 heading() const;
    Vec3 rightward() const;

    std::vector<PathPoint> m_points;
    double m_halfWidth;
    double m_spacing;     // sample distance on straights [m]
    double m_arcStep;     // sample angle in bends [rad]
    int m_loopSegments;
    double m_bridgeHeight;
    double m_bridgeRamp;  // length of each approach ramp [m]
    double m_bridgeDeck;  // half length of the level deck [m]
    double m_tunnelHalf;  // half width of the opening under the deck [m]

    // Cursor used while laying out the circuit
    Vec3 m_cursor;
    double m_angle;       // heading in the ground plane, 0->+x, positive->right
};
