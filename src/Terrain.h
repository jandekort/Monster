// Monster - the collision world: ground plane, road surfaces, ramps and a crushable wreck
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Math.h"
#include "Track.h"

#include <array>
#include <vector>

// Struct Face: a quad of the world, used for drawing and (when solid) for collision
struct Face
{
    enum class Surface
    {
        Road,
        Stripe,   // edge marking on raised road
        Deck,     // top of a jump ramp
        Flank,    // side of a jump ramp
        Concrete, // bridge walls and underside
        DirtPale, // the two shades the dirt road is tiled in
        DirtDark,
        Bank      // the grassy sides of the cutting the dirt road runs in
    };

    static double traction(Surface surface); // 1->tarmac, less->looser going

    Vec3 m_a;
    Vec3 m_b;
    Vec3 m_c;
    Vec3 m_d;
    Vec3 m_normal;
    Surface m_surface = Surface::Road;
    bool m_solid = true; // true->collides, false->drawn only

    Face() = default;
    Face(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, Surface surface, bool solid);
};

// Struct Hit: result of a ray cast
struct Hit
{
    double m_distance = 0.0;
    Vec3 m_point;
    Vec3 m_normal;        // unit, facing the ray origin
    double m_traction = 1.0; // of the surface struck
    int m_wreck = -1;     // index of the wreck the ray hit, negative->not a wreck
};

// Struct Slab: a jump ramp piece on the start straight, linear in x
struct Slab
{
    double m_x0 = 0.0;
    double m_x1 = 0.0;
    double m_h0 = 0.0; // height at m_x0
    double m_h1 = 0.0; // height at m_x1
    double m_halfWidth = 0.0;

    Slab(double x0, double x1, double h0, double h1, double halfWidth);
};

// Class Wreck: an old car lying across the road that flattens under load
class Wreck
{
public:
    explicit Wreck(const PathPoint& at);

    // Mutator
    void applyLoad(double load, double dt);
    void reset() { m_crush = 0.0; }

    // Accessor
    std::array<Face, 3> faces() const; // roof and the two sloping flanks
    double getCrush() const { return m_crush; } // 0->intact, 1->flat
    double getCurrentHeight() const;
    double getReach() const; // radius around the car outside which nothing can hit
    const Vec3& getPosition() const { return m_position; } // on the road surface
    const Vec3& getForward() const { return m_forward; }   // along the road
    const Vec3& getUp() const { return m_up; }
    const Vec3& getCross() const { return m_cross; }       // across the road
    double getHalfWidth() const { return m_halfWidth; }
    double getHalfLength() const { return m_halfLength; }
    double getSlopeRun() const { return m_slopeRun; }

private:
    Vec3 m_position;
    Vec3 m_forward;
    Vec3 m_up;
    Vec3 m_cross;
    double m_halfWidth;       // along the road
    double m_halfLength;      // across the road
    double m_slopeRun;        // run of the sloped flanks along the road
    double m_height;          // intact height
    double m_minFraction;     // remaining height fraction when flat
    double m_loadThreshold;   // loads below this do not deform [N]
    double m_crushResistance; // impulse needed to flatten [N s]
    double m_crush;
};

// Struct DirtRoad: a sunken dirt track running beside the bridge, which the road from the start
// line jumps over. Its cross section is a flat floor with a slope up to ground level either side.
struct DirtRoad
{
    Vec3 m_centre;      // on the ground, at the middle of its length
    Vec3 m_forward;     // along the dirt road
    Vec3 m_cross;       // across it
    double m_halfFloor = 4.0;
    double m_slopeRun = 2.5;  // horizontal run of the banks, each side
    double m_halfLength = 30.0;
    double m_endRun = 12.0;   // the ends climb back to ground level over this distance
    double m_depth = 1.7;

    double halfWidth() const { return m_halfFloor + m_slopeRun; }
    bool contains(const Vec3& point) const;
    double height(const Vec3& point) const; // ground level over the dirt road, 0 or below
};

// Class Terrain
class Terrain
{
public:
    Terrain();

    // Nearest surface along a ray; direction must be a unit vector
    bool raycast(const Vec3& origin, const Vec3& direction, double reach, Hit& hit) const;

    // Mutator
    void crush(size_t index, double load, double dt) { m_wrecks.at(index).applyLoad(load, dt); }
    void reset();

    // Accessor
    const Track& getTrack() const { return m_track; }
    const std::vector<Face>& getFaces() const { return m_faces; }
    const std::vector<Slab>& getSlabs() const { return m_slabs; }
    const std::vector<Wreck>& getWrecks() const { return m_wrecks; }
    double getCrush() const;
    const DirtRoad& getDirtRoad() const { return m_dirt; }
    const std::vector<Vec3>& getDirtPath() const { return m_dirtPath; } // centreline, road to road
    size_t getSlipLeave() const { return m_slipLeave; }   // track point where the dirt road leaves
    size_t getSlipRejoin() const { return m_slipRejoin; } // and the one where it comes back
    double groundFloor(const Vec3& point) const; // solid ground under a point, 0 off the dirt road // how far through the row of wrecks, 0->all intact, 1->all flat

private:
    // Struct Triangle: precomputed for the ray test
    struct Triangle
    {
        Vec3 m_a;
        Vec3 m_ab;
        Vec3 m_ac;
        Vec3 m_normal;
        double m_traction;
    };

    void placeDirtRoad();
    void buildDirtRoad();
    void buildSlipRoads();
    void buildSlipRoad(const Vec3& from, const Vec3& leaving, const Vec3& to, const Vec3& arriving, int shade,
        const PathPoint& junction);
    void placeWrecks();
    void buildRoad();
    void buildSlabs();
    void buildGrid();
    void addFace(const Face& face) { m_faces.push_back(face); }
    static bool intersect(const Triangle& triangle, const Vec3& origin, const Vec3& direction, double reach, Hit& hit);
    static Triangle makeTriangle(const Vec3& a, const Vec3& b, const Vec3& c, double traction);

    Track m_track;
    DirtRoad m_dirt;
    double m_dirtOffset;  // from the bridge to the dirt road, to its right [m]
    int m_dirtColumns;    // tiles across the floor of the dirt road
    double m_slipGap;     // left between a slip road and the edge of the main road [m]
    double m_slipSplay;   // how sharply a slip road turns off the main road, as sideways per forward
    double m_groundSlack; // by how much a surface lying in the ground plane wins over it [m]
    std::vector<Vec3> m_dirtPath;
    size_t m_slipLeave;
    size_t m_slipRejoin;
    std::vector<Slab> m_slabs;
    std::vector<Wreck> m_wrecks;
    size_t m_wreckCount;
    double m_wreckLeadIn;  // distance past the last gate to the first wreck [m]
    double m_wreckSpacing; // between one wreck and the next [m]
    std::vector<Face> m_faces;
    std::vector<Triangle> m_triangles;

    // Uniform grid over the solid triangles in the ground plane
    double m_cellSize;
    double m_gridX;
    double m_gridZ;
    int m_gridColumns;
    int m_gridRows;
    std::vector<std::vector<int>> m_cells;

    double m_grassTraction; // the going off the road
    double m_stripeWidth;   // edge marking on raised road [m]
    double m_deckThickness; // drawn thickness of the bridge deck [m]
};
