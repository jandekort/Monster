// Monster - chase camera and scene mesh builder
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Math.h"
#include "Mesh.h"

#include <vector>

class Game;
class Terrain;
class Track;
struct PathPoint;
class Truck;
class Wreck;

struct Barrier;

// Class ChaseCamera
class ChaseCamera
{
public:
    ChaseCamera();

    // Mutator
    void snap() { m_initialised = false; } // jump to the truck on the next update
    void update(const Truck& truck, double dt);

    // Accessor
    Mat4 viewProjection(double aspect) const;
    const Vec3& getEye() const { return m_eye; }

private:
    double m_distance;     // behind the truck
    double m_height;       // above the truck
    double m_invertedDistance; // fraction of m_distance when the truck is upside down
    double m_headingRate;  // [1/s]
    double m_positionRate; // [1/s]
    double m_fieldOfView;
    double m_minHeight;    // above the ground
    double m_near;
    double m_far;
    bool m_initialised;    // false->snap on next update
    Vec3 m_heading;        // smoothed horizontal driving direction
    Vec3 m_eye;
    Vec3 m_target;
};

// Class Scene
class Scene
{
public:
    Scene();

    // Mutator
    void build(const Game& game, const Vec3& eye);

    // Accessor
    const Mesh& getMesh() const { return m_mesh; }

private:
    // Struct Pole: a wooden telephone pole beside the road
    struct Pole
    {
        Vec3 m_foot;
        Vec3 m_out;   // level unit vector away from the road; the cross-arms reach back over the verge
        bool m_wired; // true->the wires run on to the next pole
    };

    // Struct Bend: a bend of the circuit, by its track points
    struct Bend
    {
        size_t m_start;
        size_t m_end;
        int m_outside; // side of the outside, in units of the track points' m_cross
        bool m_quiet;  // true->no gate, junction, wreck, barrier or stunt near
    };

    // Struct Sign: a board of arrows on the outside of a bend
    struct Sign
    {
        Vec3 m_foot;
        Vec3 m_face;  // level unit vector out of the board, towards the traffic coming up to it
        Vec3 m_arrow; // level unit vector along the board, the way the arrows point
    };

    void addGround(const Terrain& terrain, const Vec3& eye);
    void addFaces(const Terrain& terrain);
    void addMarkings(const Terrain& terrain);
    void addCheckpoints(const Track& track);
    void addGate(const PathPoint& point, double halfWidth, const Color& banner, const char* label,
        double textHeight);
    void addText(const Vec3& center, const Vec3& right, const Vec3& up, double height,
        const char* text, const Color& color);
    void addGlyph(const Vec3& center, const Vec3& right, const Vec3& up, double pixel,
        const unsigned char* rows, const Color& color);
    void plantTrees(const Terrain& terrain);
    std::vector<Bend> findBends(const Terrain& terrain) const;
    bool siteIsClear(const Terrain& terrain, const Vec3& foot, double at, double reach) const;
    void raisePoles(const Terrain& terrain);
    void placeSigns(const Terrain& terrain);
    void addSigns();
    void addPoles();
    void addWire(const Vec3& from, const Vec3& to, const Color& color);
    void addWreck(const Wreck& wreck);
    void addBarrier(const Barrier& barrier);
    void addWater(const Terrain& terrain, double time);
    void addRiverside(const Terrain& terrain, double time);
    void addSpray(const Truck& truck, const Terrain& terrain, double time);
    void addScenery();
    void addTruck(const Truck& truck, const Color& paint);
    void addWheel(const Vec3& center, const Vec3& axle, const Vec3& up, double radius, double halfWidth, double spin);

    Mesh m_mesh;
    double m_tileSize;
    int m_tileRange;   // tiles drawn in each direction around the camera
    int m_wheelSides;
    double m_gateHeight;      // top of the poles above the road [m]
    double m_gateBanner;      // underside of the banner, which must clear the chase camera as
                              // well as the truck, or it fills the screen on the way through [m]
    double m_poleHalfWidth;
    double m_bannerHalfDepth; // thickness of the banner along the road [m]
    double m_digitHeight;     // height of a checkpoint number on its banner [m]
    double m_labelHeight;     // height of the start and finish lettering [m]
    int m_glyphColumns;       // banner font, pixels across
    int m_glyphRows;          // banner font, pixels down
    double m_treeSpacing;
    double m_treeClearance; // distance kept from the road centre [m]
    bool m_planted;         // true->m_trees and m_poles are filled
    std::vector<Vec3> m_trees; // foot position; y holds the size factor
    double m_poleHeight;
    double m_poleSpacing;   // along the centreline [m]
    double m_poleOffset;    // a pole stands this far beyond the edge of the road [m]
    double m_lineRun;       // a telephone line runs on this far either side of its bend [m]
    double m_quietRun;      // a bend with a gate, a junction, a wreck or a stunt this near has none [m]
    double m_wireSag;       // drop of a wire halfway between poles [m]
    double m_wireRadius;
    std::vector<Pole> m_poles;
    double m_signOffset;    // a sign stands this far beyond the edge of the road [m]
    double m_signLead;      // and faces the road this far before it [m]
    int m_signsPerBend;
    std::vector<Sign> m_signs;
};
