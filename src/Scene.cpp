// Monster - chase camera and scene mesh builder
// Author: Jan de Kort
// Created: 2026-11-14
#include "Scene.h"

#include "Check.h"
#include "Game.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <cmath>

namespace
{
    // Struct Glyph: one character of the banner font, five pixels across and seven down. In each
    // row the highest of the five bits is the leftmost pixel. Only the characters the gates and
    // the sign at the bridge use are carried.
    struct Glyph
    {
        char m_character;
        unsigned char m_rows[7];
    };

    const Glyph g_font[] = {
        { ' ', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
        { '/', { 0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10 } },
        { '0', { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E } },
        { '1', { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E } },
        { '2', { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F } },
        { '3', { 0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E } },
        { '4', { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 } },
        { '5', { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E } },
        { '6', { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E } },
        { '7', { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 } },
        { '8', { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E } },
        { '9', { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C } },
        { 'A', { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 } },
        { 'B', { 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E } },
        { 'C', { 0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E } },
        { 'D', { 0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E } },
        { 'E', { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F } },
        { 'F', { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 } },
        { 'G', { 0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F } },
        { 'H', { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 } },
        { 'I', { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F } },
        { 'L', { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F } },
        { 'N', { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 } },
        { 'O', { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E } },
        { 'R', { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 } },
        { 'S', { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E } },
        { 'T', { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 } }
    };

    // Struct Rect: a rectangle in the ground plane, square to the world
    struct Rect
    {
        double m_x0;
        double m_x1;
        double m_z0;
        double m_z1;

        bool isEmpty() const { return m_x1 - m_x0 <= 1.0e-9 || m_z1 - m_z0 <= 1.0e-9; }
        Rect clippedTo(const Rect& other) const
        {
            return Rect{ std::max(m_x0, other.m_x0), std::min(m_x1, other.m_x1),
                std::max(m_z0, other.m_z0), std::min(m_z1, other.m_z1) };
        }
    };

    // The footprint of a trough sunk into the ground. It lies square to the world.
    Rect footprint(const DirtRoad& sunk)
    {
        const double halfX = std::fabs(sunk.m_cross.x) * sunk.halfWidth() + std::fabs(sunk.m_forward.x) * sunk.m_halfLength;
        const double halfZ = std::fabs(sunk.m_cross.z) * sunk.halfWidth() + std::fabs(sunk.m_forward.z) * sunk.m_halfLength;
        return Rect{ sunk.m_centre.x - halfX, sunk.m_centre.x + halfX, sunk.m_centre.z - halfZ, sunk.m_centre.z + halfZ };
    }

    const unsigned char* glyphFor(char character)
    {
        const auto found = std::find_if(std::begin(g_font), std::end(g_font),
            [&](const Glyph& glyph) { return glyph.m_character == character; });
        require(found != std::end(g_font), "Scene: the banner font has no such character");
        return found->m_rows;
    }
}

// Class ChaseCamera
ChaseCamera::ChaseCamera()
    :
    m_distance(11.0)
    , m_height(4.0)
    , m_invertedDistance(0.5)
    , m_headingRate(4.0)
    , m_positionRate(6.0)
    , m_fieldOfView(60.0 * M_PI / 180.0)
    , m_minHeight(1.0)
    , m_near(1.0)
    , m_far(4000.0)
    , m_initialised(false)
    , m_heading(1.0, 0.0, 0.0)
    , m_eye()
    , m_target()
{
}

void ChaseCamera::update(const Truck& truck, double dt)
{
    const Vec3& position = truck.getPosition();
    const Vec3 f = truck.forward();
    const Vec3 flat(f.x, 0.0, f.z);
    // Hold the direction while the truck is steep or upside down, as in the loop
    const Vec3 heading = truck.up().y > 0.5 && length(flat) > 0.3 ? normalize(flat) : m_heading;

    const double headingBlend = m_initialised ? std::min(1.0, m_headingRate * dt) : 1.0;
    m_heading = normalize(m_heading + (heading - m_heading) * headingBlend, heading);

    // Away from upright the camera follows the truck's own axes instead of the ground's,
    // which keeps it inside the loop: behind the truck and towards the loop centre
    const double upright = 0.5 * (1.0 + clamp(truck.up().y, -1.0, 1.0)); // 1->wheels down, 0->wheels up
    const Vec3 back = normalize(m_heading * upright + f * (1.0 - upright), m_heading);
    const Vec3 above = normalize(Vec3(0.0, upright, 0.0) + truck.up() * (1.0 - upright), truck.up());
    const double distance = m_distance * (m_invertedDistance + (1.0 - m_invertedDistance) * upright);
    const Vec3 desired = position - back * distance + above * m_height;
    const double positionBlend = m_initialised ? std::min(1.0, m_positionRate * dt) : 1.0;
    m_eye += (desired - m_eye) * positionBlend;
    m_eye.y = std::max(m_eye.y, m_minHeight);
    m_target = position + above * 1.2 + back * 2.0;
    m_initialised = true;
}

Mat4 ChaseCamera::viewProjection(double aspect) const
{
    return Mat4::perspective(m_fieldOfView, aspect, m_near, m_far)
        * Mat4::lookAt(m_eye, m_target, Vec3(0.0, 1.0, 0.0));
}

// Class Scene
Scene::Scene()
    :
    m_mesh()
    , m_tileSize(20.0)
    , m_tileRange(24)
    , m_wheelSides(12)
    , m_gateHeight(8.0)
    , m_gateBanner(6.2)
    , m_poleHalfWidth(0.22)
    , m_bannerHalfDepth(0.14)
    , m_digitHeight(1.3)
    , m_labelHeight(0.75)
    , m_glyphColumns(5)
    , m_glyphRows(7)
    , m_treeSpacing(24.0)
    , m_treeClearance(14.0)
    , m_planted(false)
    , m_trees()
    , m_poleHeight(8.0)
    , m_poleSpacing(24.0)
    , m_poleOffset(4.0)
    , m_lineRun(60.0)
    , m_quietRun(30.0)
    , m_wireSag(0.35)
    , m_wireRadius(0.025)
    , m_poles()
{
}

void Scene::build(const Game& game, const Vec3& eye)
{
    m_mesh.clear();
    addGround(game.getTerrain(), eye);
    addFaces(game.getTerrain());
    addMarkings(game.getTerrain());
    addCheckpoints(game.getTerrain().getTrack());
    for (const Wreck& wreck : game.getTerrain().getWrecks())
    {
        addWreck(wreck);
    }
    addBarrier(game.getTerrain().getBarrier());
    addWater(game.getTerrain(), game.getRaceTime());
    addRiverside(game.getTerrain(), game.getRaceTime());
    if (!m_planted)
    {
        plantTrees(game.getTerrain());
        raisePoles(game.getTerrain());
        m_planted = true;
    }
    addPoles();
    addScenery();
    for (const Racer& racer : game.getRacers())
    {
        addTruck(racer.m_truck, racer.m_color);
        addSpray(racer.m_truck, game.getTerrain(), game.getRaceTime());
    }
}

// The grass is laid in tiles, with the footprints of the cutting and the riverbed cut out of
// them: those have their own tiles, and grass drawn over the holes would hide them entirely.
void Scene::addGround(const Terrain& terrain, const Vec3& eye)
{
    const Vec3 upward(0.0, 1.0, 0.0);
    const Color light(0.30f, 0.62f, 0.26f);
    const Color dark(0.26f, 0.56f, 0.23f);
    const Color distant(0.28f, 0.59f, 0.245f);
    const std::vector<Rect> holes = { footprint(terrain.getDirtRoad()), footprint(terrain.getRiverbed()) };

    { // Far field, below the deepest the ground is cut away, so it never shows through a hole
        const double reach = 5000.0;
        const double y = -std::max(terrain.getDirtRoad().m_depth, terrain.getRiverbed().m_depth) - 1.0;
        m_mesh.addQuad(Vec3(eye.x - reach, y, eye.z - reach), Vec3(eye.x + reach, y, eye.z - reach),
            Vec3(eye.x + reach, y, eye.z + reach), Vec3(eye.x - reach, y, eye.z + reach), distant, upward);
    }

    const auto patch = [&](const Rect& rect, const Color& color)
    {
        if (!rect.isEmpty())
        {
            m_mesh.addQuad(Vec3(rect.m_x0, 0.0, rect.m_z0), Vec3(rect.m_x1, 0.0, rect.m_z0),
                Vec3(rect.m_x1, 0.0, rect.m_z1), Vec3(rect.m_x0, 0.0, rect.m_z1), color, upward);
        }
    };

    { // Checkered tiles around the camera give a sense of speed
        const long cx = std::lround(std::floor(eye.x / m_tileSize));
        const long cz = std::lround(std::floor(eye.z / m_tileSize));
        std::vector<Rect> pieces;
        std::vector<Rect> left;
        for (long ix = cx - m_tileRange; ix <= cx + m_tileRange; ++ix)
        {
            for (long iz = cz - m_tileRange; iz <= cz + m_tileRange; ++iz)
            {
                const Rect tile{ ix * m_tileSize, (ix + 1) * m_tileSize, iz * m_tileSize, (iz + 1) * m_tileSize };
                const Color& color = ((ix + iz) & 1) != 0 ? light : dark;
                const bool whole = std::all_of(holes.begin(), holes.end(),
                    [&](const Rect& hole) { return tile.clippedTo(hole).isEmpty(); });
                if (whole)
                {
                    patch(tile, color);
                    continue;
                }

                // Each hole takes its share out of whatever is left of the tile
                pieces.assign(1, tile);
                for (const Rect& hole : holes)
                {
                    left.clear();
                    for (const Rect& piece : pieces)
                    {
                        const Rect cut = piece.clippedTo(hole);
                        if (cut.isEmpty())
                        {
                            left.push_back(piece);
                            continue;
                        }
                        left.push_back(Rect{ piece.m_x0, cut.m_x0, piece.m_z0, piece.m_z1 }); // the four pieces
                        left.push_back(Rect{ cut.m_x1, piece.m_x1, piece.m_z0, piece.m_z1 }); // around the hole
                        left.push_back(Rect{ cut.m_x0, cut.m_x1, piece.m_z0, cut.m_z0 });
                        left.push_back(Rect{ cut.m_x0, cut.m_x1, cut.m_z1, piece.m_z1 });
                    }
                    pieces.swap(left);
                }
                for (const Rect& piece : pieces)
                {
                    patch(piece, color);
                }
            }
        }
    }
}

void Scene::addFaces(const Terrain& terrain)
{
    const Color asphalt(0.36f, 0.36f, 0.38f);
    const Color stripe(0.90f, 0.25f, 0.20f);
    const Color deck(0.88f, 0.72f, 0.30f);
    const Color flank(0.70f, 0.30f, 0.20f);
    const Color concrete(0.62f, 0.62f, 0.60f);
    const Color dirtPale(0.62f, 0.49f, 0.33f);
    const Color dirtDark(0.52f, 0.40f, 0.26f);
    const Color gravelPale(0.76f, 0.70f, 0.55f);
    const Color gravelDark(0.69f, 0.63f, 0.49f);
    const Color logPale(0.56f, 0.40f, 0.23f);
    const Color logDark(0.44f, 0.30f, 0.16f);
    const Color logEnd(0.84f, 0.70f, 0.45f);
    const Color timber(0.34f, 0.24f, 0.14f);
    const double roadLift = 0.03;   // keeps the road clear of the ground it lies on
    const double stripeLift = 0.06;

    for (const Face& face : terrain.getFaces())
    {
        Color color;
        double lift = 0.0;
        switch (face.m_surface)
        {
        case Face::Surface::Road:
            color = asphalt;
            lift = roadLift;
            break;
        case Face::Surface::Stripe:
            color = stripe;
            lift = stripeLift;
            break;
        case Face::Surface::Deck:
            color = deck;
            break;
        case Face::Surface::Flank:
            color = flank;
            break;
        case Face::Surface::Concrete:
            color = concrete;
            break;
        case Face::Surface::DirtPale:
            color = dirtPale;
            break;
        case Face::Surface::DirtDark:
            color = dirtDark;
            break;
        case Face::Surface::Bank:
            color = Color(0.30f, 0.62f, 0.26f); // the field's own green, shaded by the slope it lies on
            break;
        case Face::Surface::GravelPale:
            color = gravelPale;
            break;
        case Face::Surface::GravelDark:
            color = gravelDark;
            break;
        case Face::Surface::LogPale:
            color = logPale;
            break;
        case Face::Surface::LogDark:
            color = logDark;
            break;
        case Face::Surface::LogEnd:
            color = logEnd;
            break;
        case Face::Surface::Sunken:
            color = Color(0.20f, 0.34f, 0.47f);
            break;
        case Face::Surface::Timber:
            color = timber;
            break;
        default:
            require(false, "Scene::addFaces: unknown surface");
        }
        const Vec3 offset = face.m_normal * lift;
        // The dirt road is lit as level ground, so that it is one colour from end to end.
        // The banks keep their real slope, so the side in shadow reads darker, as it did before
        const bool earth = face.m_surface == Face::Surface::DirtPale || face.m_surface == Face::Surface::DirtDark;
        m_mesh.addQuad(face.m_a + offset, face.m_b + offset, face.m_c + offset, face.m_d + offset, color,
            earth ? Vec3(0.0, 1.0, 0.0) : face.m_normal);
    }
}

void Scene::addMarkings(const Terrain& terrain)
{
    const Track& track = terrain.getTrack();
    const Color paint(0.95f, 0.95f, 0.90f);
    const std::vector<PathPoint>& points = track.getPoints();
    const double lift = 0.07;
    const double halfWidth = 0.15;
    const double pitch = 8.0; // one dash and one gap [m]

    for (size_t i = 0; i + 1 < points.size(); ++i)
    {
        const PathPoint& p = points[i];
        const PathPoint& q = points[i + 1];
        const bool missing = p.m_kind == PathPoint::Kind::Gap || q.m_kind == PathPoint::Kind::Gap
            || terrain.isSunk(p.m_position);
        if (!missing && std::fmod(p.m_distance, pitch) < 0.5 * pitch)
        {
            const Vec3 a = p.m_position + p.m_normal * lift;
            const Vec3 b = q.m_position + q.m_normal * lift;
            m_mesh.addQuad(a - p.m_cross * halfWidth, a + p.m_cross * halfWidth,
                b + q.m_cross * halfWidth, b - q.m_cross * halfWidth, paint, p.m_normal);
        }
    }

    { // Start line
        const PathPoint& start = points.front();
        const Vec3 a = start.m_position + start.m_normal * lift;
        const Vec3 b = a + start.m_forward * 0.6;
        const Vec3 side = start.m_cross * track.getHalfWidth();
        m_mesh.addQuad(a - side, a + side, b + side, b - side, paint, start.m_normal);
    }
}

// A gate at the start and finish line and at each numbered checkpoint
void Scene::addCheckpoints(const Track& track)
{
    const Color checkpoint(0.10f, 0.22f, 0.48f);
    const Color startFinish = checkpoint; // the line is a checkpoint like the rest
    const std::vector<PathPoint>& points = track.getPoints();
    const double halfWidth = track.getGateHalfWidth();

    addGate(track.getStartFinish(), halfWidth, startFinish, "START / FINISH", m_labelHeight);
    for (size_t i = 0; i < track.getCheckpoints().size(); ++i)
    {
        char label[12];
        std::snprintf(label, sizeof(label), "%d", (int) i + 1);
        addGate(points[track.getCheckpoints()[i]], halfWidth, checkpoint, label, m_digitHeight);
    }
}

// Two poles outside the road edge and a banner across it, carrying a label
void Scene::addGate(const PathPoint& point, double halfWidth, const Color& banner, const char* label,
    double textHeight)
{
    const Color pole(0.93f, 0.93f, 0.88f);
    const Color lettering(1.00f, 0.88f, 0.30f);
    const Vec3 up(0.0, 1.0, 0.0);
    const Vec3& f = point.m_forward;
    const Vec3& across = point.m_cross;
    const double bannerHalfHeight = 0.5 * (m_gateHeight - m_gateBanner);

    for (double side : { -1.0, 1.0 })
    {
        m_mesh.addBox(point.m_position + across * (side * halfWidth) + up * (0.5 * m_gateHeight),
            f, up, across, Vec3(m_poleHalfWidth, 0.5 * m_gateHeight, m_poleHalfWidth), pole);
    }

    const Vec3 middle = point.m_position + up * (m_gateBanner + bannerHalfHeight);
    m_mesh.addBox(middle, f, up, across, Vec3(m_bannerHalfDepth, bannerHalfHeight, halfWidth), banner);

    // The label faces the oncoming truck, which reads it with the road's right hand to its right
    addText(middle - f * (m_bannerHalfDepth + 0.03), across, up, textHeight, label, lettering);
}

void Scene::addText(const Vec3& center, const Vec3& right, const Vec3& up, double height,
    const char* text, const Color& color)
{
    const size_t count = std::strlen(text);
    require(count > 0, "Scene::addText: nothing to draw");

    const double pixel = height / m_glyphRows;
    const double advance = m_glyphColumns + 1.0; // one blank pixel between characters
    const double width = advance * count - 1.0;
    for (size_t i = 0; i < count; ++i)
    {
        const double offset = advance * i + 0.5 * m_glyphColumns - 0.5 * width;
        addGlyph(center + right * (offset * pixel), right, up, pixel, glyphFor(text[i]), color);
    }
}

void Scene::addGlyph(const Vec3& center, const Vec3& right, const Vec3& up, double pixel,
    const unsigned char* rows, const Color& color)
{
    const Vec3 normal = normalize(cross(right, up));
    const auto isLit = [&](int row, int column)
    {
        return (rows[row] & (1 << (m_glyphColumns - 1 - column))) != 0;
    };

    for (int row = 0; row < m_glyphRows; ++row)
    {
        for (int column = 0; column < m_glyphColumns; ++column)
        {
            if (!isLit(row, column))
            {
                continue;
            }

            // A run of lit pixels becomes one quad, so a solid bar is a single face
            int end = column;
            while (end + 1 < m_glyphColumns && isLit(row, end + 1))
            {
                ++end;
            }
            const Vec3 middle = center
                + right * ((0.5 * (column + end + 1) - 0.5 * m_glyphColumns) * pixel)
                + up * ((0.5 * m_glyphRows - row - 0.5) * pixel);
            const Vec3 halfX = right * (0.5 * (end - column + 1) * pixel);
            const Vec3 halfY = up * (0.5 * pixel);
            m_mesh.addQuad(middle - halfX - halfY, middle + halfX - halfY,
                middle + halfX + halfY, middle - halfX + halfY, color, normal);
            column = end;
        }
    }
}

namespace
{
    // In a hole or on its brink, where a tree or a pole would stand in the air
    bool besideSunk(const DirtRoad& sunk, const Vec3& foot)
    {
        const double margin = 4.0;
        const Vec3 offset = foot - sunk.m_centre;
        return std::fabs(dot(offset, sunk.m_cross)) < sunk.halfWidth() + margin
            && std::fabs(dot(offset, sunk.m_forward)) < sunk.m_halfLength + margin;
    }

    double levelDistance(const Vec3& a, const Vec3& b)
    {
        return std::hypot(a.x - b.x, a.z - b.z);
    }
}

// Trees on a jittered grid, kept clear of the road
void Scene::plantTrees(const Terrain& terrain)
{
    const Track& track = terrain.getTrack();

    const std::vector<PathPoint>& points = track.getPoints();
    double minX = points.front().m_position.x;
    double maxX = minX;
    double minZ = points.front().m_position.z;
    double maxZ = minZ;
    for (const PathPoint& point : points)
    {
        minX = std::min(minX, point.m_position.x);
        maxX = std::max(maxX, point.m_position.x);
        minZ = std::min(minZ, point.m_position.z);
        maxZ = std::max(maxZ, point.m_position.z);
    }

    const double margin = 3.0 * m_treeSpacing;
    int seed = 0;
    for (double x = minX - margin; x <= maxX + margin; x += m_treeSpacing)
    {
        for (double z = minZ - margin; z <= maxZ + margin; z += m_treeSpacing)
        {
            ++seed;
            const double jitterX = ((seed * 37) % 17) / 17.0 - 0.5;
            const double jitterZ = ((seed * 53) % 19) / 19.0 - 0.5;
            const Vec3 foot(x + 0.7 * m_treeSpacing * jitterX, 0.0, z + 0.7 * m_treeSpacing * jitterZ);
            const bool clear = std::all_of(points.begin(), points.end(),
                [&](const PathPoint& point)
                {
                    const double dx = point.m_position.x - foot.x;
                    const double dz = point.m_position.z - foot.z;
                    return dx*dx + dz*dz > m_treeClearance*m_treeClearance;
                });
            const bool offTheDirt = std::all_of(terrain.getDetours().begin(), terrain.getDetours().end(),
                [&](const Detour& detour)
                {
                    return std::all_of(detour.m_path.begin(), detour.m_path.end(),
                        [&](const Vec3& point)
                        {
                            const double dx = point.x - foot.x;
                            const double dz = point.z - foot.z;
                            return dx*dx + dz*dz > 0.5 * m_treeClearance*m_treeClearance;
                        });
                });
            const bool onFirmGround = !besideSunk(terrain.getDirtRoad(), foot) && !besideSunk(terrain.getRiverbed(), foot);
            if (clear && offTheDirt && onFirmGround && (seed * 29) % 5 != 0)
            {
                m_trees.push_back(Vec3(foot.x, 1.0 + 0.1 * ((seed * 5) % 7), foot.z));
            }
        }
    }
}

// Telephone lines follow the outside of the quiet bends, those with no gate, junction, wreck,
// barrier or stunt near, and run on along the road either side. A pole goes in every
// m_poleSpacing, unless it would stand by a gate, on another part of the circuit, by a dirt road
// or in a hole; there the wires break off.
void Scene::raisePoles(const Terrain& terrain)
{
    const Track& track = terrain.getTrack();
    const std::vector<PathPoint>& points = track.getPoints();

    std::vector<double> busy; // where along the lap something is going on [m]
    std::transform(track.getCheckpoints().begin(), track.getCheckpoints().end(), std::back_inserter(busy),
        [&](size_t gate) { return points[gate].m_distance; });
    busy.push_back(0.0);
    busy.push_back(track.getLength());
    for (const Detour& detour : terrain.getDetours())
    {
        busy.push_back(points[detour.m_leave].m_distance);
        busy.push_back(points[detour.m_rejoin].m_distance);
    }
    for (const PathPoint& point : points)
    {
        if (point.m_kind != PathPoint::Kind::Flat)
        {
            busy.push_back(point.m_distance);
        }
    }
    std::vector<Vec3> sights;
    std::transform(terrain.getWrecks().begin(), terrain.getWrecks().end(), std::back_inserter(sights),
        [](const Wreck& wreck) { return wreck.getPosition(); });
    sights.push_back(terrain.getBarrier().m_position);

    // The side a line runs on, in units of m_cross: the outside of its bend, 0->no line.
    // Where the lines of two bends that turn opposite ways meet, the line stops.
    std::vector<int> side(points.size(), 0);
    for (size_t start = 0; start < points.size(); )
    {
        size_t end = start;
        while (end + 1 < points.size() && points[end + 1].m_corner == points[start].m_corner)
        {
            ++end;
        }
        const double from = points[start].m_distance;
        const double to = points[end].m_distance;
        const bool quiet = std::none_of(busy.begin(), busy.end(),
            [&](double at) { return at > from - m_quietRun && at < to + m_quietRun; })
            && std::all_of(points.begin() + start, points.begin() + end + 1,
            [&](const PathPoint& point)
            {
                return std::all_of(sights.begin(), sights.end(),
                    [&](const Vec3& sight) { return levelDistance(sight, point.m_position) > m_quietRun; });
            });
        if (points[start].m_corner && quiet)
        {
            const Vec3 turn = points[end].m_forward - points[start].m_forward;
            const int outside = dot(turn, points[(start + end) / 2].m_cross) > 0.0 ? -1 : 1;
            for (size_t i = 0; i < points.size(); ++i)
            {
                const double at = points[i].m_distance;
                if (at > from - m_lineRun && at < to + m_lineRun && points[i].m_kind == PathPoint::Kind::Flat)
                {
                    side[i] = side[i] == 0 || side[i] == outside ? outside : 0;
                }
            }
        }
        start = end + 1;
    }

    const double reach = track.getHalfWidth() + m_poleOffset;
    const double lap = track.getLength();
    const auto siteIsClear = [&](const Vec3& foot, double at)
    {
        const bool offTheCircuit = std::all_of(points.begin(), points.end(),
            [&](const PathPoint& point)
            {
                const double along = std::fabs(point.m_distance - at);
                return std::min(along, lap - along) < 2.0 * reach
                    || levelDistance(point.m_position, foot) > reach - 1.0;
            });
        const bool byNoGate = std::all_of(track.getCheckpoints().begin(), track.getCheckpoints().end(),
            [&](size_t gate) { return levelDistance(points[gate].m_position, foot) > 2.0 * reach; })
            && levelDistance(track.getStartFinish().m_position, foot) > 2.0 * reach;
        const bool offTheDirt = std::all_of(terrain.getDetours().begin(), terrain.getDetours().end(),
            [&](const Detour& detour)
            {
                return std::all_of(detour.m_path.begin(), detour.m_path.end(),
                    [&](const Vec3& point) { return levelDistance(point, foot) > reach; });
            });
        return offTheCircuit && byNoGate && offTheDirt
            && !besideSunk(terrain.getDirtRoad(), foot) && !besideSunk(terrain.getRiverbed(), foot);
    };

    std::vector<int> lineOf; // which line each pole is on
    int line = 0;
    double lastPole = 0.0;
    for (size_t i = 0; i < points.size(); ++i)
    {
        const PathPoint& point = points[i];
        if (side[i] == 0 || i == 0 || side[i] != side[i - 1])
        {
            ++line;
            lastPole = point.m_distance - m_poleSpacing;
        }
        if (side[i] == 0 || point.m_distance - lastPole < m_poleSpacing)
        {
            continue;
        }
        lastPole = point.m_distance;
        const Vec3 out = normalize(Vec3(point.m_cross.x, 0.0, point.m_cross.z)) * (double) side[i];
        const Vec3 foot = Vec3(point.m_position.x, 0.0, point.m_position.z) + out * reach;
        if (!siteIsClear(foot, point.m_distance))
        {
            ++line;
            continue;
        }
        m_poles.push_back(Pole{ foot, out, false });
        lineOf.push_back(line);
    }
    for (size_t k = 0; k + 1 < m_poles.size(); ++k)
    {
        m_poles[k].m_wired = lineOf[k] == lineOf[k + 1];
    }
}

// A weathered round pole with two cross-arms: a short one at the top, reaching back over the
// verge, and a longer one below it across the pole, with glass insulators carrying the wires
void Scene::addPoles()
{
    const Color wood(0.47f, 0.35f, 0.24f);
    const Color sawn(0.62f, 0.50f, 0.34f);
    const Color timber(0.36f, 0.26f, 0.17f);
    const Color glass(0.60f, 0.80f, 0.72f);
    const Color wire(0.10f, 0.10f, 0.11f);
    const Vec3 up(0.0, 1.0, 0.0);

    // Struct Arm: a cross-arm, from the pole towards the road (negative) and away from it
    struct Arm
    {
        double m_height; // of its top, below the top of the pole [m]
        double m_near;
        double m_far;
    };
    const Arm arms[] = { { 0.02, -1.45, 0.10 }, { 0.80, -1.50, 0.80 } };
    const double armHalf = 0.06;
    const double poleRadius = 0.13;
    const double insulatorHeight = 0.14;

    // Struct Insulator: where along an arm it stands, and on which
    struct Insulator
    {
        size_t m_arm;
        double m_out;
    };
    const Insulator insulators[] = { { 0, -1.25 }, { 0, -0.55 }, { 1, -1.30 }, { 1, -0.55 }, { 1, 0.55 } };
    const auto tip = [&](const Pole& pole, const Insulator& insulator)
    {
        return pole.m_foot + pole.m_out * insulator.m_out
            + up * (m_poleHeight - arms[insulator.m_arm].m_height + 0.85 * insulatorHeight);
    };

    for (size_t k = 0; k < m_poles.size(); ++k)
    {
        const Pole& pole = m_poles[k];
        const Vec3 along = cross(pole.m_out, up);
        m_mesh.addCylinder(pole.m_foot - up * 0.2, pole.m_foot + up * m_poleHeight, poleRadius, wood, sawn);
        for (const Arm& arm : arms)
        {
            const Vec3 middle = pole.m_foot + pole.m_out * (0.5 * (arm.m_near + arm.m_far))
                + up * (m_poleHeight - arm.m_height - armHalf);
            m_mesh.addBox(middle, pole.m_out, up, along,
                Vec3(0.5 * (arm.m_far - arm.m_near), armHalf, 0.8 * armHalf), timber);
        }
        for (const Insulator& insulator : insulators)
        {
            const Vec3 base = pole.m_foot + pole.m_out * insulator.m_out
                + up * (m_poleHeight - arms[insulator.m_arm].m_height + 0.5 * insulatorHeight);
            m_mesh.addBox(base, pole.m_out, up, along, Vec3(0.04, 0.5 * insulatorHeight, 0.04), glass);
            if (pole.m_wired)
            {
                // Hanging in a curve, drawn as a few straight lengths
                const Vec3 from = tip(pole, insulator);
                const Vec3 to = tip(m_poles[k + 1], insulator);
                const int pieces = 4;
                const auto at = [&](int piece)
                {
                    const double t = piece / (double) pieces;
                    return from + (to - from) * t - up * (4.0 * m_wireSag * t * (1.0 - t));
                };
                for (int piece = 0; piece < pieces; ++piece)
                {
                    addWire(at(piece), at(piece + 1), wire);
                }
            }
        }
    }
}

void Scene::addWire(const Vec3& from, const Vec3& to, const Color& color)
{
    const Vec3 span = to - from;
    const double reach = length(span);
    require(reach > 0.0, "Scene::addWire: the ends must differ");
    const Vec3 axisX = span * (1.0 / reach);
    const Vec3 axisZ = normalize(cross(axisX, Vec3(0.0, 1.0, 0.0)), Vec3(0.0, 0.0, 1.0));
    const Vec3 axisY = cross(axisZ, axisX);
    m_mesh.addBox((from + to) * 0.5, axisX, axisY, axisZ, Vec3(0.5 * reach, m_wireRadius, m_wireRadius), color);
}

void Scene::addWreck(const Wreck& wreck)
{
    const Vec3& x = wreck.getForward();
    const Vec3& y = wreck.getUp();
    const Vec3& z = wreck.getCross();
    const double crush = wreck.getCrush();
    const double height = wreck.getCurrentHeight();
    const double spread = 1.0 + 0.12 * crush; // flattened metal spreads out
    const Color paint = Color(0.20f, 0.45f, 0.75f).scaled((float) (1.0 - 0.45 * crush));
    const Color glass = Color(0.75f, 0.88f, 0.95f).scaled((float) (1.0 - 0.60 * crush));
    const Color rubber(0.10f, 0.10f, 0.10f);

    const double bodyHeight = 0.62 * height;
    const double halfWidth = (wreck.getHalfWidth() + 0.3 * wreck.getSlopeRun()) * spread;
    const double halfLength = wreck.getHalfLength() * spread;
    const Vec3& base = wreck.getPosition();

    m_mesh.addBox(base + y * (0.5 * bodyHeight + 0.05), x, y, z,
        Vec3(halfWidth, 0.5 * bodyHeight, halfLength), paint);
    m_mesh.addBox(base + y * (bodyHeight + 0.5 * (height - bodyHeight)) + z * (-0.1 * halfLength), x, y, z,
        Vec3(0.85 * halfWidth, 0.5 * (height - bodyHeight), 0.5 * halfLength), glass);
    for (int i = 0; i < 4; ++i)
    {
        const double sx = (i & 1) != 0 ? 1.0 : -1.0;
        const double sz = (i & 2) != 0 ? 1.0 : -1.0;
        m_mesh.addBox(base + x * (sx * halfWidth) + y * 0.3 + z * (sz * 0.62 * halfLength), x, y, z,
            Vec3(0.12, 0.3, 0.33), rubber);
    }
}

// Before the bridge: a striped board across the road, which stands until a truck drives through
// it and lies in pieces on the road after that, and a sign on the verge that stays where it is
void Scene::addBarrier(const Barrier& barrier)
{
    const Color red(0.85f, 0.15f, 0.12f);
    const Color white(0.95f, 0.95f, 0.92f);
    const Color steel(0.35f, 0.35f, 0.37f);
    const Color board(0.97f, 0.80f, 0.12f);
    const Color lettering(0.08f, 0.08f, 0.08f);
    const Vec3 up(0.0, 1.0, 0.0);
    const Vec3& f = barrier.m_forward;
    const Vec3& across = barrier.m_cross;
    const Vec3& base = barrier.m_position;

    { // The board, in three lengths of red and white stripes
        const int stripes = 10;
        const double stripe = 2.0 * barrier.m_halfWidth / stripes;
        const double halfThickness = 0.05;
        const double halfHeight = 0.22;
        const double railHeight = 1.05;
        const int lengths[] = { 3, 4, 3 };
        const double thrown[] = { 3.2, 5.4, 3.9 };  // how far along the road each length lands [m]
        const double turned[] = { 0.5, -0.2, -0.7 }; // and how far it is turned round [rad]
        int first = 0;
        for (int piece = 0; piece < 3; ++piece)
        {
            const int count = lengths[piece];
            const double side = -barrier.m_halfWidth + stripe * (first + 0.5 * count);
            const double angle = barrier.m_broken ? turned[piece] : 0.0;
            const Vec3 along = across * std::cos(angle) + f * std::sin(angle);
            const Vec3 square = f * std::cos(angle) - across * std::sin(angle);
            const Vec3 middle = base + across * side
                + (barrier.m_broken ? f * thrown[piece] + up * (halfThickness + 0.04) : up * railHeight);
            for (int k = 0; k < count; ++k)
            {
                const Vec3 center = middle + along * ((k + 0.5 - 0.5 * count) * stripe);
                const Color& color = ((first + k) & 1) != 0 ? white : red;
                if (barrier.m_broken)
                {
                    m_mesh.addBox(center, up, square, along, Vec3(halfThickness, halfHeight, 0.5 * stripe), color);
                }
                else
                {
                    m_mesh.addBox(center, f, up, along, Vec3(halfThickness, halfHeight, 0.5 * stripe), color);
                }
            }
            first += count;
        }

        // The trestles it stands on
        const double legHalfHeight = 0.5 * railHeight;
        int leg = 0;
        for (double side : { -0.94, -0.32, 0.32, 0.94 })
        {
            const Vec3 foot = base + across * (side * barrier.m_halfWidth);
            if (barrier.m_broken)
            {
                m_mesh.addBox(foot + f * (1.4 + 0.5 * leg) + up * 0.09, up, f, across,
                    Vec3(0.05, legHalfHeight, 0.05), steel);
            }
            else
            {
                m_mesh.addBox(foot + f * 0.08 + up * legHalfHeight, f, up, across, Vec3(0.05, legHalfHeight, 0.05), steel);
                m_mesh.addBox(foot + f * 0.08 + up * 0.07, f, up, across, Vec3(0.5, 0.04, 0.07), steel);
            }
            ++leg;
        }
    }

    { // The sign, on the left hand verge
        const double halfWidth = 2.3;
        const double halfHeight = 1.0;
        const double height = 2.5; // of the middle of the sign above the ground [m]
        const double lineHeight = 0.62;
        const Vec3 middle = base - across * (barrier.m_halfWidth + 1.2 + halfWidth) + up * height;
        for (double side : { -1.0, 1.0 })
        {
            m_mesh.addBox(middle + across * (side * (halfWidth - 0.3)) - up * (0.5 * height) + f * 0.12, f, up, across,
                Vec3(0.07, 0.5 * height + halfHeight, 0.07), steel);
        }
        m_mesh.addBox(middle, f, up, across, Vec3(0.05, halfHeight, halfWidth), board);
        addText(middle - f * 0.08 + up * 0.42, across, up, lineHeight, "BRIDGE", lettering);
        addText(middle - f * 0.08 - up * 0.42, across, up, lineHeight, "CLOSED", lettering);
    }
}

// The river: a sheet of tiles at the water level, as far as the bed lies below it. Over the
// banks the water is shallow and pale; in the middle it is deep, and bands of lighter water drift
// downstream. Where it meets the land a line of foam laps in and out.
void Scene::addWater(const Terrain& terrain, double time)
{
    const DirtRoad& bed = terrain.getRiverbed();
    const Color deep(0.12f, 0.35f, 0.60f);
    const Color pale(0.22f, 0.50f, 0.73f);
    const Color shallow(0.36f, 0.66f, 0.72f);
    const Color foam(0.90f, 0.96f, 1.00f);
    const Vec3 upward(0.0, 1.0, 0.0);
    const double level = terrain.getWaterLevel();
    const double wet = terrain.getWaterDepth() / bed.m_depth; // share of a bank or an end that is under water
    const double acrossEdge = bed.m_halfFloor + bed.m_slopeRun * wet;
    const double alongEdge = bed.m_halfLength - bed.m_endRun * (1.0 - wet);
    const double tile = 2.5;
    const auto point = [&](double along, double across, double lift)
    {
        const Vec3 flat = bed.m_centre + bed.m_forward * along + bed.m_cross * across;
        return Vec3(flat.x, level + lift, flat.z);
    };

    // Lines the sheet is cut along: the water's edge, the foot of each bank or end, and tiles between
    const auto cuts = [&](double edge, double foot)
    {
        std::vector<double> lines;
        lines.push_back(-edge);
        const int count = std::max(1, (int) std::lround(2.0 * foot / tile));
        for (int i = 0; i <= count; ++i)
        {
            lines.push_back(-foot + 2.0 * foot * i / count);
        }
        lines.push_back(edge);
        return lines;
    };
    const std::vector<double> along = cuts(alongEdge, alongEdge - bed.m_endRun * wet);
    const std::vector<double> across = cuts(acrossEdge, bed.m_halfFloor);

    for (size_t row = 0; row + 1 < along.size(); ++row)
    {
        for (size_t column = 0; column + 1 < across.size(); ++column)
        {
            const bool edge = row == 0 || row + 2 == along.size() || column == 0 || column + 2 == across.size();
            const float blend = (float) (0.5 + 0.5 * std::sin(0.9 * row + 1.7 * column - 1.4 * time));
            const Color color = edge
                ? shallow.scaled(0.96f + 0.04f * blend)
                : Color(deep.r + (pale.r - deep.r) * blend, deep.g + (pale.g - deep.g) * blend,
                    deep.b + (pale.b - deep.b) * blend);
            m_mesh.addQuad(point(along[row], across[column], 0.0), point(along[row], across[column + 1], 0.0),
                point(along[row + 1], across[column + 1], 0.0), point(along[row + 1], across[column], 0.0),
                color, upward);
        }
    }

    // Foam along both banks, each length of it lapping to its own beat
    const double lift = 0.02;
    const int lengths = (int) std::ceil(2.0 * alongEdge / tile);
    for (int i = 0; i < lengths; ++i)
    {
        const double from = -alongEdge + 2.0 * alongEdge * i / lengths;
        const double to = -alongEdge + 2.0 * alongEdge * (i + 1) / lengths;
        for (double side : { -1.0, 1.0 })
        {
            const double width = 0.26 + 0.20 * std::sin(1.3 * time + 2.3 * i + side);
            m_mesh.addQuad(point(from, side * (acrossEdge - width), lift), point(from, side * (acrossEdge + 0.1), lift),
                point(to, side * (acrossEdge + 0.1), lift), point(to, side * (acrossEdge - width), lift), foam, upward);
        }
    }
}

// What grows and lies along the river: clumps of reeds on the sand above the water's edge, rocks
// in the shallows, and a few logs of the bridge afloat where they have lodged. None of it is
// solid, and all of it keeps clear of where the trucks cross.
void Scene::addRiverside(const Terrain& terrain, double time)
{
    const DirtRoad& bed = terrain.getRiverbed();
    const Vec3 upward(0.0, 1.0, 0.0);
    const double level = terrain.getWaterLevel();
    const double wet = terrain.getWaterDepth() / bed.m_depth;
    const double acrossEdge = bed.m_halfFloor + bed.m_slopeRun * wet;
    const double alongEdge = bed.m_halfLength - bed.m_endRun * (1.0 - wet);
    const double fordAt = dot(terrain.getFord() - bed.m_centre, bed.m_forward);
    const double crossing = 7.5; // half the width kept clear for the trucks [m]
    const auto ground = [&](double along, double across)
    {
        const Vec3 flat = bed.m_centre + bed.m_forward * along + bed.m_cross * across;
        return Vec3(flat.x, bed.height(flat), flat.z);
    };

    { // Reeds
        const Color blade(0.36f, 0.56f, 0.20f);
        const Color dry(0.58f, 0.62f, 0.26f);
        const Color head(0.40f, 0.24f, 0.12f);
        const double spacing = 2.2;
        int seed = 0;
        for (double along = -alongEdge + 2.0; along < alongEdge - 2.0; along += spacing)
        {
            for (double side : { -1.0, 1.0 })
            {
                ++seed;
                if (std::fabs(along - fordAt) < crossing || (seed * 29) % 9 < 4)
                {
                    continue;
                }
                const Vec3 foot = ground(along + 0.6 * ((seed * 37) % 5 - 2) / 2.0,
                    side * (acrossEdge + 0.15 + 0.12 * ((seed * 53) % 5)));
                for (int stalk = 0; stalk < 5; ++stalk)
                {
                    const double height = 0.8 + 0.16 * ((seed * 7 + stalk * 3) % 6);
                    const double sway = 0.05 * std::sin(1.6 * time + seed + stalk);
                    const Vec3 lean(0.10 * (stalk - 2) + sway, 1.0, 0.07 * ((stalk * 5 + seed) % 5 - 2));
                    const Vec3 y = normalize(lean);
                    const Vec3 x = normalize(cross(y, Vec3(0.0, 0.0, 1.0)));
                    const Vec3 z = cross(x, y);
                    const Vec3 root = foot + Vec3(0.16 * (stalk - 2), 0.0, 0.11 * ((stalk * 3) % 3 - 1));
                    m_mesh.addBox(root + y * (0.5 * height), x, y, z, Vec3(0.035, 0.5 * height, 0.035),
                        (stalk + seed) % 3 == 0 ? dry : blade);
                    if ((stalk + seed) % 4 == 0)
                    {
                        m_mesh.addBox(root + y * (height + 0.12), x, y, z, Vec3(0.06, 0.16, 0.06), head);
                    }
                }
            }
        }
    }

    { // Rocks, half in the water
        const Color stone(0.56f, 0.56f, 0.57f);
        const Color mossy(0.47f, 0.50f, 0.44f);
        const std::vector<Vec3> outline = {
            Vec3(-1.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.8, 0.45, 0.0), Vec3(0.15, 0.78, 0.0), Vec3(-0.6, 0.55, 0.0) };
        //                       along the river, across it (share of the way to the edge), size, turned
        const double rocks[][4] = { { -19.0, -0.96, 0.9, 0.4 }, { -11.5,  0.98, 1.2, 2.1 }, {  -3.0, -0.60, 0.7, 1.0 },
                                    {   2.5,  0.20, 1.0, 2.8 }, {   6.0,  0.97, 0.8, 0.2 }, {  -8.0, -1.00, 1.3, 1.6 },
                                    { -16.0,  0.35, 0.8, 0.9 }, { -22.5,  0.90, 1.0, 2.4 }, {  -1.0,  1.00, 0.6, 0.7 } };
        int index = 0;
        for (const double* rock : rocks)
        {
            ++index;
            if (std::fabs(rock[0] - fordAt) < crossing)
            {
                continue;
            }
            const double size = rock[2];
            const Vec3 x(std::cos(rock[3]), 0.0, std::sin(rock[3]));
            const Vec3 z(-x.z, 0.0, x.x);
            std::vector<Vec3> scaled;
            std::transform(outline.begin(), outline.end(), std::back_inserter(scaled),
                [&](const Vec3& corner) { return corner * size; });
            const Vec3 base = ground(rock[0], rock[1] * acrossEdge);
            m_mesh.addPrism(Vec3(base.x, std::max(base.y, level - 0.45 * size), base.z), x, upward, z, scaled,
                0.75 * size, 0.3 * size, index % 3 == 0 ? mossy : stone);
        }
    }

    { // Logs afloat downstream of where the second half of the bridge stood
        const Color bark(0.44f, 0.30f, 0.16f);
        const Color sawn(0.84f, 0.70f, 0.45f);
        const Vec3 right = -bed.m_forward;
        //                      over the river, to the right of the track, length, turned
        const double logs[][4] = { { 3.2, -7.2, 3.4, 0.35 }, { 6.4, -8.6, 2.6, -0.5 }, { 9.3, -6.4, 3.0, 1.2 } };
        int index = 0;
        for (const double* log : logs)
        {
            ++index;
            const double turn = log[3] + 0.06 * std::sin(0.5 * time + index);
            const Vec3 axis = right * std::cos(turn) + bed.m_cross * std::sin(turn);
            const Vec3 middle = terrain.getFord() + bed.m_cross * log[0] + right * log[1]
                + upward * (level - 0.04 + 0.05 * std::sin(1.7 * time + 2.0 * index));
            m_mesh.addCylinder(middle - axis * (0.5 * log[2]), middle + axis * (0.5 * log[2]), 0.25, bark, sawn);
        }
    }
}

// Spray thrown up behind each wheel that is moving through the water
void Scene::addSpray(const Truck& truck, const Terrain& terrain, double time)
{
    const Color spray(0.86f, 0.93f, 1.00f);
    const Vec3 upward(0.0, 1.0, 0.0);
    const Vec3 flat(truck.getVelocity().x, 0.0, truck.getVelocity().z);
    const double speed = length(flat);
    const double least = 1.5; // slower than this the water stays calm [m/s]
    if (speed < least)
    {
        return;
    }

    const Vec3 back = flat * (-1.0 / speed);
    const Vec3 across(-back.z, 0.0, back.x);
    const double size = std::min(1.0, speed / 12.0);
    int index = 0;
    for (const Wheel& wheel : truck.getWheels())
    {
        ++index;
        const Vec3 foot = wheel.m_center - upward * truck.getWheelRadius();
        if (!terrain.isWading(foot))
        {
            continue;
        }
        const Vec3 base(wheel.m_center.x, terrain.getWaterLevel(), wheel.m_center.z);
        for (int fin = -1; fin <= 1; ++fin)
        {
            const double flutter = 0.7 + 0.3 * std::sin(23.0 * time + 2.1 * fin + 1.3 * index);
            const Vec3 root = base + across * (0.45 * fin) + back * 0.3;
            const Vec3 tip = root + back * (1.1 * size) + across * (0.5 * fin * size)
                + upward * ((0.5 + 1.1 * size) * flutter);
            m_mesh.addTriangle(root - across * 0.28, root + across * 0.28, tip, spray, upward);
        }
    }
}

void Scene::addScenery()
{
    const Vec3 x(1.0, 0.0, 0.0);
    const Vec3 y(0.0, 1.0, 0.0);
    const Vec3 z(0.0, 0.0, 1.0);

    { // Trees
        const Color trunk(0.45f, 0.30f, 0.15f);
        const Color leaves(0.12f, 0.42f, 0.16f);
        for (const Vec3& tree : m_trees)
        {
            const Vec3 foot(tree.x, 0.0, tree.z);
            const double size = tree.y;
            m_mesh.addBox(foot + y * (1.0 * size), x, y, z, Vec3(0.3, 1.0 * size, 0.3), trunk);
            m_mesh.addPyramid(foot + y * (1.6 * size), 1.8 * size, 5.0 * size, leaves);
        }
    }

    { // Mountains on the horizon
        const Color rock(0.50f, 0.52f, 0.62f);
        const Color snow(0.90f, 0.92f, 0.96f);
        const int count = 9;
        for (int i = 0; i < count; ++i)
        {
            const double angle = 2.0 * M_PI * i / (double) count + 0.3;
            const double radius = 2200.0 + 250.0 * (i % 3);
            const double height = 420.0 + 90.0 * ((i * 4) % 5);
            const Vec3 foot(100.0 + radius * std::cos(angle), -1.0, radius * std::sin(angle));
            m_mesh.addPyramid(foot, 1.1 * height, height, rock);
            m_mesh.addPyramid(foot + y * (0.72 * height), 0.31 * height, 0.29 * height, snow);
        }
    }
}

void Scene::addTruck(const Truck& truck, const Color& paint)
{
    const Vec3 f = truck.forward();
    const Vec3 u = truck.up();
    const Vec3 r = truck.right();
    const Color trim(0.95f, 0.80f, 0.15f);
    const Color glass(0.60f, 0.82f, 0.95f);
    const Color steel(0.22f, 0.22f, 0.25f);

    const Color lamp(1.00f, 0.95f, 0.70f);
    const Color tail(0.80f, 0.10f, 0.08f);
    const Vec3 origin = truck.toWorld(Vec3(0.0, 0.0, 0.0));
    const double bodyHalfWidth = 1.2;

    { // Body: every part is an outline seen from the side, drawn out across the truck with its
      // edges cut back, so that nose, tail, windscreen and roof are sloped instead of square
        const std::vector<Vec3> hull = {
            Vec3(-2.20, -0.10, 0.0), Vec3( 2.15, -0.10, 0.0), Vec3( 2.30,  0.04, 0.0), Vec3( 2.30,  0.24, 0.0),
            Vec3( 2.24,  0.40, 0.0), Vec3( 2.05,  0.52, 0.0), Vec3( 1.20,  0.62, 0.0), Vec3(-2.22,  0.62, 0.0),
            Vec3(-2.30,  0.52, 0.0), Vec3(-2.30,  0.02, 0.0) };
        const std::vector<Vec3> doors = {
            Vec3(-0.74,  0.60, 0.0), Vec3( 1.30,  0.60, 0.0), Vec3( 1.14,  0.82, 0.0), Vec3(-0.69,  0.82, 0.0) };
        const std::vector<Vec3> windows = {
            Vec3(-0.68,  0.80, 0.0), Vec3( 1.14,  0.80, 0.0), Vec3( 0.84,  1.17, 0.0), Vec3(-0.57,  1.17, 0.0) };
        const std::vector<Vec3> roof = {
            Vec3(-0.60,  1.15, 0.0), Vec3( 0.89,  1.15, 0.0), Vec3( 0.81,  1.26, 0.0), Vec3( 0.66,  1.31, 0.0),
            Vec3(-0.42,  1.31, 0.0), Vec3(-0.56,  1.26, 0.0) };
        m_mesh.addPrism(origin, f, u, r, hull, bodyHalfWidth, 0.14, paint);
        m_mesh.addPrism(origin, f, u, r, doors, 1.12, 0.10, paint);
        m_mesh.addPrism(origin, f, u, r, windows, 1.04, 0.12, glass);
        m_mesh.addPrism(origin, f, u, r, roof, 1.00, 0.14, paint);

        m_mesh.addBox(truck.toWorld(Vec3( 2.27, -0.02, 0.0)), f, u, r, Vec3(0.08, 0.07, 1.10), trim);  // bumper
        m_mesh.addBox(truck.toWorld(Vec3(-1.50,  0.64, 0.0)), f, u, r, Vec3(0.70, 0.04, 0.98), steel); // load bed
        for (double side : { -1.0, 1.0 })
        {
            m_mesh.addBox(truck.toWorld(Vec3( 2.30, 0.16, side * 0.80)), f, u, r, Vec3(0.03, 0.07, 0.20), lamp);
            m_mesh.addBox(truck.toWorld(Vec3(-2.30, 0.30, side * 0.88)), f, u, r, Vec3(0.03, 0.09, 0.13), tail);
        }
    }

    { // Axles and wheels
        const std::vector<Wheel>& wheels = truck.getWheels();
        for (size_t i = 0; i + 1 < wheels.size(); i += 2)
        {
            const Vec3 middle = (wheels[i].m_center + wheels[i + 1].m_center) * 0.5;
            const Vec3 span = wheels[i + 1].m_center - wheels[i].m_center;
            const Vec3 along = normalize(span, r);
            const Vec3 axleUp = normalize(u - along * dot(u, along), u);
            m_mesh.addBox(middle, cross(axleUp, along), axleUp, along, Vec3(0.14, 0.14, 0.5 * length(span)), steel);
            m_mesh.addBox((middle + truck.toWorld(Vec3(wheels[i].m_attach.x, 0.0, 0.0))) * 0.5, f, u, r,
                Vec3(0.08, 0.5 * length(middle - truck.toWorld(Vec3(wheels[i].m_attach.x, 0.0, 0.0))), 0.08), trim);
        }
        for (const Wheel& wheel : wheels)
        {
            addWheel(wheel.m_center, wheel.m_axle, u, truck.getWheelRadius(), truck.getWheelHalfWidth(), wheel.m_spin);
        }
    }
}

void Scene::addWheel(const Vec3& center, const Vec3& axle, const Vec3& up, double radius, double halfWidth, double spin)
{
    const Color treadA(0.13f, 0.13f, 0.14f);
    const Color treadB(0.20f, 0.20f, 0.21f);
    const Color wall(0.16f, 0.16f, 0.17f);
    const Color hubA(0.95f, 0.80f, 0.15f);
    const Color hubB(0.80f, 0.80f, 0.82f);
    const double hubRadius = 0.5 * radius;

    const Vec3 v = normalize(up - axle * dot(up, axle), Vec3(0.0, 1.0, 0.0));
    const Vec3 h = cross(v, axle);
    const auto spoke = [&](int i)
    {
        const double angle = -spin + 2.0 * M_PI * i / (double) m_wheelSides;
        return h * std::cos(angle) + v * std::sin(angle);
    };

    for (int i = 0; i < m_wheelSides; ++i)
    {
        const Vec3 s0 = spoke(i);
        const Vec3 s1 = spoke(i + 1);
        const Vec3 mid = normalize(s0 + s1);
        const Vec3 near = axle * (-halfWidth);
        const Vec3 far = axle * halfWidth;

        m_mesh.addQuad(center + s0*radius + near, center + s1*radius + near,
            center + s1*radius + far, center + s0*radius + far, (i & 1) != 0 ? treadA : treadB, mid);

        for (int side = 0; side < 2; ++side)
        {
            const Vec3 offset = side == 0 ? near : far;
            const Vec3 normal = side == 0 ? -axle : axle;
            m_mesh.addQuad(center + s0*radius + offset, center + s1*radius + offset,
                center + s1*hubRadius + offset, center + s0*hubRadius + offset, wall, normal);
            m_mesh.addTriangle(center + offset, center + s0*hubRadius + offset, center + s1*hubRadius + offset,
                (i & 1) != 0 ? hubA : hubB, normal);
        }
    }
}
