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
    // row the highest of the five bits is the leftmost pixel. Only the characters the gates use
    // are carried.
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
        { 'F', { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 } },
        { 'H', { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 } },
        { 'I', { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F } },
        { 'N', { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 } },
        { 'R', { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 } },
        { 'S', { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E } },
        { 'T', { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 } }
    };

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
{
}

void Scene::build(const Game& game, const Vec3& eye)
{
    m_mesh.clear();
    addGround(eye);
    addFaces(game.getTerrain());
    addMarkings(game.getTerrain());
    addCheckpoints(game.getTerrain().getTrack());
    for (const Wreck& wreck : game.getTerrain().getWrecks())
    {
        addWreck(wreck);
    }
    plantTrees(game.getTerrain().getTrack());
    addScenery();
    for (const Racer& racer : game.getRacers())
    {
        addTruck(racer.m_truck, racer.m_color);
    }
}

void Scene::addGround(const Vec3& eye)
{
    const Vec3 upward(0.0, 1.0, 0.0);
    const Color light(0.30f, 0.62f, 0.26f);
    const Color dark(0.26f, 0.56f, 0.23f);
    const Color distant(0.28f, 0.59f, 0.245f);

    { // Far field, slightly below the tiles
        const double reach = 5000.0;
        const double y = -0.5;
        m_mesh.addQuad(Vec3(eye.x - reach, y, eye.z - reach), Vec3(eye.x + reach, y, eye.z - reach),
            Vec3(eye.x + reach, y, eye.z + reach), Vec3(eye.x - reach, y, eye.z + reach), distant, upward);
    }

    { // Checkered tiles around the camera give a sense of speed
        const long cx = std::lround(std::floor(eye.x / m_tileSize));
        const long cz = std::lround(std::floor(eye.z / m_tileSize));
        for (long ix = cx - m_tileRange; ix <= cx + m_tileRange; ++ix)
        {
            for (long iz = cz - m_tileRange; iz <= cz + m_tileRange; ++iz)
            {
                const double x = ix * m_tileSize;
                const double z = iz * m_tileSize;
                m_mesh.addQuad(Vec3(x, 0.0, z), Vec3(x + m_tileSize, 0.0, z),
                    Vec3(x + m_tileSize, 0.0, z + m_tileSize), Vec3(x, 0.0, z + m_tileSize),
                    ((ix + iz) & 1) != 0 ? light : dark, upward);
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
        default:
            require(false, "Scene::addFaces: unknown surface");
        }
        const Vec3 offset = face.m_normal * lift;
        m_mesh.addQuad(face.m_a + offset, face.m_b + offset, face.m_c + offset, face.m_d + offset, color, face.m_normal);
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
            || terrain.getDirtRoad().contains(p.m_position);
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

// Trees on a jittered grid, kept clear of the road
void Scene::plantTrees(const Track& track)
{
    if (m_planted)
    {
        return;
    }
    m_planted = true;

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
            if (clear && (seed * 29) % 5 != 0)
            {
                m_trees.push_back(Vec3(foot.x, 1.0 + 0.1 * ((seed * 5) % 7), foot.z));
            }
        }
    }
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

    { // Body
        m_mesh.addBox(truck.toWorld(Vec3( 0.0, 0.25, 0.0)), f, u, r, Vec3(2.3, 0.35, 1.2), paint);
        m_mesh.addBox(truck.toWorld(Vec3( 0.2, 0.88, 0.0)), f, u, r, Vec3(0.9, 0.28, 1.0), glass);
        m_mesh.addBox(truck.toWorld(Vec3( 0.2, 1.23, 0.0)), f, u, r, Vec3(0.95, 0.07, 1.05), paint);
        m_mesh.addBox(truck.toWorld(Vec3( 2.32, 0.25, 0.0)), f, u, r, Vec3(0.03, 0.12, 1.0), trim);
        m_mesh.addBox(truck.toWorld(Vec3(-1.5, 0.66, 0.0)), f, u, r, Vec3(0.75, 0.06, 1.15), steel);
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
