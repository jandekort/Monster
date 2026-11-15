// Monster - native tests for terrain, truck physics and scoring
// Author: Jan de Kort
// Created: 2026-11-14
#include "Game.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
    int g_failures = 0;

    void check(bool condition, const std::string& what)
    {
        std::cout << (condition ? "  ok    " : "  FAIL  ") << what << std::endl;
        if (!condition)
        {
            ++g_failures;
        }
    }

    void run(Game& game, double seconds, const Controls& controls)
    {
        const double frame = 1.0 / 60.0;
        game.setControls(controls);
        for (double t = 0.0; t < seconds; t += frame)
        {
            game.step(frame);
        }
    }

    // Struct Pilot: drives the player's truck round the circuit the way the opponents drive theirs
    struct Pilot
    {
        Controls drive(const Game& game) const
        {
            return Driver().drive(game.getTruck(), game.getProgress(), game.getTerrain().getTrack());
        }
    };

    Controls drive(double throttle, double brake, double steer)
    {
        Controls controls;
        controls.m_throttle = throttle;
        controls.m_brake = brake;
        controls.m_steer = steer;
        return controls;
    }
}

int main()
{
    std::cout << "Track" << std::endl;
    {
        Track track;
        const std::vector<PathPoint>& points = track.getPoints();
        const auto count = [&](PathPoint::Kind kind)
        {
            return std::count_if(points.begin(), points.end(),
                [&](const PathPoint& point) { return point.m_kind == kind; });
        };
        const auto highest = std::max_element(points.begin(), points.end(),
            [](const PathPoint& a, const PathPoint& b) { return a.m_position.y < b.m_position.y; });
        bool orthonormal = true;
        for (const PathPoint& point : points)
        {
            orthonormal = orthonormal
                && std::fabs(length(point.m_normal) - 1.0) < 1.0e-9
                && std::fabs(dot(point.m_normal, point.m_forward)) < 1.0e-3
                && std::fabs(dot(point.m_normal, point.m_cross)) < 1.0e-3;
        }
        std::cout << "    " << points.size() << " points, " << track.getLength() << " m" << std::endl;
        check(length(points.back().m_position - points.front().m_position) < 1.0e-6, "circuit closes");
        check(track.getLength() > 800.0, "lap is long enough");
        check(count(PathPoint::Kind::Loop) > 0 && count(PathPoint::Kind::Bridge) > 0, "has a loop and a bridge");
        check(std::fabs(highest->m_position.y - 20.0) < 1.0e-6 && highest->m_normal.y < -0.99, "loop top is upside down");
        check(orthonormal, "road frames are orthonormal");
    }

    std::cout << "Checkpoints" << std::endl;
    {
        Track track;
        const std::vector<PathPoint>& points = track.getPoints();
        const std::vector<size_t>& checkpoints = track.getCheckpoints();
        bool ordered = true;
        bool onGoodRoad = true;
        for (size_t i = 0; i < checkpoints.size(); ++i)
        {
            const PathPoint& point = points[checkpoints[i]];
            ordered = ordered && (i == 0 || checkpoints[i] > checkpoints[i - 1]);
            onGoodRoad = onGoodRoad && point.m_kind == PathPoint::Kind::Flat && !point.m_corner;
            std::cout << "    gate " << i + 1 << " at " << point.m_distance << " m" << std::endl;
        }
        check(checkpoints.size() == 4, "there are four numbered checkpoints");
        check(ordered, "they are in driving order");
        check(onGoodRoad, "every gate stands on level, straight road");
        check(track.getStartFinish().m_distance == 0.0, "the start and finish line opens the lap");
        check(points[checkpoints.front()].m_distance > 0.1 * track.getLength(),
            "the first gate is well clear of the line");
        check(points[checkpoints.back()].m_distance < track.getLength(), "the last is before the line");
        check(track.getGateHalfWidth() > track.getHalfWidth(), "the gate is wider than the road");
    }

    std::cout << "Terrain" << std::endl;
    {
        Terrain terrain;
        const Vec3 down(0.0, -1.0, 0.0);
        const auto heightAt = [&](double x, double z, double from)
        {
            Hit hit;
            return terrain.raycast(Vec3(x, from, z), down, 100.0, hit) ? hit.m_point.y : -1.0;
        };
        Hit hit;
        check(std::fabs(heightAt(0.0, 0.0, 5.0)) < 1.0e-9, "flat at the start");
        check(std::fabs(heightAt(78.5, 0.0, 20.0) - 1.95) < 1.0e-9, "jump ramp is linear");
        check(heightAt(78.5, 6.0, 20.0) == 0.0, "jump ramp has a finite width");
        check(heightAt(92.5, 0.0, 20.0) < -1.0, "the dirt road is sunk between the ramps");
        check(terrain.raycast(Vec3(78.5, 10.0, 0.0), down, 100.0, hit) && hit.m_normal.x < -0.2, "ramp normal leans back");
        const Wreck& wreck = terrain.getWrecks().front();
        const Vec3 above = wreck.getPosition() + wreck.getUp() * 5.0;
        check(terrain.raycast(above, -wreck.getUp(), 100.0, hit) && hit.m_wreck == 0
            && dot(hit.m_point - wreck.getPosition(), wreck.getUp()) > 1.0, "a wreck is in the way");
        check(std::fabs(heightAt(62.0, -6.0, 20.0) - terrain.getTrack().getBridgeHeight()) < 1.0e-6,
            "bridge deck beside the crossing");
        check(heightAt(62.0, 0.0, 3.0) == 0.0, "road passes under the bridge");
        check(terrain.raycast(Vec3(40.0, 1.5, 30.0), Vec3(1.0, 0.0, 0.0), 40.0, hit) && hit.m_normal.x < -0.99, "embankment has walls");
        check(!terrain.raycast(Vec3(40.0, 1.5, 0.0), Vec3(1.0, 0.0, 0.0), 25.0, hit), "tunnel is open");
        check(!terrain.raycast(Vec3(0.0, 1.0, 0.0), Vec3(0.0, 1.0, 0.0), 100.0, hit), "nothing above the start");

        bool thrown = false;
        try
        {
            Wreck sample(terrain.getTrack().getStartFinish());
            sample.applyLoad(1.0, -1.0);
        }
        catch (const std::logic_error&)
        {
            thrown = true;
        }
        check(thrown, "negative time step is rejected");
    }

    std::cout << "Truck at rest" << std::endl;
    {
        Game game(0);
        run(game, 4.0, drive(0.0, 0.0, 0.0));
        const Truck& truck = game.getTruck();
        std::cout << "    height " << truck.getPosition().y << ", speed " << truck.getSpeed() << std::endl;
        check(truck.getWheelsOnGround() == 4, "four wheels on the ground");
        check(truck.getSpeed() < 0.05, "settles");
        check(truck.getPosition().y > 1.2 && truck.getPosition().y < 1.7, "rides on its springs");
        check(truck.up().y > 0.999, "stays level");
        check(game.getScore() == 0, "no points for standing still");
    }

    std::cout << "Steering" << std::endl;
    {
        Game game(0);
        run(game, 1.5, drive(1.0, 0.0, 0.0));
        run(game, 2.0, drive(0.4, 0.0, 1.0));
        const Truck& truck = game.getTruck();
        std::cout << "    z " << truck.getPosition().z << ", up.y " << truck.up().y << std::endl;
        check(truck.getPosition().z > 2.0, "steering right moves to +z");
        check(truck.forward().z > 0.2, "heading turns right");
        check(truck.up().y > 0.9, "does not roll over in a turn");
    }

    std::cout << "Reverse" << std::endl;
    {
        Game game(0);
        const double from = game.getTruck().getPosition().x;
        run(game, 3.0, drive(0.0, 1.0, 0.0));
        check(game.getTruck().getPosition().x < from - 3.0, "brake reverses from standstill");
    }

    std::cout << "Jump" << std::endl;
    {
        Game game(0);
        bool flew = false;
        double peak = 0.0;
        const double frame = 1.0 / 60.0;
        game.setControls(drive(1.0, 0.0, 0.0));
        for (double t = 0.0; t < 14.0 && game.getTruck().getPosition().x < 175.0; t += frame)
        {
            game.step(frame);
            flew = flew || game.getJudge().isAirborne();
            peak = std::max(peak, game.getTruck().getPosition().y);
        }
        const Truck& truck = game.getTruck();
        std::cout << "    x " << truck.getPosition().x << ", z " << truck.getPosition().z
            << ", peak " << peak << ", up.y " << truck.up().y << std::endl;
        std::cout << "    hud " << game.hudString() << std::endl;
        check(flew, "truck leaves the ramp");
        check(peak > 4.5, "jump gains height");
        check(truck.getPosition().x > 140.0, "clears the gap");
        check(std::fabs(truck.getPosition().z) < 2.0, "tracks straight");
        check(game.getJudge().getLastRating() != Judge::Rating::None, "landing is rated");
        check(truck.up().y > 0.8 && !game.isWrecked(), "lands on its wheels");

        const int score = game.getScore();
        game.reset();
        check(game.getScore() == 0 && game.getBestScore() == score, "reset keeps the best score");
    }

    std::cout << "The row of wrecks" << std::endl;
    {
        Game game(0);
        const Terrain& terrain = game.getTerrain();
        const std::vector<Wreck>& wrecks = terrain.getWrecks();
        const Track& track = terrain.getTrack();
        const double frame = 1.0 / 60.0;

        bool afterTheLastGate = true;
        bool acrossTheRoad = true;
        bool inOrder = true;
        for (size_t i = 0; i < wrecks.size(); ++i)
        {
            const PathPoint& gate = track.getPoints()[track.getCheckpoints().back()];
            const Vec3 fromGate = wrecks[i].getPosition() - gate.m_position;
            afterTheLastGate = afterTheLastGate && dot(fromGate, gate.m_forward) > 0.0;
            acrossTheRoad = acrossTheRoad
                && wrecks[i].getHalfLength() > wrecks[i].getHalfWidth()
                && length(wrecks[i].getPosition() - track.getStartFinish().m_position) > 20.0;
            if (i > 0)
            {
                const Vec3 step = wrecks[i].getPosition() - wrecks[i - 1].getPosition();
                inOrder = inOrder && dot(step, wrecks[i].getForward()) > 4.0 && length(step) < 12.0;
            }
        }
        check(wrecks.size() == 3, "there are three wrecks");
        check(afterTheLastGate, "they lie past the last gate, on the run back to the line");
        check(acrossTheRoad, "across the road and clear of the start line");
        check(inOrder, "one behind the next, in a row along the road");
        check(terrain.getCrush() == 0.0, "all of them start intact");

        // Drive the row down from the last gate
        game.startAt(track.getCheckpoints().back());
        game.setControls(drive(1.0, 0.0, 0.0));
        for (double t = 0.0; t < 14.0; t += frame)
        {
            game.step(frame);
        }
        const Truck& truck = game.getTruck();
        std::cout << "    crush " << terrain.getCrush() << ", points " << game.getCrushPoints()
            << ", up.y " << truck.up().y << ", height " << truck.getPosition().y << std::endl;
        check(terrain.getCrush() > 0.9, "driving the row flattens all of them");
        check(game.getCrushPoints() >= 498 && game.getCrushPoints() <= 502, "and scores once for the row");
        check(truck.up().y > 0.8 && !game.isWrecked(), "the truck rides over them and stays upright");
        check(dot(truck.getPosition() - wrecks.back().getPosition(), wrecks.back().getForward()) > 0.0,
            "and comes out the far side");

        game.reset();
        check(terrain.getCrush() == 0.0, "reset restores the row");
    }

    std::cout << "Cornering" << std::endl;
    {
        // On empty ground well clear of the circuit: the turn sweeps a wide circle, and the
        // wreck or a ramp caught mid-corner would decide the outcome instead of the tyres
        Terrain terrain;
        Truck truck;
        truck.placeAt(Vec3(0.0, 1.64, 200.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
        const double step = 1.0 / 240.0;
        const double target = 18.0;
        for (double t = 0.0; t < 20.0 && truck.getSpeed() < target; t += step)
        {
            truck.step(step, drive(1.0, 0.0, 0.0), terrain);
        }

        const Vec3 entry = truck.forward();
        double slip = 0.0;
        double minUp = 1.0;
        double yaw = 0.0;
        int samples = 0;
        for (double t = 0.0; t < 4.0; t += step)
        {
            truck.step(step, drive(truck.getSpeed() < target ? 1.0 : 0.0, 0.0, 1.0), terrain);
            minUp = std::min(minUp, truck.up().y);
            if (t > 1.0)
            {
                slip = std::max(slip, std::fabs(dot(truck.getVelocity(), truck.right())));
                yaw += std::fabs(dot(truck.getAngularVelocity(), truck.up()));
                ++samples;
            }
        }
        const double radius = truck.getSpeed() * samples / yaw;
        std::cout << "    sideslip " << slip << " m/s, radius " << radius
            << " m, min up.y " << minUp << std::endl;
        check(slip < 0.8, "tyres hold instead of sliding sideways");
        check(radius < 65.0, "the turn is tight (grass is slower now, so it is wider than on tarmac)");
        check(minUp > 0.9, "a hard corner does not tip the truck");
        check(dot(truck.forward(), entry) < 0.5, "the truck turns well past a right angle");
    }

    std::cout << "Broken bridge and dirt road" << std::endl;
    {
        Terrain terrain;
        const Track& track = terrain.getTrack();
        const DirtRoad& dirt = terrain.getDirtRoad();
        const PathPoint& gap = track.getGapCentre();
        const Vec3 down(0.0, -1.0, 0.0);
        const Vec3 high(0.0, 2.0, 0.0);
        Hit hit;

        // The deck is missing over the tunnel: a ray dropped there falls through to the road below
        check(terrain.raycast(gap.m_position + high, down, 50.0, hit) && hit.m_point.y == 0.0,
            "the bridge has a hole in it above the tunnel");
        const Vec3 beside = gap.m_position + gap.m_forward * (track.getBridgeGap() + 3.0) + high;
        check(terrain.raycast(beside, down, 50.0, hit)
            && std::fabs(hit.m_point.y - track.getBridgeHeight()) < 1.0e-6, "the deck carries on past it");

        // The dirt road runs alongside, to the right as the bridge is driven, and is sunk
        check(dot(dirt.m_centre - gap.m_position, gap.m_cross) > 0.0, "the dirt road lies to its right");
        check(std::fabs(dot(dirt.m_forward, gap.m_forward)) > 0.99, "running alongside it");
        check(terrain.raycast(dirt.m_centre + Vec3(0.0, 5.0, 0.0), down, 50.0, hit) && hit.m_point.y < -1.0,
            "sunk into the ground");

        bool pale = false;
        bool dark = false;
        double tile = 0.0;
        for (const Face& face : terrain.getFaces())
        {
            pale = pale || face.m_surface == Face::Surface::DirtPale;
            dark = dark || face.m_surface == Face::Surface::DirtDark;
            if (face.m_surface == Face::Surface::DirtPale || face.m_surface == Face::Surface::DirtDark)
            {
                tile = std::max(tile, length(face.m_b - face.m_a));
            }
        }
        std::cout << "    dirt tiles of " << tile << " m, floor at " << dirt.height(dirt.m_centre)
            << " m" << std::endl;
        check(pale && dark, "tiled in two shades");
        check(tile > 0.0 && tile < 10.0, "in tiles smaller than the grass");

        // The going: tarmac, then dirt, then grass
        const double onRoad = Face::traction(Face::Surface::Road);
        const double onDirt = Face::traction(Face::Surface::DirtPale);
        check(terrain.raycast(dirt.m_centre + Vec3(0.0, 5.0, 0.0), down, 50.0, hit)
            && std::fabs(hit.m_traction - onDirt) < 1.0e-9, "a ray onto the dirt road reports its going");
        check(terrain.raycast(Vec3(0.0, 5.0, 200.0), down, 50.0, hit) && hit.m_traction < onDirt && onDirt < onRoad,
            "the road is the fastest going, then the dirt road, then the grass");

        // and it shows in what the truck can do from rest
        const auto after = [&](const Vec3& from, const Vec3& forward)
        {
            Terrain ground;
            Truck truck;
            truck.placeAt(from, forward, Vec3(0.0, 1.0, 0.0));
            for (double t = 0.0; t < 3.0; t += 1.0 / 240.0)
            {
                truck.step(1.0 / 240.0, drive(1.0, 0.0, 0.0), ground);
            }
            return truck.getSpeed();
        };
        const Truck standing;
        const double onTheDirt = after(dirt.m_centre - dirt.m_forward * 15.0
            + Vec3(0.0, dirt.height(dirt.m_centre) + standing.getRideHeight(), 0.0), dirt.m_forward);
        const double onTheGrass = after(Vec3(0.0, standing.getRideHeight(), 200.0), Vec3(1.0, 0.0, 0.0));
        std::cout << "    after three seconds: dirt " << onTheDirt << " m/s, grass " << onTheGrass
            << " m/s" << std::endl;
        check(onTheDirt > onTheGrass + 0.5, "the truck gets along faster on the dirt road than on the grass");
    }

    std::cout << "Dirt road round the bridge" << std::endl;
    {
        Terrain terrain;
        const Track& track = terrain.getTrack();
        const DirtRoad& dirt = terrain.getDirtRoad();
        const std::vector<Vec3>& path = terrain.getDirtPath();
        const std::vector<PathPoint>& points = track.getPoints();
        const double onDirt = Face::traction(Face::Surface::DirtPale);
        const Vec3 down(0.0, -1.0, 0.0);

        // Dirt under every step of the way, from one edge of the main road to the other
        bool unbroken = true;
        double longestStep = 0.0;
        for (size_t i = 0; i < path.size(); ++i)
        {
            Hit hit;
            // The first and last few steps lie on the main road itself, where the slip roads branch off
            const bool junction = i < 12 || i + 12 >= path.size();
            unbroken = unbroken && terrain.raycast(path[i] + Vec3(0.0, 4.0, 0.0), down, 20.0, hit)
                && (std::fabs(hit.m_traction - onDirt) < 1.0e-9 || (junction && hit.m_traction > onDirt));
            if (i > 0)
            {
                longestStep = std::max(longestStep, length(path[i] - path[i - 1]));
            }
        }
        check(path.size() > 40 && unbroken, "the dirt road is dirt all the way along");
        check(longestStep < 4.0, "in one piece, with no jumps in it");

        // It starts and ends beside the main road, either side of the bridge, without lying on it
        const auto nearestRoad = [&](const Vec3& point)
        {
            double nearest = 1.0e9;
            for (const PathPoint& road : points)
            {
                if (road.m_kind == PathPoint::Kind::Flat && !dirt.contains(road.m_position))
                {
                    const double dx = road.m_position.x - point.x;
                    const double dz = road.m_position.z - point.z;
                    nearest = std::min(nearest, std::sqrt(dx*dx + dz*dz));
                }
            }
            return nearest;
        };
        double closest = 1.0e9;
        double drawnClosest = 1.0e9;
        for (const Face& face : terrain.getFaces())
        {
            if (face.m_surface == Face::Surface::DirtPale || face.m_surface == Face::Surface::DirtDark)
            {
                // The cutting itself passes under the jump, where the main road is cut on purpose:
                // it is the slip roads outside it that must keep off the tarmac
                for (const Vec3& vertex : { face.m_a, face.m_b, face.m_c, face.m_d })
                {
                    if (!dirt.contains(vertex))
                    {
                        // Tiles drawn under the tarmac at a junction close the seam but are not driven on
                        drawnClosest = std::min(drawnClosest, nearestRoad(vertex));
                        closest = std::min(closest, face.m_solid ? nearestRoad(vertex) : 1.0e9);
                    }
                }
            }
        }
        const PathPoint& gap = track.getGapCentre();
        std::cout << "    " << path.size() << " steps, nearest the dirt comes to the middle of the main road "
            << closest << " m (the road is " << track.getHalfWidth() << " m to its edge)" << std::endl;
        check(closest > track.getHalfWidth() - 3.0, "the dirt driven on stays off the main road");
        check(drawnClosest < track.getHalfWidth() - 1.0, "and the slip roads reach in under its edge, leaving no seam");
        check(closest < track.getHalfWidth() + 0.1, "and meets its edge with no grass between");
        check(dot(path.front() - gap.m_position, gap.m_forward) < -40.0
            && dot(path.back() - gap.m_position, gap.m_forward) > 40.0,
            "leaving it before the bridge and rejoining after");

        // and it enters the cutting through its ends, not over its sides
        const Vec3 mouth = dirt.m_centre - dirt.m_forward * dirt.m_halfLength;
        bool throughTheEnd = false;
        for (const Vec3& point : path)
        {
            throughTheEnd = throughTheEnd || length(Vec3(point.x - mouth.x, 0.0, point.z - mouth.z)) < 1.5;
        }
        check(throughTheEnd, "it runs into the end of the cutting, down its entrance ramp");
    }

    std::cout << "Jumping the hole in the bridge" << std::endl;
    {
        const Track track;
        const std::vector<PathPoint>& points = track.getPoints();
        const size_t hole = (size_t) (&track.getGapCentre() - &points.front());
        size_t beyond = hole;
        while (points[beyond].m_kind == PathPoint::Kind::Gap)
        {
            ++beyond;
        }
        const size_t crest = (size_t) (&track.getBridgeCrest() - &points.front());
        std::cout << "    lip at " << points[hole - 1].m_position.y << " m, deck beyond at "
            << points[beyond].m_position.y << " m, " << length(points[beyond].m_position - points[hole - 1].m_position)
            << " m apart" << std::endl;
        check(hole < crest, "the hole comes before the middle of the bridge");
        check(points[hole - 1].m_position.y > points[beyond].m_position.y + 0.5, "the take-off lip stands above the deck beyond");
        check(length(points[beyond].m_position - points[hole - 1].m_position) > 7.0, "and the hole is wider than it was");

        // Driven at it with more or less throttle: speed decides
        const auto attempt = [&](double throttle)
        {
            Game game(0);
            Pilot pilot;
            game.startAt(hole - 45);
            double lowest = 99.0;
            double speed = 0.0;
            for (double t = 0.0; t < 30.0 && game.getProgress() < beyond + 6 && game.getProgress() + 50 > hole; t += 1.0 / 60.0)
            {
                Controls controls = pilot.drive(game);
                controls.m_throttle = std::min(controls.m_throttle, throttle);
                game.setControls(controls);
                game.step(1.0 / 60.0);
                if (game.getProgress() + 1 >= hole)
                {
                    lowest = std::min(lowest, game.getTruck().getPosition().y);
                    speed = speed == 0.0 ? game.getTruck().getSpeed() : speed;
                }
            }
            std::cout << "    throttle " << throttle << ": " << speed * 3.6 << " km/h at the lip, lowest "
                << lowest << " m" << std::endl;
            return lowest;
        };
        check(attempt(1.0) > 5.0, "at full throttle the truck lands on the far deck");
        check(attempt(0.25) < 3.0, "too slow, and it drops through the hole");
    }

    std::cout << "Round the bridge by the dirt road" << std::endl;
    {
        Game game(0);
        const Terrain& terrain = game.getTerrain();
        const Driver driver;
        game.startAt(terrain.getSlipLeave() - 2);
        double lowest = 9.0;
        for (double t = 0.0; t < 40.0 && game.getProgress() < terrain.getSlipRejoin(); t += 1.0 / 60.0)
        {
            game.setControls(driver.follow(game.getTruck(), terrain.getDirtPath()));
            game.step(1.0 / 60.0);
            lowest = std::min(lowest, game.getTruck().getPosition().y);
        }
        std::cout << "    progress " << game.getProgress() << " (rejoins at " << terrain.getSlipRejoin()
            << "), lowest " << lowest << " m, lap time so far " << game.getLapTime() << " s" << std::endl;
        check(lowest < 0.0, "the dirt road leads down through the cutting");
        check(game.getProgress() >= terrain.getSlipRejoin(), "and coming out the far end counts as having passed the bridge");
        check(game.getTruck().up().y > 0.9 && !game.isWrecked(), "with the truck still on its wheels");
    }

    std::cout << "Bridge closed" << std::endl;
    {
        Game game(0);
        const Terrain& terrain = game.getTerrain();
        const std::vector<PathPoint>& points = terrain.getTrack().getPoints();
        const Barrier& barrier = terrain.getBarrier();
        const size_t leave = terrain.getSlipLeave();
        size_t climb = leave;
        while (points[climb].m_kind == PathPoint::Kind::Flat)
        {
            ++climb;
        }
        const double pastTheMouth = dot(barrier.m_position - points[leave].m_position, points[leave].m_forward);
        const double beforeTheRamp = dot(points[climb].m_position - barrier.m_position, barrier.m_forward);
        std::cout << "    " << pastTheMouth << " m beyond where the dirt road leaves, " << beforeTheRamp
            << " m before the road starts to climb" << std::endl;
        check(pastTheMouth > 15.0, "the barrier stands beyond the mouth of the dirt road");
        check(beforeTheRamp > 2.0 && barrier.m_position.y < 1.0e-9, "and on level road, before the ramp to the bridge");
        check(!barrier.m_broken, "it stands at the start");

        // Round by the dirt road it is left alone
        const Driver driver;
        game.startAt(leave - 2);
        for (double t = 0.0; t < 40.0 && game.getProgress() < terrain.getSlipRejoin(); t += 1.0 / 60.0)
        {
            game.setControls(driver.follow(game.getTruck(), terrain.getDirtPath()));
            game.step(1.0 / 60.0);
        }
        check(game.getProgress() >= terrain.getSlipRejoin() && !barrier.m_broken,
            "a truck that goes round by the dirt road leaves it standing");

        // Straight on, and it goes down without holding the truck up
        Pilot pilot;
        game.startAt(leave);
        double slowest = 99.0;
        for (double t = 0.0; t < 8.0 && game.getProgress() < climb + 4; t += 1.0 / 60.0)
        {
            game.setControls(pilot.drive(game));
            game.step(1.0 / 60.0);
            slowest = t > 1.0 ? std::min(slowest, game.getTruck().getSpeed()) : slowest;
        }
        check(barrier.m_broken, "a truck that drives on to the bridge knocks it down");
        check(game.getProgress() >= climb && slowest > 5.0, "and is not stopped by it");
        game.reset();
        check(!game.getTerrain().getBarrier().m_broken, "a new race stands it up again");
    }

    std::cout << "Shortcut through the riverbed" << std::endl;
    {
        Game game(0);
        const Terrain& terrain = game.getTerrain();
        const Track& track = terrain.getTrack();
        const std::vector<PathPoint>& points = track.getPoints();
        const Detour& shortcut = terrain.getShortcut();
        const DirtRoad& bed = terrain.getRiverbed();
        const Vec3& ford = terrain.getFord();
        const size_t lastGate = track.getCheckpoints().back();
        const Vec3 down(0.0, -1.0, 0.0);
        const auto ground = [&](const Vec3& at)
        {
            Hit hit;
            check(terrain.raycast(Vec3(at.x, 6.0, at.z), down, 20.0, hit), "there is ground to stand on");
            return hit;
        };

        std::cout << "    lap " << track.getLength() << " m; last gate at " << points[lastGate].m_distance
            << " m, shortcut from " << points[shortcut.m_leave].m_distance << " to "
            << points[shortcut.m_rejoin].m_distance << " m" << std::endl;
        check(track.getLength() > 1100.0, "the lap is longer than it was");
        check(track.getLength() - points[lastGate].m_distance > 300.0, "with a long run home from the last gate");
        const bool beyondTheWrecks = std::all_of(terrain.getWrecks().begin(), terrain.getWrecks().end(),
            [&](const Wreck& wreck)
            {
                return dot(points[shortcut.m_leave].m_position - wreck.getPosition(), points[shortcut.m_leave].m_forward) > 5.0;
            });
        check(shortcut.m_leave > lastGate && beyondTheWrecks, "the shortcut leaves the road after the last gate and the wrecks");
        check(shortcut.m_rejoin > shortcut.m_leave && shortcut.m_rejoin + 1 < points.size()
            && track.getLength() - points[shortcut.m_rejoin].m_distance < 50.0,
            "and comes back to it just before the start and finish line");
        check(dot(shortcut.m_path[shortcut.m_path.size() / 2] - points[shortcut.m_leave].m_position,
            points[shortcut.m_leave].m_cross) < -20.0, "it runs to the left of the road");

        // The river
        const double level = terrain.getWaterLevel();
        const Vec3 right = -bed.m_forward; // of a truck driving the shortcut
        const Hit floor = ground(ford + bed.m_cross * 5.0);
        std::cout << "    floor at " << floor.m_point.y << " m, water at " << level << " m, held to "
            << floor.m_speedLimit * 3.6 << " km/h in it" << std::endl;
        check(level < -1.5, "the water lies well below the road, so there is a climb out of it");
        check(level - floor.m_point.y > 0.7 && level - floor.m_point.y < 1.7, "it is about as deep as a wheel is high");
        check(std::fabs(floor.m_speedLimit * 3.6 - 25.0) < 1.0e-6, "in the water a truck makes 25 km/h at most");
        check(ground(ford - bed.m_cross * 30.0).m_speedLimit == 0.0, "and nothing holds it back on dry land");
        check(std::fabs(ground(ford - bed.m_forward * 9.0).m_traction - 0.70) < 1.0e-9, "the floor of the river is gravel");
        check(ground(ford - bed.m_cross * (bed.halfWidth() - 1.0)).m_point.y > level + 1.0
            && std::fabs(ground(ford - bed.m_cross * (bed.halfWidth() - 1.0)).m_traction - 0.86) < 1.0e-9,
            "the track goes down the bank to it as dirt");

        // The bridge, looked at across its width a third of the way over the river
        const Vec3 onTheTrack = *std::min_element(shortcut.m_path.begin(), shortcut.m_path.end(),
            [&](const Vec3& a, const Vec3& b)
            {
                return length(a - (ford - bed.m_cross * 7.0)) < length(b - (ford - bed.m_cross * 7.0));
            });
        const Hit lane = ground(onTheTrack);
        const Hit sunk = ground(onTheTrack - right * 4.0);
        const Hit beside = ground(onTheTrack + right * 4.0);
        double dryWidth = 0.0;
        for (double side = -8.0; side < 8.0; side += 0.05)
        {
            const Hit hit = ground(onTheTrack + right * side);
            dryWidth += hit.m_point.y > level ? 0.05 : 0.0;
        }
        std::cout << "    bridge: the lane at " << lane.m_point.y << " m, its sunken side at " << sunk.m_point.y
            << " m, " << dryWidth << " m of it dry" << std::endl;
        check(lane.m_point.y > level + 0.2 && lane.m_point.y < -0.5 && lane.m_speedLimit == 0.0,
            "the track runs over the edge of the log bridge that is still above the water");
        check(sunk.m_point.y < level - 0.2 && sunk.m_point.y > floor.m_point.y + 0.05 && sunk.m_speedLimit > 0.0,
            "the other side of the bridge has sunk under water");
        check(std::fabs(beside.m_point.y - floor.m_point.y) < 1.0e-6, "and to the right of it is open water");
        check(dryWidth > 3.9 && dryWidth < 5.2, "the dry lane is little wider than a truck");
        check(std::fabs(ground(ford + bed.m_cross * 2.0 + right * 1.0).m_point.y - floor.m_point.y) < 1.0e-6
            && std::fabs(ground(ford + bed.m_cross * 6.0 - right * 3.0).m_point.y - floor.m_point.y) < 1.0e-6,
            "the second half of the bridge has gone");

        // Driven from the same spot: round by the road, and by the shortcut, slowing for the
        // river or not, along the dry lane or over the sunken side of the bridge
        const Driver driver;
        Driver::Style precise; // looks a short way ahead, and so keeps closely to the line it is given
        precise.m_lookAhead = 5.0;
        const Driver steady(precise);
        std::vector<Vec3> overTheSunkenSide = shortcut.m_path;
        for (Vec3& point : overTheSunkenSide)
        {
            point = point - (bed.contains(point) ? right * 4.5 : Vec3());
        }
        struct Run
        {
            double m_time = 0.0;
            double m_lowest = 9.0;
            double m_firstWet = 99.0; // how far over the river the first wheel went under [m]
            double m_wadeSpeed = 0.0; // fastest once it has been in the water a second [m/s]
            double m_wadeTime = 0.0;  // with a wheel in the water [s]
            bool m_arrived = false;
            bool m_upright = true;
        };
        const auto run = [&](const std::vector<Vec3>* path, double riverPace, const Driver& who)
        {
            Game trial(0);
            Pilot pilot;
            Run result;
            trial.startAt(shortcut.m_leave - 3);
            for (; result.m_time < 90.0 && trial.getProgress() < shortcut.m_rejoin; result.m_time += 1.0 / 60.0)
            {
                const Truck& truck = trial.getTruck();
                Controls controls = path != nullptr ? who.follow(truck, *path) : pilot.drive(trial);
                if (riverPace > 0.0 && length(truck.getPosition() - ford) < bed.halfWidth() + 22.0
                    && dot(truck.getPosition() - ford, bed.m_cross) < 0.0)
                {
                    controls.m_throttle = truck.getSpeed() < riverPace ? controls.m_throttle : 0.0;
                    controls.m_brake = truck.getSpeed() > riverPace + 1.0 ? 1.0 : 0.0;
                }
                trial.setControls(controls);
                trial.step(1.0 / 60.0);
                const bool wading = std::any_of(truck.getWheels().begin(), truck.getWheels().end(),
                    [&](const Wheel& wheel)
                    {
                        return wheel.m_onGround
                            && terrain.isWading(wheel.m_center - Vec3(0.0, truck.getWheelRadius(), 0.0));
                    });
                if (wading && result.m_wadeTime == 0.0)
                {
                    result.m_firstWet = dot(truck.getPosition() - ford, bed.m_cross);
                }
                result.m_wadeTime += wading ? 1.0 / 60.0 : 0.0;
                result.m_wadeSpeed = wading && result.m_wadeTime > 1.0
                    ? std::max(result.m_wadeSpeed, truck.getSpeed()) : result.m_wadeSpeed;
                result.m_lowest = std::min(result.m_lowest, truck.getPosition().y);
                result.m_upright = result.m_upright && truck.up().y > 0.5 && !trial.isWrecked();
            }
            result.m_arrived = trial.getProgress() >= shortcut.m_rejoin;
            return result;
        };
        const auto report = [](const char* name, const Run& result)
        {
            std::cout << "    " << name << ": " << result.m_time << " s, first wet " << result.m_firstWet
                << " m from the middle of the river, " << result.m_wadeTime << " s in the water at up to "
                << result.m_wadeSpeed * 3.6 << " km/h, lowest " << result.m_lowest << " m"
                << (result.m_arrived ? "" : ", did not arrive") << (result.m_upright ? "" : ", went over") << std::endl;
        };
        const Run road = run(nullptr, 0.0, driver);
        const Run careful = run(&shortcut.m_path, 7.0, steady);
        const Run flatOut = run(&shortcut.m_path, 0.0, driver);
        const Run sunkenSide = run(&overTheSunkenSide, 7.0, steady);
        report("by the road", road);
        report("the dry lane, slowing for the river", careful);
        report("the dry lane, flat out", flatOut);
        report("the sunken side, slowing for the river", sunkenSide);
        check(road.m_arrived && careful.m_arrived && sunkenSide.m_arrived, "road and shortcut both lead back to the road before the line");
        check(careful.m_upright && careful.m_firstWet > -7.5, "a truck that keeps to the dry lane stays dry most of the way over the bridge");
        check(careful.m_wadeTime > 1.2, "but then it has to wade: the second half of the bridge has gone");
        check(careful.m_wadeSpeed > 0.0 && careful.m_wadeSpeed < 25.0 / 3.6 + 0.7, "at no more than 25 km/h");
        check(careful.m_lowest < -1.0, "and climb back out of the river to the road");
        check(sunkenSide.m_upright && sunkenSide.m_firstWet < careful.m_firstWet - 3.0
            && sunkenSide.m_wadeTime > careful.m_wadeTime + 0.8, "over the sunken side of the bridge it wades for longer");
        check(sunkenSide.m_time > careful.m_time + 0.5, "which costs time");
        check(careful.m_time < road.m_time, "taken with care the shortcut still beats the road");
    }

    std::cout << "No lap not counted at the start" << std::endl;
    {
        Game game(0);
        Pilot pilot;
        const size_t last = game.getTerrain().getTrack().getPoints().size() - 1;
        bool announced = false;
        for (double t = 0.0; t < 8.0; t += 1.0 / 60.0)
        {
            game.setControls(pilot.drive(game));
            game.step(1.0 / 60.0);
            announced = announced || game.hudString().find("NOT COUNTED") != std::string::npos;
        }
        check(game.getPlayer().m_missedClock >= 0.0 && game.getLap() == 1,
            "crossing the line from the grid does not count as a lap");
        check(!announced, "but nothing is said about it in the first seconds of the race");

        // Later in the race the same thing is announced
        for (double t = 0.0; t < 9.0; t += 1.0 / 60.0)
        {
            game.setControls(Controls());
            game.step(1.0 / 60.0);
        }
        game.startAt(last - 14);
        for (double t = 0.0; t < 8.0 && !announced; t += 1.0 / 60.0)
        {
            game.setControls(pilot.drive(game));
            game.step(1.0 / 60.0);
            announced = game.hudString().find("NOT COUNTED") != std::string::npos;
        }
        std::cout << "    announced " << game.getRaceTime() << " s into the race" << std::endl;
        check(announced && game.getRaceTime() > 15.0, "a lap that does not count later on is announced");
    }

    std::cout << "Tyre setups" << std::endl;
    {
        // All of this runs on empty ground well clear of the circuit: a turn circle started near
        // the track sweeps back over the wreck at x = 35, and the collision, not the tyres, then
        // decides the result.
        const double step = 1.0 / 240.0;
        const auto runUp = [&](Truck& truck, Terrain& terrain, double seconds, double target)
        {
            for (double t = 0.0; t < seconds && truck.getSpeed() < target; t += step)
            {
                truck.step(step, drive(1.0, 0.0, 0.0), terrain);
            }
        };

        // Turn radius at full lock, and how far the truck leaned doing it. Radius is the measure
        // of grip here: at a fixed steering angle a grippier tyre turns tighter rather than
        // sliding less, so sideslip alone would read backwards.
        const auto corner = [&](int index, double target)
        {
            Terrain terrain;
            Truck truck;
            truck.setSetup(Truck::setupAt(index));
            truck.placeAt(Vec3(0.0, 1.64, 200.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
            runUp(truck, terrain, 30.0, target);

            double minUp = 1.0;
            double yaw = 0.0;
            int samples = 0;
            for (double t = 0.0; t < 4.0; t += step)
            {
                truck.step(step, drive(truck.getSpeed() < target ? 1.0 : 0.0, 0.0, 1.0), terrain);
                minUp = std::min(minUp, truck.up().y);
                if (t > 1.0)
                {
                    yaw += std::fabs(dot(truck.getAngularVelocity(), truck.up()));
                    ++samples;
                }
            }
            return std::make_pair(truck.getSpeed() * samples / yaw, minUp);
        };

        const auto topSpeed = [&](int index)
        {
            Terrain terrain;
            Truck truck;
            truck.setSetup(Truck::setupAt(index));
            truck.placeAt(Vec3(0.0, 1.64, 200.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
            runUp(truck, terrain, 25.0, 1000.0);
            return truck.getSpeed();
        };

        bool tipped = false;
        bool gripFalls = true;
        bool speedRises = true;
        double previousRadius = 0.0;
        double previousTop = 0.0;
        for (int index = 1; index <= Truck::setupCount(); ++index)
        {
            const auto [radius, minUp] = corner(index, 18.0);
            const double top = topSpeed(index);
            std::cout << "    " << index << " " << Truck::setupName(index)
                << ": grip " << Truck::setupAt(index).m_grip
                << ", radius " << radius << " m, top " << (int) std::lround(top * 3.6)
                << " km/h, min up.y " << minUp << std::endl;
            tipped = tipped || minUp < 0.9;
            gripFalls = gripFalls && (index == 1 || radius > previousRadius + 0.5);
            speedRises = speedRises && (index == 1 || top > previousTop + 1.0);
            previousRadius = radius;
            previousTop = top;
        }
        check(!tipped, "no setup tips the truck in a sustained full lock corner");
        check(gripFalls, "each harder setup needs a wider turn, so it grips less");
        check(speedRises, "each harder setup reaches a higher speed on the straight");
        check(corner(1, 18.0).first < 0.85 * corner(Truck::setupCount(), 18.0).first,
            "the softest tyre turns far tighter than the hardest");

        Truck truck;
        truck.setSetup(Truck::Setup{ 5.0, 40.0, 0.9 });
        check(truck.getSetup().m_grip == Truck::maxGrip(), "an absurd grip is clamped");
        truck.setSetup(Truck::Setup{ 0.0, 40.0, 0.9 });
        check(truck.getSetup().m_grip == Truck::minGrip(), "and so is a useless one");
    }

    std::cout << "Choosing a setup" << std::endl;
    {
        Game game(0);
        check(game.getSetupIndex() == 3, "the standard setup is the default");
        check(std::string(game.getSetupName()) == "STANDARD", "and it is named");

        game.selectSetup(0);
        check(game.getSetupIndex() == 1, "an index below the range is clamped to the softest");
        game.selectSetup(99);
        check(game.getSetupIndex() == Truck::setupCount(), "and above it to the hardest");

        game.selectSetup(1);
        const double grip = game.getTruck().getSetup().m_grip;
        game.reset();
        check(game.getSetupIndex() == 1 && game.getTruck().getSetup().m_grip == grip,
            "restarting keeps the chosen setup");
        game.recover();
        check(game.getSetupIndex() == 1, "and so does recovering");
    }

    std::cout << "Racers" << std::endl;
    {
        Game game(0);
        run(game, 16.0, drive(1.0, 0.0, 0.0)); // far enough to score and to clear the first gate
        const Racer& racer = game.getRacers().front();
        std::cout << "    " << racer.m_name << ": score " << racer.getScore() << " + wreck "
            << game.getCrushPoints() << ", gates " << racer.getCheckpointsPassed() << std::endl;

        check(game.getRacers().size() == 1, "the game holds exactly one racer");
        check(&game.getPlayer() == &racer && racer.m_name == "PLAYER", "and it is the player");
        check(&game.getTruck() == &racer.m_truck && &game.getJudge() == &racer.m_judge,
            "the game's truck and judge are the player's");
        check(game.getProgress() == racer.m_progress && game.getLap() == racer.m_lap
            && game.getLapTime() == racer.m_lapTime && game.getBestLapTime() == racer.m_bestLapTime,
            "progress, lap and lap times agree");
        check(racer.getCheckpointsPassed() >= 1
            && game.getCheckpointsPassed() == racer.getCheckpointsPassed()
            && game.lapIsComplete() == racer.lapIsComplete(), "checkpoints agree");
        check(game.isWrecked() == racer.m_wrecked, "the wrecked state agrees");
        check(racer.getScore() > 0 && game.getScore() == racer.getScore() + game.getCrushPoints(),
            "the score is the player's plus the wreck's");

        game.selectSetup(2);
        game.setControls(drive(0.25, 0.5, -1.0));
        check(game.getSetupIndex() == 2 && racer.m_setupIndex == 2
            && racer.m_truck.getSetup().m_grip == Truck::setupAt(2).m_grip, "a setup choice lands on the player");
        check(racer.m_controls.m_throttle == 0.25 && racer.m_controls.m_brake == 0.5
            && racer.m_controls.m_steer == -1.0, "and so do the controls");
    }

    std::cout << "Recovery" << std::endl;
    {
        Game game(0);
        run(game, 3.0, drive(1.0, 0.0, 0.0));
        const int scored = game.getScore();

        game.recover();
        check(game.getTruck().up().y > 0.99, "recovery sets the truck upright");
        check(game.getTruck().getSpeed() < 1.0e-9, "recovery brings it to rest");
        check(game.getScore() == scored, "recovery keeps the score");
        check(game.getProgress() == 0, "with no gate cleared it returns to the start of the lap");

        run(game, 2.0, drive(0.0, 0.0, 0.0));
        const Truck& truck = game.getTruck();
        std::cout << "    rests at y " << truck.getPosition().y << ", wheels " << truck.getWheelsOnGround() << std::endl;
        check(truck.getWheelsOnGround() == 4, "it is standing on the road afterwards");
        check(!game.isWrecked(), "and is not wrecked");
    }

    std::cout << "Upside down" << std::endl;
    {
        Terrain terrain;
        Truck truck;
        truck.placeAt(Vec3(10.0, 2.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, -1.0, 0.0));
        double lowest = truck.getPosition().y;
        // Full throttle throughout: input must not be able to launch a truck lying on its roof
        for (double t = 0.0; t < 5.0; t += 1.0 / 240.0)
        {
            truck.step(1.0 / 240.0, drive(1.0, 0.0, 0.0), terrain);
            lowest = std::min(lowest, truck.getPosition().y);
        }
        std::cout << "    rests at y " << truck.getPosition().y << ", lowest " << lowest
            << ", up.y " << truck.up().y << ", body contact " << truck.hasBodyContact() << std::endl;
        check(lowest > -0.5, "an upside down truck does not sink through the ground");
        check(truck.hasBodyContact(), "it rests on its roof");
        check(truck.getPosition().y > 0.5 && truck.getPosition().y < 2.5, "at a sensible height");
    }

    std::cout << "Full lap on autopilot" << std::endl;
    {
        Game game(0);
        Pilot pilot;
        const std::vector<PathPoint>& points = game.getTerrain().getTrack().getPoints();
        const double frame = 1.0 / 60.0;
        bool inverted = false;
        bool underBridge = false;
        int gatesPassed = 0;
        double bridgeHeight = 0.0;
        double worstOffset = 0.0;
        Vec3 worstAt;
        for (double t = 0.0; t < 120.0 && game.getLap() == 1 && !game.isWrecked(); t += frame)
        {
            game.setControls(pilot.drive(game));
            game.step(frame);
            const Truck& truck = game.getTruck();
            const Vec3& position = truck.getPosition();
            const PathPoint& here = points[game.getProgress()];
            inverted = inverted || (truck.up().y < -0.9 && truck.getWheelsOnGround() == 4);
            gatesPassed = std::max(gatesPassed, game.getCheckpointsPassed());
            underBridge = underBridge || (std::fabs(position.x - 62.0) < 1.0 && std::fabs(position.z) < 2.0 && position.y < 2.0);
            if (here.m_kind == PathPoint::Kind::Bridge)
            {
                bridgeHeight = std::max(bridgeHeight, position.y);
            }
            if (truck.getWheelsOnGround() > 0 && (here.m_kind == PathPoint::Kind::Loop || here.m_kind == PathPoint::Kind::Bridge))
            {
                // Sideways distance from the nearest centreline point around the progress marker
                const size_t first = game.getProgress() > 3 ? game.getProgress() - 3 : 0;
                const size_t last = std::min(game.getProgress() + 10, points.size() - 1);
                const auto nearest = std::min_element(points.begin() + first, points.begin() + last + 1,
                    [&](const PathPoint& a, const PathPoint& b)
                    {
                        return length(a.m_position - position) < length(b.m_position - position);
                    });
                const double offset = std::fabs(dot(position - nearest->m_position, nearest->m_cross));
                if (offset > worstOffset)
                {
                    worstOffset = offset;
                    worstAt = position;
                }
            }
        }
        std::cout << "    hud " << game.hudString() << std::endl;
        std::cout << "    bridge height " << bridgeHeight << ", worst offset " << worstOffset
            << " at (" << worstAt.x << ", " << worstAt.z << ")" << std::endl;
        check(!game.isWrecked(), "truck survives the lap");
        check(gatesPassed == game.getCheckpointCount(), "every checkpoint is passed on the way round");
        check(game.getLap() == 2, "lap is completed");
        check(game.getBestLapTime() > 30.0 && game.getBestLapTime() < 90.0, "lap time is recorded");
        check(underBridge, "drives through the tunnel");
        check(inverted, "drives upside down through the loop");
        check(bridgeHeight > 7.0, "drives over the bridge");
        check(worstOffset < 2.0, "stays in the middle of the loop and the bridge");
        check(game.getScore() >= 1300, "wreck, jump and loop all score");
    }

    std::cout << "Crossing a gate" << std::endl;
    {
        Game game(0);
        const Track& track = game.getTerrain().getTrack();
        const std::vector<PathPoint>& points = track.getPoints();
        const size_t gate = track.getCheckpoints().front();
        const double frame = 1.0 / 60.0;
        const auto driveOn = [&](double seconds)
        {
            game.setControls(drive(1.0, 0.0, 0.0));
            for (double t = 0.0; t < seconds; t += frame)
            {
                game.step(frame);
            }
        };

        // Beginning just past the gate: the truck never comes up to it, so it stays uncleared
        game.startAt(gate + 4);
        driveOn(5.0);
        std::cout << "    from past the gate: " << game.getCheckpointsPassed() << " cleared" << std::endl;
        check(game.getCheckpointsPassed() == 0, "a gate the truck began beyond is not cleared");

        // Beginning short of it and driving through does clear it
        game.startAt(gate - 15);
        check(game.getCheckpointsPassed() == 0, "the lap starts again with none cleared");
        driveOn(6.0);
        const Vec3 past = game.getTruck().getPosition() - points[gate].m_position;
        std::cout << "    from before the gate: " << game.getCheckpointsPassed()
            << " cleared, " << dot(past, points[gate].m_forward) << " m beyond it" << std::endl;
        check(game.getCheckpointsPassed() == 1, "driving through the gate clears it");
        check(dot(past, points[gate].m_forward) > 5.0, "having passed the far line");
    }

    std::cout << "Back to the last checkpoint" << std::endl;
    {
        Game game(0);
        const Track& track = game.getTerrain().getTrack();
        const std::vector<PathPoint>& points = track.getPoints();
        const double frame = 1.0 / 60.0;

        run(game, 3.0, drive(1.0, 0.0, 0.0));
        game.recover();
        check(length(game.getTruck().getPosition() - track.getStartFinish().m_position) < 3.0,
            "with no gate cleared, recovery returns to the start and finish line");

        const size_t gate = track.getCheckpoints().front();
        game.startAt(gate - 15);
        game.setControls(drive(1.0, 0.0, 0.0));
        for (double t = 0.0; t < 6.0; t += frame)
        {
            game.step(frame);
        }
        check(game.getCheckpointsPassed() == 1, "the first gate is cleared");

        game.recover();
        const Vec3 offset = game.getTruck().getPosition() - points[gate].m_position;
        std::cout << "    back at " << std::fabs(dot(offset, points[gate].m_forward)) << " m from the gate"
            << std::endl;
        check(std::fabs(dot(offset, points[gate].m_forward)) < 1.0
            && std::fabs(dot(offset, points[gate].m_cross)) < 1.0, "recovery returns to the gate it cleared");
        check(game.getCheckpointsPassed() == 1, "and the gate stays cleared");
    }

    std::cout << "Lap begun partway round" << std::endl;
    {
        Game game(0);
        Pilot pilot;
        game.startAt(game.getTerrain().getTrack().getCheckpoints().back() + 3);
        check(game.getCheckpointsPassed() == 0, "starting partway gives up the checkpoints of that lap");

        const double frame = 1.0 / 60.0;
        size_t previous = game.getProgress();
        bool reachedTheLine = false;
        for (double t = 0.0; t < 60.0 && !reachedTheLine && !game.isWrecked(); t += frame)
        {
            game.setControls(pilot.drive(game));
            game.step(frame);
            reachedTheLine = game.getProgress() < previous;
            previous = game.getProgress();
        }
        std::cout << "    hud " << game.hudString() << std::endl;
        check(reachedTheLine, "the truck drives round to the start line");
        check(game.getLap() == 1, "the lap is not counted");
        check(game.getBestLapTime() < 0.0, "and no lap time is recorded");
    }

    std::cout << "Loop needs speed" << std::endl;
    {
        Game game(0);
        Pilot pilot;
        const std::vector<PathPoint>& points = game.getTerrain().getTrack().getPoints();
        const double frame = 1.0 / 60.0;
        bool reachedTop = false;
        for (double t = 0.0; t < 60.0 && !game.isWrecked(); t += frame)
        {
            Controls controls = pilot.drive(game);
            if (points[game.getProgress()].m_kind == PathPoint::Kind::Loop || game.getTruck().getPosition().z > 100.0)
            {
                controls.m_throttle = game.getTruck().getSpeed() < 7.0 ? 1.0 : 0.0; // too slow for the loop
            }
            game.setControls(controls);
            game.step(frame);
            reachedTop = reachedTop || (game.getTruck().up().y < -0.9 && game.getTruck().getWheelsOnGround() == 4);
        }
        check(!reachedTop, "a slow truck does not make it over the top");
        check(game.getLap() == 1, "the loop cannot be skipped");
    }

    std::cout << "Starting grid" << std::endl;
    {
        Game race(Game::maxOpponents());
        const std::vector<Racer>& racers = race.getRacers();
        const PathPoint& line = race.getTerrain().getTrack().getStartFinish();
        bool behindTheLine = true;
        bool onTheRoad = true;
        bool apart = true;
        bool distinct = true;
        for (size_t i = 0; i < racers.size(); ++i)
        {
            const Vec3 offset = racers[i].m_truck.getPosition() - line.m_position;
            behindTheLine = behindTheLine && dot(offset, line.m_forward) < -racers[i].m_truck.getHullRadius();
            onTheRoad = onTheRoad && std::fabs(dot(offset, line.m_cross)) + 2.3 < race.getTerrain().getTrack().getHalfWidth();
            for (size_t j = i + 1; j < racers.size(); ++j)
            {
                apart = apart && Truck::overlap(racers[i].m_truck, racers[j].m_truck) == 0.0;
                distinct = distinct && racers[i].m_gridSlot != racers[j].m_gridSlot
                    && racers[i].m_name != racers[j].m_name
                    && (racers[i].m_color.r != racers[j].m_color.r || racers[i].m_color.g != racers[j].m_color.g
                        || racers[i].m_color.b != racers[j].m_color.b);
            }
        }
        check(racers.size() == 4, "the player and three opponents");
        check(!racers[0].m_driven && racers[1].m_driven && racers[2].m_driven && racers[3].m_driven,
            "the player drives, the computer drives the rest");
        check(distinct, "each has its own slot, name and colour");
        check(behindTheLine, "all of them are behind the start line");
        check(onTheRoad, "and on the road");
        check(apart, "without touching each other");
        check(race.getPosition() == 4, "the player starts fourth, at the back");
        check(race.getPosition(racers[1]) == 1, "and the front of the grid leads");
        check(Game(0).getPosition() == 1, "alone, the player is first of one");
    }

    std::cout << "Bumping" << std::endl;
    {
        // Two trucks on empty ground, clear of the circuit, driven at each other and then let go
        const double step = 1.0 / 240.0;
        const auto crash = [&](const Vec3& where, const Vec3& facing, double throttle)
        {
            Terrain terrain;
            Truck a;
            Truck b;
            a.placeAt(Vec3(0.0, 1.64, 200.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
            b.placeAt(where, facing, Vec3(0.0, 1.0, 0.0));
            bool touched = false;
            double lowest = 9.0;
            double highest = 0.0;
            double leastUpright = 1.0;
            for (double t = 0.0; t < 14.0; t += step)
            {
                const bool driving = t < 6.0;
                Truck::collide(a, b);
                a.step(step, drive(driving ? 1.0 : 0.0, 0.0, 0.0), terrain);
                b.step(step, drive(driving ? throttle : 0.0, 0.0, 0.0), terrain);
                touched = touched || Truck::overlap(a, b) > 0.0;
                for (const Truck* truck : { &a, &b })
                {
                    lowest = std::min(lowest, truck->getPosition().y);
                    highest = std::max(highest, truck->getPosition().y);
                    leastUpright = std::min(leastUpright, truck->up().y);
                }
            }
            std::cout << "    touched " << touched << ", height " << lowest << " to " << highest
                << " m, least upright " << leastUpright << ", overlap at the end "
                << Truck::overlap(a, b) << " m" << std::endl;
            return touched && Truck::overlap(a, b) == 0.0 && highest < 1.7 && lowest > 1.2 && leastUpright > 0.9;
        };
        check(crash(Vec3(120.0, 1.64, 200.0), Vec3(-1.0, 0.0, 0.0), 1.0),
            "head-on: they separate, stay on the ground, above the road and upright");
        check(crash(Vec3(40.0, 1.64, 200.0), Vec3(1.0, 0.0, 0.0), 0.0),
            "from behind into a standing truck: the same");

        // Equal and opposite: two trucks pushed apart from rest pick up no net momentum
        Terrain terrain;
        Truck a;
        Truck b;
        a.placeAt(Vec3(0.0, 1.39, 200.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
        b.placeAt(Vec3(3.0, 1.39, 200.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
        check(Truck::overlap(a, b) > 0.5, "two trucks set down inside each other overlap");
        Truck::collide(a, b);
        a.step(step, drive(0.0, 0.0, 0.0), terrain);
        b.step(step, drive(0.0, 0.0, 0.0), terrain);
        check(a.getVelocity().x < 0.0 && b.getVelocity().x > 0.0
            && std::fabs(a.getVelocity().x + b.getVelocity().x) < 1.0e-6, "the push is equal and opposite");
    }

    std::cout << "Three lap race" << std::endl;
    {
        Game race(Game::maxOpponents());
        Pilot pilot;
        const std::vector<Racer>& racers = race.getRacers();
        const double frame = 1.0 / 60.0;
        std::vector<int> laps(racers.size(), 1);
        std::vector<int> gates(racers.size(), 0);
        double lowest = 9.0;
        double deepest = 0.0;
        double lowestOnTheBridge = 9.0;
        const DirtRoad& cutting = race.getTerrain().getDirtRoad();
        const DirtRoad& riverbed = race.getTerrain().getRiverbed();
        std::vector<int> byDirtRoad(racers.size(), 0);
        std::vector<int> byShortcut(racers.size(), 0);
        std::vector<bool> inTheCutting(racers.size(), false);
        std::vector<bool> inTheRiverbed(racers.size(), false);
        const std::vector<PathPoint>& where = race.getTerrain().getTrack().getPoints();
        bool lapsWereComplete = true;
        bool overWithThePlayer = true;
        const auto allFinished = [&]()
        {
            return std::all_of(racers.begin(), racers.end(), [](const Racer& racer) { return racer.m_finished; });
        };
        for (double t = 0.0; t < 400.0 && !allFinished(); t += frame)
        {
            race.setControls(pilot.drive(race));
            race.step(frame);
            for (size_t i = 0; i < racers.size(); ++i)
            {
                // A lap is credited at the line, long after the last gate: the frame before, every
                // gate of that lap must already have been cleared
                if (racers[i].m_lap != laps[i])
                {
                    lapsWereComplete = lapsWereComplete && gates[i] == race.getCheckpointCount();
                    laps[i] = racers[i].m_lap;
                }
                gates[i] = racers[i].getCheckpointsPassed();
                lowest = std::min(lowest, racers[i].m_truck.getPosition().y);
                // Down in the cutting on purpose: a truck knocked into it off the road does not count
                const bool sunk = cutting.contains(racers[i].m_truck.getPosition())
                    && racers[i].m_truck.getPosition().y < 0.6 && racers[i].m_detour == 0;
                byDirtRoad[i] += sunk && !inTheCutting[i] ? 1 : 0;
                inTheCutting[i] = sunk;
                const bool fording = riverbed.contains(racers[i].m_truck.getPosition());
                byShortcut[i] += fording && !inTheRiverbed[i] ? 1 : 0;
                inTheRiverbed[i] = fording;
                const PathPoint::Kind kind = where[racers[i].m_progress].m_kind;
                if (kind == PathPoint::Kind::Bridge || kind == PathPoint::Kind::Gap)
                {
                    lowestOnTheBridge = std::min(lowestOnTheBridge, racers[i].m_truck.getPosition().y);
                }
                // While the race is on. Once the player has finished, the truck is parked on the
                // road beyond the line, and the others arrive there at full speed.
                for (size_t j = i + 1; j < racers.size() && !race.isRaceOver(); ++j)
                {
                    deepest = std::max(deepest, Truck::overlap(racers[i].m_truck, racers[j].m_truck));
                }
            }
            overWithThePlayer = overWithThePlayer && race.isRaceOver() == racers.front().m_finished;
        }

        std::vector<int> places;
        bool sensibleTimes = true;
        int wreckPoints = 0;
        int finishedOff = 0;
        for (const Racer& racer : racers)
        {
            std::cout << "    " << racer.m_name << ": place " << racer.m_finishPlace << " in "
                << racer.m_finishTime << " s, best lap " << racer.m_bestLapTime
                << " s, score " << race.getScore(racer) << std::endl;
            places.push_back(racer.m_finishPlace);
            sensibleTimes = sensibleTimes && racer.m_finishTime > 120.0 && racer.m_finishTime < 300.0;
            wreckPoints += race.getCrushPoints(racer);
            finishedOff += racer.m_flattened ? 1 : 0;
        }
        std::sort(places.begin(), places.end());
        std::cout << "    lowest truck " << lowest << " m, deepest hull overlap " << deepest << " m" << std::endl;
        std::cout << "    hud " << race.hudString() << std::endl;

        check(allFinished(), "all four racers complete three laps");
        check(sensibleTimes, "each in a sensible time");
        check(places == std::vector<int>({ 1, 2, 3, 4 }), "they finish first to fourth, no place shared");
        check(lapsWereComplete, "every credited lap passed every checkpoint");
        check(lowest > -1.0, "nobody falls through the ground, the dirt road's cutting included");
        std::cout << "    lowest on the bridge " << lowestOnTheBridge << " m" << std::endl;
        check(lowestOnTheBridge > 3.0, "whoever takes the bridge clears the hole instead of dropping through");
        const int opponentsByDirt = byDirtRoad[1] + byDirtRoad[2] + byDirtRoad[3];
        std::cout << "    by the dirt road: opponents " << opponentsByDirt << " times out of 9, the player "
            << byDirtRoad[0] << std::endl;
        check(opponentsByDirt >= 5, "the opponents take the dirt road most of the time");
        check(opponentsByDirt < 9, "but not always");
        const int opponentsByShortcut = byShortcut[1] + byShortcut[2] + byShortcut[3];
        std::cout << "    through the riverbed: opponents " << opponentsByShortcut << " times out of 9, the player "
            << byShortcut[0] << std::endl;
        check(opponentsByShortcut == 0, "the opponents never take the shortcut");
        check(deepest < 0.8, "trucks bump but do not drive through each other");
        check(overWithThePlayer && race.isRaceOver(), "the race is over when the player finishes");
        check(race.getPosition() == racers.front().m_finishPlace, "the player's position is the finishing place");
        check(finishedOff == 1 && wreckPoints >= 498 && wreckPoints <= 502,
            "the wreck's points go to whoever crushed it, once");

        // After the flag the player's truck is brought to a stop
        const double speedAtTheFlag = race.getTruck().getSpeed();
        race.setControls(drive(1.0, 0.0, 0.0));
        for (double t = 0.0; t < 8.0; t += frame)
        {
            race.step(frame);
        }
        std::cout << "    player once the race is over: " << speedAtTheFlag << " -> " << race.getTruck().getSpeed()
            << " m/s" << std::endl;
        check(race.getTruck().getSpeed() < 3.0, "after the finish the player's truck stops, whatever the keys say");
    }

    std::cout << "Restarting the race" << std::endl;
    {
        Game fresh(Game::maxOpponents());
        Game race(Game::maxOpponents());
        Pilot pilot;
        const double frame = 1.0 / 60.0;
        for (double t = 0.0; t < 70.0; t += frame) // into the second lap for most of them
        {
            race.setControls(pilot.drive(race));
            race.step(frame);
        }
        const int lapsBefore = race.getRacers()[1].m_lap;
        race.reset();

        bool onTheGrid = true;
        bool nothingDone = true;
        for (size_t i = 0; i < race.getRacers().size(); ++i)
        {
            const Racer& racer = race.getRacers()[i];
            onTheGrid = onTheGrid
                && length(racer.m_truck.getPosition() - fresh.getRacers()[i].m_truck.getPosition()) < 1.0e-9
                && racer.m_truck.getSpeed() == 0.0;
            nothingDone = nothingDone && racer.m_lap == 1 && racer.m_progress == 0
                && racer.getCheckpointsPassed() == 0 && !racer.m_finished && race.getScore(racer) == 0;
        }
        check(lapsBefore > 1, "the race was well under way");
        check(onTheGrid, "a restart puts all four back on the grid, standing still");
        check(nothingDone, "with no laps, gates or points to their name");
        check(race.getRaceTime() == 0.0 && race.getPosition() == 4, "and the clock and the order start again");
    }

    std::cout << (g_failures == 0 ? "All tests passed" : "Tests FAILED") << std::endl;
    return g_failures == 0 ? 0 : 1;
}
