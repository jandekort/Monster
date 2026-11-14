// Monster - native tests for terrain, truck physics and scoring
// Author: Jan de Kort
// Created: 2026-11-14
#include "Autopilot.h"
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
        check(heightAt(0.0, 0.0, 5.0) == 0.0, "flat at the start");
        check(std::fabs(heightAt(97.0, 0.0, 10.0) - 1.75) < 1.0e-9, "jump ramp is linear");
        check(heightAt(97.0, 6.0, 10.0) == 0.0, "jump ramp has a finite width");
        check(heightAt(110.0, 0.0, 10.0) == 0.0, "gap between the ramps");
        check(terrain.raycast(Vec3(97.0, 10.0, 0.0), down, 100.0, hit) && hit.m_normal.x < -0.2, "ramp normal leans back");
        check(terrain.raycast(Vec3(35.0, 5.0, 0.0), down, 100.0, hit) && hit.m_wreck && hit.m_point.y > 1.0, "wreck is in the way");
        check(std::fabs(heightAt(62.0, 0.0, 20.0) - terrain.getTrack().getBridgeHeight()) < 1.0e-6, "bridge deck over the crossing");
        check(heightAt(62.0, 0.0, 3.0) == 0.0, "road passes under the bridge");
        check(terrain.raycast(Vec3(40.0, 1.5, 30.0), Vec3(1.0, 0.0, 0.0), 40.0, hit) && hit.m_normal.x < -0.99, "embankment has walls");
        check(!terrain.raycast(Vec3(40.0, 1.5, 0.0), Vec3(1.0, 0.0, 0.0), 40.0, hit), "tunnel is open");
        check(!terrain.raycast(Vec3(0.0, 1.0, 0.0), Vec3(0.0, 1.0, 0.0), 100.0, hit), "nothing above the start");

        bool thrown = false;
        try
        {
            Wreck wreck(0.0, 0.0);
            wreck.applyLoad(1.0, -1.0);
        }
        catch (const std::logic_error&)
        {
            thrown = true;
        }
        check(thrown, "negative time step is rejected");
    }

    std::cout << "Truck at rest" << std::endl;
    {
        Game game;
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
        Game game;
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
        Game game;
        run(game, 3.0, drive(0.0, 1.0, 0.0));
        check(game.getTruck().getPosition().x < -3.0, "brake reverses from standstill");
    }

    std::cout << "Jump" << std::endl;
    {
        Game game;
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
            << ", peak " << peak << ", crush " << game.getTerrain().getWreck().getCrush()
            << ", up.y " << truck.up().y << std::endl;
        std::cout << "    hud " << game.hudString() << std::endl;
        check(game.getTerrain().getWreck().getCrush() > 0.9, "wreck is crushed");
        check(game.getCrushPoints() > 250, "crushing scores");
        check(flew, "truck leaves the ramp");
        check(peak > 4.5, "jump gains height");
        check(truck.getPosition().x > 140.0, "clears the gap");
        check(std::fabs(truck.getPosition().z) < 2.0, "tracks straight");
        check(game.getJudge().getLastRating() != Judge::Rating::None, "landing is rated");
        check(truck.up().y > 0.8 && !game.isWrecked(), "lands on its wheels");

        const int score = game.getScore();
        game.reset();
        check(game.getScore() == 0 && game.getBestScore() == score, "reset keeps the best score");
        check(game.getTerrain().getWreck().getCrush() == 0.0, "reset restores the wreck");
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
        check(radius < 45.0, "the turn is tight");
        check(minUp > 0.9, "a hard corner does not tip the truck");
        check(dot(truck.forward(), entry) < 0.0, "the truck turns well past a right angle");
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
        Game game;
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

    std::cout << "Recovery" << std::endl;
    {
        Game game;
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
        Game game;
        Autopilot pilot;
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
        Game game;
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
        Game game;
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
        Game game;
        Autopilot pilot;
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
        Game game;
        Autopilot pilot;
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

    std::cout << (g_failures == 0 ? "All tests passed" : "Tests FAILED") << std::endl;
    return g_failures == 0 ? 0 : 1;
}
