// Monster - a computer driver: follows the centreline, brakes for bends, floors it for the loop
// Author: Jan de Kort
// Created: 2026-11-14
#include "Driver.h"

#include "Check.h"
#include "Track.h"

#include <algorithm>
#include <cmath>

Driver::Driver()
    :
    m_style()
{
}

Driver::Driver(const Style& style)
    :
    m_style(style)
{
    require(style.m_lookAhead > 0.0 && style.m_planDistance > 0.0, "Driver: distances must be positive");
    require(style.m_cornerSpeed > 0.0 && style.m_braking > 0.0, "Driver: speeds must be positive");
    require(style.m_throttleCap > 0.0 && style.m_throttleCap <= 1.0, "Driver: throttle cap must be in (0, 1]");
}

Controls Driver::follow(const Truck& truck, const std::vector<Vec3>& path) const
{
    return follow(truck, path, std::vector<double>());
}

Controls Driver::follow(const Truck& truck, const std::vector<Vec3>& path, const std::vector<double>& speeds) const
{
    require(!path.empty(), "Driver::follow: nothing to follow");
    require(speeds.empty() || speeds.size() == path.size(), "Driver::follow: one speed for every point of the path");
    const Vec3& position = truck.getPosition();
    const auto flatDistance = [&](const Vec3& point)
    {
        return std::hypot(point.x - position.x, point.z - position.z);
    };

    size_t nearest = 0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        if (flatDistance(path[i]) < flatDistance(path[nearest]))
        {
            nearest = i;
        }
    }
    const double set = speeds.empty() ? 0.0 : speeds[nearest];
    const double lookAhead = set > 0.0 ? std::min(m_style.m_lookAhead, m_style.m_carefulLookAhead) : m_style.m_lookAhead;
    size_t target = nearest;
    while (target + 1 < path.size() && flatDistance(path[target]) < lookAhead)
    {
        ++target;
    }

    Controls controls;
    const Vec3 to = path[target] - position;
    controls.m_steer = clamp(m_style.m_steerGain * std::atan2(dot(to, truck.right()), dot(to, truck.forward())), -1.0, 1.0);
    const double pace = m_style.m_detourPace * m_style.m_cornerSpeed;
    const double limit = set > 0.0 ? std::min(set, pace) : pace;
    controls.m_throttle = truck.getSpeed() < limit ? 1.0 : 0.0;
    controls.m_brake = truck.getSpeed() > limit + 1.0 ? 1.0 : 0.0;
    return controls;
}

Controls Driver::drive(const Truck& truck, size_t progress, const Track& track) const
{
    const std::vector<PathPoint>& points = track.getPoints();
    const size_t here = progress;
    const size_t count = points.size() - 1; // the last point repeats the first
    const double lapLength = track.getLength();
    const auto distanceAhead = [&](size_t index)
    {
        const double d = points[index].m_distance - points[here].m_distance;
        return d < 0.0 ? d + lapLength : d;
    };

    Controls controls;
    controls.m_throttle = 1.0;

    // In the loop and in the air: keep the wheels straight and the throttle open
    if (truck.up().y < 0.6 || truck.getWheelsOnGround() == 0
        || points[here].m_kind == PathPoint::Kind::Loop || points[here].m_kind == PathPoint::Kind::Gap)
    {
        return controls;
    }

    { // Steer towards a point further along the centreline
        size_t target = here;
        while (distanceAhead(target) < m_style.m_lookAhead)
        {
            target = (target + 1) % count;
        }
        const Vec3 to = points[target].m_position - truck.getPosition();
        const double angle = std::atan2(dot(to, truck.right()), dot(to, truck.forward()));
        controls.m_steer = clamp(m_style.m_steerGain * angle, -1.0, 1.0);
    }

    { // Slow down for the bends ahead, but never hold back on the run into the loop
        double limit = 100.0;
        bool loopAhead = false;
        for (size_t index = here; distanceAhead(index) < m_style.m_planDistance; index = (index + 1) % count)
        {
            if (points[index].m_corner)
            {
                limit = std::min(limit, std::sqrt(m_style.m_cornerSpeed*m_style.m_cornerSpeed
                    + 2.0 * m_style.m_braking * distanceAhead(index)));
            }
            loopAhead = loopAhead || points[index].m_kind == PathPoint::Kind::Loop
                || points[index].m_kind == PathPoint::Kind::Gap;
        }
        const double speed = truck.getSpeed();
        controls.m_throttle = speed < limit ? (loopAhead ? 1.0 : m_style.m_throttleCap) : 0.0;
        controls.m_brake = speed > limit + 1.0 ? 1.0 : 0.0;
    }

    return controls;
}
