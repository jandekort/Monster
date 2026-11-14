// Monster - a simple driver that follows the centreline, used by the tests
// Author: Jan de Kort
// Created: 2026-11-14
#include "Autopilot.h"

#include <algorithm>
#include <cmath>

Autopilot::Autopilot()
    :
    m_lookAhead(14.0)
    , m_steerGain(2.5)
    , m_cornerSpeed(12.0)
    , m_braking(4.5)
    , m_planDistance(80.0)
{
}

Controls Autopilot::drive(const Game& game) const
{
    const std::vector<PathPoint>& points = game.getTerrain().getTrack().getPoints();
    const Truck& truck = game.getTruck();
    const size_t here = game.getProgress();
    const size_t count = points.size() - 1; // the last point repeats the first
    const double lapLength = game.getTerrain().getTrack().getLength();
    const auto distanceAhead = [&](size_t index)
    {
        const double d = points[index].m_distance - points[here].m_distance;
        return d < 0.0 ? d + lapLength : d;
    };

    Controls controls;
    controls.m_throttle = 1.0;

    // In the loop and in the air: keep the wheels straight and the throttle open
    if (truck.up().y < 0.6 || truck.getWheelsOnGround() == 0 || points[here].m_kind == PathPoint::Kind::Loop)
    {
        return controls;
    }

    { // Steer towards a point further along the centreline
        size_t target = here;
        while (distanceAhead(target) < m_lookAhead)
        {
            target = (target + 1) % count;
        }
        const Vec3 to = points[target].m_position - truck.getPosition();
        const double angle = std::atan2(dot(to, truck.right()), dot(to, truck.forward()));
        controls.m_steer = clamp(m_steerGain * angle, -1.0, 1.0);
    }

    { // Slow down for the bends ahead
        double limit = 100.0;
        for (size_t index = here; distanceAhead(index) < m_planDistance; index = (index + 1) % count)
        {
            if (points[index].m_corner)
            {
                limit = std::min(limit, std::sqrt(m_cornerSpeed*m_cornerSpeed + 2.0 * m_braking * distanceAhead(index)));
            }
        }
        const double speed = truck.getSpeed();
        controls.m_throttle = speed < limit ? 1.0 : 0.0;
        controls.m_brake = speed > limit + 1.0 ? 1.0 : 0.0;
    }

    return controls;
}
