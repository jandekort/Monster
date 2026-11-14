// Monster - rigid body monster truck with four sprung wheels
// Author: Jan de Kort
// Created: 2026-11-14
#include "Truck.h"

#include "Check.h"
#include "Terrain.h"

#include <algorithm>
#include <cmath>

// Struct Wheel
Wheel::Wheel(const Vec3& attach, bool steered)
    :
    m_attach(attach)
    , m_steered(steered)
    , m_onGround(false)
    , m_compression(0.0)
    , m_spin(0.0)
    , m_spinRate(0.0)
    , m_center()
    , m_axle(0.0, 0.0, 1.0)
{
}

// Class Truck
Truck::Truck()
    :
    m_position()
    , m_velocity()
    , m_angularVelocity()
    , m_orientation()
    , m_steerAngle(0.0)
    , m_wheelsOnGround(0)
    , m_bodyContact(false)
    , m_contactNormal(0.0, 1.0, 0.0)
    , m_wheels()
    , m_bodyPoints()
    , m_mass(4500.0)
    , m_inertia(6000.0, 12000.0, 11000.0)
    , m_gravity(9.81)
    , m_wheelRadius(0.84)
    , m_wheelHalfWidth(0.5)
    , m_suspensionLength(0.8)
    , m_suspensionTravel(0.6)
    , m_springRate(44000.0)
    , m_damperRate(5600.0)
    , m_antiRollRate(60000.0)
    , m_bumpStopRate(250000.0)
    , m_maxWheelLoad(150000.0)
    , m_wheelSpokes(7)
    , m_spokeSpread(60.0 * M_PI / 180.0)
    , m_maxCompression(0.9)
    , m_rayMargin(0.5)
    , m_grip(1.10)
    , m_slipSaturation(0.9)  // lateral slip speed at which the tyre reaches full grip
    , m_driveForce(8000.0)
    , m_brakeForce(9000.0)
    , m_topSpeed(40.0)
    , m_reverseSpeed(12.0)
    , m_rollingDrag(40.0)
    , m_maxSteer(0.5)
    , m_steerRate(2.5)
    , m_steerFadeSpeed(15.0)
    , m_bodyStiffness(200000.0)
    , m_bodyDamping(15000.0)
    , m_bodyFriction(0.6)
    , m_airPitchRate(0.7)
    , m_airDamping(0.3)
{
    { // Wheels
        const double axleX = 1.75;
        const double axleY = 0.0;
        const double trackZ = 1.80;
        m_wheels.reserve(4);
        m_wheels.push_back(Wheel(Vec3( axleX, axleY, -trackZ), true));
        m_wheels.push_back(Wheel(Vec3( axleX, axleY,  trackZ), true));
        m_wheels.push_back(Wheel(Vec3(-axleX, axleY, -trackZ), false));
        m_wheels.push_back(Wheel(Vec3(-axleX, axleY,  trackZ), false));
    }

    { // Collision points: chassis corners, belly and roof
        m_bodyPoints.reserve(14);
        m_bodyPoints.push_back(Vec3( 2.3, -0.1, -1.2));
        m_bodyPoints.push_back(Vec3( 2.3, -0.1,  1.2));
        m_bodyPoints.push_back(Vec3(-2.3, -0.1, -1.2));
        m_bodyPoints.push_back(Vec3(-2.3, -0.1,  1.2));
        m_bodyPoints.push_back(Vec3( 2.3,  0.6, -1.2));
        m_bodyPoints.push_back(Vec3( 2.3,  0.6,  1.2));
        m_bodyPoints.push_back(Vec3(-2.3,  0.6, -1.2));
        m_bodyPoints.push_back(Vec3(-2.3,  0.6,  1.2));
        m_bodyPoints.push_back(Vec3( 0.0, -0.1, -1.2));
        m_bodyPoints.push_back(Vec3( 0.0, -0.1,  1.2));
        m_bodyPoints.push_back(Vec3( 1.1,  1.3, -1.0));
        m_bodyPoints.push_back(Vec3( 1.1,  1.3,  1.0));
        m_bodyPoints.push_back(Vec3(-0.7,  1.3, -1.0));
        m_bodyPoints.push_back(Vec3(-0.7,  1.3,  1.0));
    }
}

double Truck::getRideHeight() const
{
    return m_wheelRadius + m_suspensionLength - m_wheels.front().m_attach.y;
}

void Truck::reset(const Vec3& position)
{
    placeAt(position, Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
}

void Truck::placeAt(const Vec3& position, const Vec3& forward, const Vec3& normal)
{
    const Vec3 u = normalize(normal, Vec3(0.0, 1.0, 0.0));
    const Vec3 f = normalize(forward - u * dot(forward, u), Vec3(1.0, 0.0, 0.0));
    m_position = position;
    m_velocity = Vec3();
    m_angularVelocity = Vec3();
    m_orientation = Quat::fromAxes(f, u, cross(f, u));
    m_steerAngle = 0.0;
    m_wheelsOnGround = 0;
    m_bodyContact = false;
    m_contactNormal = Vec3(0.0, 1.0, 0.0);
    for (Wheel& wheel : m_wheels)
    {
        wheel.m_onGround = false;
        wheel.m_compression = 0.0;
        wheel.m_spin = 0.0;
        wheel.m_spinRate = 0.0;
        wheel.m_center = toWorld(wheel.m_attach) - up() * m_suspensionLength;
        wheel.m_axle = right();
    }
}

void Truck::step(double dt, const Controls& controls, Terrain& terrain)
{
    require(dt > 0.0, "Truck::step: dt must be positive");

    Vec3 force(0.0, -m_mass * m_gravity, 0.0);
    Vec3 torque;

    steer(dt, controls);
    wheelForces(dt, controls, terrain, force, torque);
    bodyForces(terrain, force, torque);
    airControl(controls, torque);
    integrate(dt, force, torque);
}

void Truck::steer(double dt, const Controls& controls)
{
    const double lock = m_maxSteer / (1.0 + getSpeed() / m_steerFadeSpeed);
    const double target = clamp(controls.m_steer, -1.0, 1.0) * lock;
    const double maxDelta = m_steerRate * dt;
    m_steerAngle += clamp(target - m_steerAngle, -maxDelta, maxDelta);
}

void Truck::wheelForces(double dt, const Controls& controls, Terrain& terrain, Vec3& force, Vec3& torque)
{
    const Vec3 f = forward();
    const Vec3 u = up();
    const Vec3 r = right();
    const double throttle = clamp(controls.m_throttle, 0.0, 1.0);
    const double brake = clamp(controls.m_brake, 0.0, 1.0);
    const double cosSteer = std::cos(m_steerAngle);
    const double sinSteer = std::sin(m_steerAngle);

    m_wheelsOnGround = 0;
    for (size_t i = 0; i < m_wheels.size(); ++i)
    {
        Wheel& wheel = m_wheels[i];
        const Vec3 heading = wheel.m_steered ? f*cosSteer + r*sinSteer : f;
        const Vec3 axle = wheel.m_steered ? r*cosSteer - f*sinSteer : r;
        const Vec3 extended = toWorld(wheel.m_attach) - u * m_suspensionLength; // hub, suspension extended

        // Each spoke's rim point is swept along the suspension axis; the spoke that needs
        // the most compression to clear the surface carries the wheel
        double depth = 0.0;
        Vec3 normal;
        Vec3 contact;
        bool onWreck = false;
        const double sweep = m_maxCompression + m_rayMargin;
        for (int k = 0; k < m_wheelSpokes; ++k)
        {
            const double angle = m_spokeSpread * (2.0 * k / (double) (m_wheelSpokes - 1) - 1.0);
            const Vec3 rim = extended + (heading * std::sin(angle) - u * std::cos(angle)) * m_wheelRadius;
            Hit hit;
            if (terrain.raycast(rim + u * sweep, -u, sweep, hit) && sweep - hit.m_distance > depth
                && dot(hit.m_normal, u) > 0.1)
            {
                depth = sweep - hit.m_distance;
                normal = hit.m_normal;
                contact = hit.m_point;
                onWreck = hit.m_wreck;
            }
        }

        double compression = 0.0;
        double rollSpeed = 0.0;
        wheel.m_onGround = depth > 0.0;
        if (wheel.m_onGround)
        {
            ++m_wheelsOnGround;

            // Suspension
            compression = std::min(depth, m_maxCompression);
            m_contactNormal = normal;
            const double rate = (compression - wheel.m_compression) / dt;
            const Wheel& partner = m_wheels[i ^ 1]; // other wheel on the same axle
            double load = m_springRate*compression + m_damperRate*rate
                + m_antiRollRate * (compression - partner.m_compression);
            if (compression > m_suspensionTravel)
            {
                load += m_bumpStopRate * (compression - m_suspensionTravel);
            }
            load = clamp(load, 0.0, m_maxWheelLoad);

            // Tyre: longitudinal drive and brake, lateral grip
            const Vec3 arm = contact - m_position;
            const Vec3 velocity = m_velocity + cross(m_angularVelocity, arm);
            const Vec3 along = normalize(heading - normal * dot(heading, normal), f);
            const Vec3 across = cross(normal, along);
            rollSpeed = dot(velocity, along);
            const double slipSpeed = dot(velocity, across);

            double drive = throttle * m_driveForce * std::max(0.0, 1.0 - rollSpeed / m_topSpeed);
            if (brake > 0.0)
            {
                if (rollSpeed > 0.5)
                {
                    drive -= brake * m_brakeForce;
                }
                else
                {
                    drive -= brake * 0.6 * m_driveForce * std::max(0.0, 1.0 + rollSpeed / m_reverseSpeed);
                }
            }
            drive -= m_rollingDrag * rollSpeed;
            double side = -m_grip * load * clamp(slipSpeed / m_slipSaturation, -1.0, 1.0);

            const double limit = m_grip * load;
            const double demand = std::sqrt(drive*drive + side*side);
            if (demand > limit)
            {
                drive *= limit / demand;
                side *= limit / demand;
            }

            const Vec3 total = normal*load + along*drive + across*side;
            force += total;
            torque += cross(arm, total);
            if (onWreck)
            {
                terrain.crush(load, dt);
            }
        }

        // Wheel state for rendering
        if (wheel.m_onGround)
        {
            wheel.m_spinRate = rollSpeed / m_wheelRadius;
        }
        else
        {
            const double freeRate = (throttle - brake) * m_topSpeed / m_wheelRadius;
            wheel.m_spinRate += (freeRate - wheel.m_spinRate) * std::min(1.0, 3.0 * dt);
        }
        wheel.m_spin += wheel.m_spinRate * dt;
        wheel.m_compression = compression;
        wheel.m_center = extended + u * std::min(compression, m_suspensionTravel);
        wheel.m_axle = axle;
    }
}

void Truck::bodyForces(const Terrain& terrain, Vec3& force, Vec3& torque)
{
    m_bodyContact = false;
    for (const Vec3& local : m_bodyPoints)
    {
        const Vec3 arm = m_orientation.rotate(local);

        // The ground is a solid half-space, not a surface seen from above: a point below it is
        // pushed out even when the centre of mass itself has ended up under the surface
        Vec3 normal(0.0, 1.0, 0.0);
        double depth = std::max(0.0, -(m_position.y + arm.y));

        { // Road surfaces: one between the centre of mass and the point means the point is through it
            const double reach = length(arm);
            const Vec3 ray = arm * (1.0 / reach);
            Hit hit;
            if (terrain.raycast(m_position, ray, reach, hit))
            {
                const double through = (reach - hit.m_distance) * -dot(ray, hit.m_normal);
                if (through > depth)
                {
                    depth = through;
                    normal = hit.m_normal;
                }
            }
        }

        if (depth > 0.0)
        {
            m_bodyContact = true;
            const Vec3 velocity = m_velocity + cross(m_angularVelocity, arm);
            const double approach = dot(velocity, normal);
            const double load = clamp(m_bodyStiffness*depth - m_bodyDamping*approach, 0.0, m_maxWheelLoad);
            const Vec3 sliding = velocity - normal * approach;
            const Vec3 friction = sliding * (-m_bodyFriction * load / (length(sliding) + 0.5));
            const Vec3 total = normal * load + friction;
            force += total;
            torque += cross(arm, total);
        }
    }
}

// In the air the wheels act as reaction wheels: throttle lifts the nose, brake drops it
void Truck::airControl(const Controls& controls, Vec3& torque) const
{
    if (m_wheelsOnGround == 0 && !m_bodyContact)
    {
        const double input = clamp(controls.m_throttle, 0.0, 1.0) - clamp(controls.m_brake, 0.0, 1.0);
        torque += right() * (m_inertia.z * m_airPitchRate * input);
        torque += m_angularVelocity * (-m_inertia.x * m_airDamping);
    }
}

void Truck::integrate(double dt, const Vec3& force, const Vec3& torque)
{
    m_velocity += force * (dt / m_mass);

    // Euler's equations in the body frame
    const Quat toBody = m_orientation.conjugate();
    Vec3 w = toBody.rotate(m_angularVelocity);
    const Vec3 t = toBody.rotate(torque);
    const Vec3 gyro = cross(w, Vec3(m_inertia.x*w.x, m_inertia.y*w.y, m_inertia.z*w.z));
    w += Vec3((t.x - gyro.x) / m_inertia.x, (t.y - gyro.y) / m_inertia.y, (t.z - gyro.z) / m_inertia.z) * dt;
    m_angularVelocity = m_orientation.rotate(w);

    m_position += m_velocity * dt;
    const Quat spin = Quat(0.0, m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z) * m_orientation;
    m_orientation = Quat(
        m_orientation.w + 0.5*dt*spin.w,
        m_orientation.x + 0.5*dt*spin.x,
        m_orientation.y + 0.5*dt*spin.y,
        m_orientation.z + 0.5*dt*spin.z).normalized();

    require(std::isfinite(m_position.x) && std::isfinite(m_position.y) && std::isfinite(m_position.z),
        "Truck::integrate: position out of bounds. Consider decreasing step size.");
}
