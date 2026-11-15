// Monster - rigid body monster truck with four sprung wheels
// Author: Jan de Kort
// Created: 2026-11-14
#include "Truck.h"

#include "Check.h"
#include "Terrain.h"

#include <algorithm>
#include <cmath>

namespace
{
    // Struct NamedSetup: one entry of the tyre range offered to the player
    struct NamedSetup
    {
        const char* m_name;
        Truck::Setup m_setup;
    };

    //                                         grip    top    slip
    const NamedSetup g_setups[] = {
        { "SOFT",        { 1.18,  34.0,  0.75 } },
        { "MEDIUM SOFT", { 1.14,  37.0,  0.82 } },
        { "STANDARD",    { 1.10,  40.0,  0.90 } },
        { "MEDIUM HARD", { 0.98,  44.0,  1.05 } },
        { "HARD",        { 0.85,  48.0,  1.25 } }
    };
}

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
    , m_hull()
    , m_pushForce()
    , m_pushTorque()
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
    , m_setup(setupAt(3))
    , m_driveForce(8000.0)
    , m_brakeForce(9000.0)
    , m_reverseSpeed(12.0)
    , m_rollingDrag(40.0)
    , m_wadeDrag(2500.0)
    , m_maxSteer(0.5)
    , m_steerRate(2.5)
    , m_steerFadeSpeed(15.0)
    , m_bodyStiffness(200000.0)
    , m_bodyDamping(15000.0)
    , m_bodyFriction(0.6)
    , m_airPitchRate(0.7)
    , m_airDamping(0.3)
    , m_hullRadius(1.25)
    , m_hullStiffness(600000.0)
    , m_hullDamping(30000.0)
    , m_maxPush(400000.0)
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

    { // Hull against other trucks: two rows of three spheres, out to the tyres
        m_hull.reserve(6);
        m_hull.push_back(Vec3( 1.6, 0.2, -1.0));
        m_hull.push_back(Vec3( 1.6, 0.2,  1.0));
        m_hull.push_back(Vec3( 0.0, 0.2, -1.0));
        m_hull.push_back(Vec3( 0.0, 0.2,  1.0));
        m_hull.push_back(Vec3(-1.6, 0.2, -1.0));
        m_hull.push_back(Vec3(-1.6, 0.2,  1.0));
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

int Truck::setupCount()
{
    return (int) (sizeof(g_setups) / sizeof(g_setups[0]));
}

Truck::Setup Truck::setupAt(int index)
{
    require(index >= 1 && index <= setupCount(), "Truck::setupAt: no such tyre setup");
    return g_setups[index - 1].m_setup;
}

const char* Truck::setupName(int index)
{
    require(index >= 1 && index <= setupCount(), "Truck::setupName: no such tyre setup");
    return g_setups[index - 1].m_name;
}

double Truck::minGrip()
{
    return 0.60;
}

// Measured: a full lock corner above roughly 1.22 trips the truck instead of gripping
double Truck::maxGrip()
{
    return 1.25;
}

void Truck::setSetup(const Setup& setup)
{
    m_setup = setup;
    m_setup.m_grip = clamp(setup.m_grip, minGrip(), maxGrip());
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
    m_pushForce = Vec3();
    m_pushTorque = Vec3();
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

    Vec3 force = Vec3(0.0, -m_mass * m_gravity, 0.0) + m_pushForce;
    Vec3 torque = m_pushTorque;
    m_pushForce = Vec3();
    m_pushTorque = Vec3();

    steer(dt, controls);
    wheelForces(dt, controls, terrain, force, torque);
    bodyForces(terrain, force, torque);
    airControl(controls, torque);
    integrate(dt, force, torque);
}

double Truck::overlap(const Truck& a, const Truck& b)
{
    double deepest = 0.0;
    for (const Vec3& pa : a.m_hull)
    {
        for (const Vec3& pb : b.m_hull)
        {
            deepest = std::max(deepest,
                a.m_hullRadius + b.m_hullRadius - length(b.toWorld(pb) - a.toWorld(pa)));
        }
    }
    return deepest;
}

// Each pair of overlapping spheres pushes the two trucks apart with a spring along the line
// between them and damping on their closing speed. The push is kept level, so a bump can shove a
// truck aside or turn it, but can neither lift it off the road nor press it into it.
void Truck::collide(Truck& a, Truck& b)
{
    const double reach = 2.0 * (a.m_hullRadius + b.m_hullRadius) + 4.0;
    if (length(b.m_position - a.m_position) > reach)
    {
        return;
    }

    const Vec3 apart = b.m_position - a.m_position;
    const Vec3 fallback = normalize(Vec3(apart.x, 0.0, apart.z), Vec3(1.0, 0.0, 0.0));
    for (const Vec3& pa : a.m_hull)
    {
        const Vec3 ca = a.toWorld(pa);
        for (const Vec3& pb : b.m_hull)
        {
            const Vec3 cb = b.toWorld(pb);
            const Vec3 between = cb - ca;
            const double depth = a.m_hullRadius + b.m_hullRadius - length(between);
            if (depth <= 0.0)
            {
                continue;
            }

            const Vec3 normal = normalize(Vec3(between.x, 0.0, between.z), fallback); // from a to b
            const Vec3 point = (ca + cb) * 0.5;
            const Vec3 va = a.m_velocity + cross(a.m_angularVelocity, point - a.m_position);
            const Vec3 vb = b.m_velocity + cross(b.m_angularVelocity, point - b.m_position);
            const double closing = dot(va - vb, normal);
            const double load = clamp(a.m_hullStiffness*depth + a.m_hullDamping*closing, 0.0, a.m_maxPush);
            b.push(normal * load, point);
            a.push(normal * -load, point);
        }
    }
}

// Only the turning part of the torque is kept: a bump must not roll or pitch the truck
void Truck::push(const Vec3& force, const Vec3& point)
{
    m_pushForce += force;
    m_pushTorque.y += cross(point - m_position, force).y;
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
        int onWreck = -1;
        double traction = 1.0;
        double speedLimit = 0.0;
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
                traction = hit.m_traction;
                speedLimit = hit.m_speedLimit;
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

            // Water holds a wheel back: it drives up to the speed the water allows only, and
            // whatever speed it arrives with above that is dragged off
            const bool held = speedLimit > 0.0;
            const double topSpeed = held ? std::min(m_setup.m_topSpeed, speedLimit) : m_setup.m_topSpeed;
            double drive = throttle * m_driveForce * traction * std::max(0.0, 1.0 - rollSpeed / topSpeed);
            if (held && std::fabs(rollSpeed) > topSpeed)
            {
                drive -= m_wadeDrag * (rollSpeed > 0.0 ? rollSpeed - topSpeed : rollSpeed + topSpeed);
            }
            if (brake > 0.0)
            {
                if (rollSpeed > 0.5)
                {
                    drive -= brake * m_brakeForce * traction;
                }
                else
                {
                    drive -= brake * 0.6 * m_driveForce * traction * std::max(0.0, 1.0 + rollSpeed / m_reverseSpeed);
                }
            }
            drive -= m_rollingDrag / traction * rollSpeed;
            const double grip = m_setup.m_grip * traction;
            double side = -grip * load * clamp(slipSpeed / m_setup.m_slipSaturation, -1.0, 1.0);

            const double limit = grip * load;
            const double demand = std::sqrt(drive*drive + side*side);
            if (demand > limit)
            {
                drive *= limit / demand;
                side *= limit / demand;
            }

            const Vec3 total = normal*load + along*drive + across*side;
            force += total;
            torque += cross(arm, total);
            if (onWreck >= 0)
            {
                terrain.crush((size_t) onWreck, load, dt);
            }
        }

        // Wheel state for rendering
        if (wheel.m_onGround)
        {
            wheel.m_spinRate = rollSpeed / m_wheelRadius;
        }
        else
        {
            const double freeRate = (throttle - brake) * m_setup.m_topSpeed / m_wheelRadius;
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
        double depth = std::max(0.0, terrain.groundFloor(m_position + arm) - (m_position.y + arm.y));

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
