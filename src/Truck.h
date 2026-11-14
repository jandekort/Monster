// Monster - rigid body monster truck with four sprung wheels
// Author: Jan de Kort
// Created: 2026-11-14
#pragma once

#include "Math.h"

#include <vector>

class Terrain;

// Struct Controls
struct Controls
{
    double m_throttle = 0.0; // 0..1
    double m_brake = 0.0;    // 0..1, reverses when (almost) standing still
    double m_steer = 0.0;    // -1->full left, +1->full right
};

// Struct Wheel
struct Wheel
{
    Vec3 m_attach;              // suspension mount in body coordinates; wheels are stored in axle pairs
    bool m_steered = false;     // true->front axle, false->rear axle
    bool m_onGround = false;
    double m_compression = 0.0; // suspension compression [m]
    double m_spin = 0.0;        // rolling angle [rad]
    double m_spinRate = 0.0;    // [rad/s]
    Vec3 m_center;              // world position of the hub
    Vec3 m_axle;                // world direction of the axle (points right)

    Wheel(const Vec3& attach, bool steered);
};

// Class Truck
// Body frame: x forward, y up, z right. The origin is the centre of mass.
class Truck
{
public:
    // Struct Setup: how the tyres are set up for a run. A softer compound grips harder and
    // reaches full grip at a smaller slip, but drags more, so the truck runs slower.
    struct Setup
    {
        double m_grip = 1.10;
        double m_topSpeed = 40.0;      // [m/s]
        double m_slipSaturation = 0.9; // lateral slip speed at which the tyre reaches full grip [m/s]
    };

    Truck();

    // Tyre setups the player can choose between, 1 is the softest
    static int setupCount();
    static Setup setupAt(int index);
    static const char* setupName(int index);
    static double minGrip();
    static double maxGrip();

    // Mutator
    void setSetup(const Setup& setup); // grip is clamped to what the truck can hold

    // Bumps two trucks apart where their hulls overlap. The pushes act in the next step().
    static void collide(Truck& a, Truck& b);
    void reset(const Vec3& position);
    void placeAt(const Vec3& position, const Vec3& forward, const Vec3& normal); // upright and at rest
    void step(double dt, const Controls& controls, Terrain& terrain);

    // Accessor
    const Vec3& getPosition() const { return m_position; }
    const Vec3& getVelocity() const { return m_velocity; }
    const Vec3& getAngularVelocity() const { return m_angularVelocity; }
    const Quat& getOrientation() const { return m_orientation; }
    const Setup& getSetup() const { return m_setup; }
    const std::vector<Wheel>& getWheels() const { return m_wheels; }
    const std::vector<Vec3>& getBodyPoints() const { return m_bodyPoints; }
    const std::vector<Vec3>& getHull() const { return m_hull; } // sphere centres in body coordinates
    double getHullRadius() const { return m_hullRadius; }
    static double overlap(const Truck& a, const Truck& b);      // deepest hull overlap [m], 0->apart
    const Vec3& getContactNormal() const { return m_contactNormal; } // surface under the last wheel contact
    int getWheelsOnGround() const { return m_wheelsOnGround; }
    bool hasBodyContact() const { return m_bodyContact; }
    double getWheelRadius() const { return m_wheelRadius; }
    double getWheelHalfWidth() const { return m_wheelHalfWidth; }
    double getSpeed() const { return length(m_velocity); }
    double getRideHeight() const; // centre of mass above flat ground with the suspension extended

    Vec3 forward() const { return m_orientation.rotate(Vec3(1.0, 0.0, 0.0)); }
    Vec3 up() const { return m_orientation.rotate(Vec3(0.0, 1.0, 0.0)); }
    Vec3 right() const { return m_orientation.rotate(Vec3(0.0, 0.0, 1.0)); }
    Vec3 toWorld(const Vec3& local) const { return m_position + m_orientation.rotate(local); }

private:
    void steer(double dt, const Controls& controls);
    void wheelForces(double dt, const Controls& controls, Terrain& terrain, Vec3& force, Vec3& torque);
    void bodyForces(const Terrain& terrain, Vec3& force, Vec3& torque);
    void airControl(const Controls& controls, Vec3& torque) const;
    void integrate(double dt, const Vec3& force, const Vec3& torque);
    void push(const Vec3& force, const Vec3& point);

    // State
    Vec3 m_position;
    Vec3 m_velocity;
    Vec3 m_angularVelocity; // world coordinates
    Quat m_orientation;
    double m_steerAngle;    // positive->right
    int m_wheelsOnGround;
    bool m_bodyContact;     // true->chassis or roof touches the terrain
    Vec3 m_contactNormal;
    std::vector<Wheel> m_wheels;
    std::vector<Vec3> m_bodyPoints; // collision points in body coordinates
    std::vector<Vec3> m_hull;       // spheres standing in for the truck against other trucks
    Vec3 m_pushForce;               // from other trucks, spent in the next step
    Vec3 m_pushTorque;

    // Chassis
    double m_mass;
    Vec3 m_inertia; // principal moments: x->roll, y->yaw, z->pitch
    double m_gravity;

    // Suspension
    double m_wheelRadius;
    double m_wheelHalfWidth;
    double m_suspensionLength; // mount to hub, fully extended
    double m_suspensionTravel;
    double m_springRate;
    double m_damperRate;
    double m_antiRollRate;     // couples the two wheels of an axle [N/m]
    double m_bumpStopRate;
    double m_maxWheelLoad;
    int m_wheelSpokes;         // contact rays fanned over the lower part of the tyre
    double m_spokeSpread;      // half angle of the fan [rad]
    double m_maxCompression;   // beyond the travel, onto the bump stop [m]
    double m_rayMargin;        // extra ray length above the tyre [m]

    // Tyres and drivetrain
    Setup m_setup;
    double m_driveForce;     // per wheel [N]
    double m_brakeForce;     // per wheel [N]
    double m_reverseSpeed;
    double m_rollingDrag;    // per wheel [N s/m]
    double m_maxSteer;
    double m_steerRate;
    double m_steerFadeSpeed; // speed at which the steering lock halves

    // Body contact and air control
    double m_bodyStiffness;
    double m_bodyDamping;
    double m_bodyFriction;
    double m_airPitchRate;   // angular acceleration from throttle/brake in the air [rad/s^2]
    double m_airDamping;     // [1/s]

    // Contact with other trucks
    double m_hullRadius;
    double m_hullStiffness;  // [N/m] per pair of overlapping spheres
    double m_hullDamping;    // [N s/m]
    double m_maxPush;        // [N] per pair of overlapping spheres
};
