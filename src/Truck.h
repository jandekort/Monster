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
    Truck();

    // Mutator
    void reset(const Vec3& position);
    void placeAt(const Vec3& position, const Vec3& forward, const Vec3& normal); // upright and at rest
    void step(double dt, const Controls& controls, Terrain& terrain);

    // Accessor
    const Vec3& getPosition() const { return m_position; }
    const Vec3& getVelocity() const { return m_velocity; }
    const Vec3& getAngularVelocity() const { return m_angularVelocity; }
    const Quat& getOrientation() const { return m_orientation; }
    const std::vector<Wheel>& getWheels() const { return m_wheels; }
    const std::vector<Vec3>& getBodyPoints() const { return m_bodyPoints; }
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
    double m_grip;           // friction coefficient
    double m_slipSaturation; // lateral slip speed at which grip saturates [m/s]
    double m_driveForce;     // per wheel [N]
    double m_brakeForce;     // per wheel [N]
    double m_topSpeed;
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
};
