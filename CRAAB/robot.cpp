#define NUM_LEGS 6

#include "robot.h"
#include "arm.h"
#include <stdexcept>



Robot::Robot(const ArmParameters& params) {
    // Initialize 6 arms with the given parameters
    arms_.reserve(NUM_LEGS);
    for (size_t i = 0; i < NUM_LEGS; ++i) {
        arms_.emplace_back(params);
    }
}

void Robot::initializeArmPositions(double bodyLength, double bodyWidth,double headWidth, double mountAngle) {
    // Half dimensions for easier calculations
    double halfLength = bodyLength / 2.0;
    double halfbodyWidth = bodyWidth / 2.0;
	double halfheadWidth = headWidth / 2.0;

    // Front legs positions and orientations
    MountPoint frontLeft(
        halfLength,     // x (front)
        halfheadWidth,  // y (left)
        0,              // z
        mountAngle,     // yaw (pointing outward and forward)
        0,              // pitch
        0               // roll
    );

    MountPoint frontRight(
        halfLength,     // x (front)
        -halfheadWidth, // y (right)
        0,              // z
        -mountAngle,    // yaw (mirror of left)
        0,              // pitch
        0               // roll
    );

    // Middle legs positions and orientations
    MountPoint middleLeft(
        0,              // x (center)
        halfbodyWidth,  // y (left)
        0,              // z
        90,             // yaw (pointing directly outward)
        0,              // pitch
        0               // roll
    );

    MountPoint middleRight(
        0,              // x (center)
        -halfbodyWidth, // y (right)
        0,              // z
        -90,            // yaw (pointing directly outward)
        0,              // pitch
        0               // roll
    );

    // Back legs positions and orientations
    MountPoint backLeft(
        -halfLength,      // x (back)
        halfheadWidth,    // y (left)
        0,                // z
        180 - mountAngle, // yaw (pointing outward and backward)
        0,                // pitch
        0                 // roll
    );

    MountPoint backRight(
        -halfLength,         // x (back)
        -halfheadWidth,      // y (right)
        0,                   // z
        -(180 - mountAngle), // yaw (mirror of left)
        0,                   // pitch
        0                    // roll
    );

    // Set mount points for all legs
    arms_[FRONT_LEFT].setMountPoint(frontLeft);
    arms_[FRONT_RIGHT].setMountPoint(frontRight);
    arms_[MIDDLE_LEFT].setMountPoint(middleLeft);
    arms_[MIDDLE_RIGHT].setMountPoint(middleRight);
    arms_[BACK_LEFT].setMountPoint(backLeft);
    arms_[BACK_RIGHT].setMountPoint(backRight);
}

Arm& Robot::getArm(size_t index) {
    if (index >= arms_.size()) {
        throw std::out_of_range("Arm index out of range");
    }
    return arms_[index];
}
/*
const Arm& Robot::getArm(size_t index) const {
    if (index >= arms_.size()) {
        throw std::out_of_range("Arm index out of range");
    }
    return arms_[index];
}
*/

void Robot::setDefaultStance() {
    // Set default angles for standing position
    for (size_t i = 0; i < NUM_LEGS; ++i) {
        Arm& arm = arms_[i];
        // Base angle remains as set by mount point
        // Shoulder and elbow angles for standing
		arm.setJointAngle(0, 0.0);            // Base reset orientation
        arm.setJointAngle(1, -20.0);          // Shoulder lift
        arm.setJointAngle(2, -70.0);          // Elbow bend
    }
}

void Robot::moveForward(double distance) {
    // Modified to use tripod gait with symmetric legs
    // Group 1: FRONT_LEFT, MIDDLE_RIGHT, BACK_LEFT
    // Group 2: FRONT_RIGHT, MIDDLE_LEFT, BACK_RIGHT

    // First tripod group
    std::vector<size_t> group1 = { FRONT_LEFT, MIDDLE_RIGHT, BACK_LEFT };
    for (size_t index : group1) {
        Arm& arm = arms_[index];
        // Lift leg
        arm.setJointAngle(1, 60.0);
        // Move forward
        double currentBase = arm.getJointAngles()[0] * 180.0 / M_PI;
        arm.setJointAngle(0, currentBase + 15.0);
    }

    // Second tripod group
    std::vector<size_t> group2 = { FRONT_RIGHT, MIDDLE_LEFT, BACK_RIGHT };
    for (size_t index : group2) {
        Arm& arm = arms_[index];
        // Lift leg
        arm.setJointAngle(1, 60.0);
        // Move forward
        double currentBase = arm.getJointAngles()[0] * 180.0 / M_PI;
        arm.setJointAngle(0, currentBase + 15.0);
    }
}

void Robot::setPose(
    double A00, double A01, double A02,
    double A10, double A11, double A12,
    double A20, double A21, double A22,
    double A30, double A31, double A32,
    double A40, double A41, double A42,
    double A50, double A51, double A52) {
	
    Arm& frontLeft = arms_[FRONT_LEFT];
	Arm& frontRight = arms_[FRONT_RIGHT];
	Arm& middleLeft = arms_[MIDDLE_LEFT];
	Arm& middleRight = arms_[MIDDLE_RIGHT];
	Arm& backLeft = arms_[BACK_LEFT];
	Arm& backRight = arms_[BACK_RIGHT];

	frontLeft.setJointAngle(0, A00);
	frontLeft.setJointAngle(1, A01);
	frontLeft.setJointAngle(2, A02);

	frontRight.setJointAngle(0, A10);
	frontRight.setJointAngle(1, A11);
	frontRight.setJointAngle(2, A12);

	middleLeft.setJointAngle(0, A20);
	middleLeft.setJointAngle(1, A21);
	middleLeft.setJointAngle(2, A22);

	middleRight.setJointAngle(0, A30);
	middleRight.setJointAngle(1, A31);
	middleRight.setJointAngle(2, A32);

	backLeft.setJointAngle(0, A40);
	backLeft.setJointAngle(1, A41);
	backLeft.setJointAngle(2, A42);

	backRight.setJointAngle(0, A50);
	backRight.setJointAngle(1, A51);
	backRight.setJointAngle(2, A52);
}