#pragma once
#include "arm.h"
#include <vector>

struct Configuration {
	ArmParameters armParams;
	double bodyLength = 93.301;
	double bodyWidth = 100.0;
	double headWidth = 75.0;
	double mountAngle = 30.0;

    double segment1Length = 10.0;
    double segment2Length = 10.0;
    double segment3Length = 10.0;
    double maxVerticalAngle = 90.0;
    double maxHorizontalAngle = 90.0;


};



class Robot {
public:
    // Enum to identify leg positions
    enum LegPosition {
        FRONT_LEFT = 0,
        FRONT_RIGHT = 1,
        MIDDLE_LEFT = 2,
        MIDDLE_RIGHT = 3,
        BACK_LEFT = 4,
        BACK_RIGHT = 5
    };

    Robot(const ArmParameters& params = ArmParameters());
    Arm& getArm(size_t index);
    const Arm& getArm(size_t index) const;
    void updateAllArms();
    void setDefaultStance();
    void moveForward(double distance);
    void moveSideways(double distance);
    void rotate(double degrees);
	void start();

    // Initialize bilateral symmetric arm positions
    void initializeArmPositions(
        double bodyLength,    // Length of robot body
        double bodyWidth,     // Width of robot body in the middle
		double headWidth,     // Distance from body center to front and backleg mount
        double mountAngle     // Angle of leg mount relative to body (degrees)
    );

	// set every joint to a differnt angle
    void setPose(
        double A00, double A01, double A02,
        double A10, double A11, double A12,
        double A20, double A21, double A22,
        double A30, double A31, double A32,
        double A40, double A41, double A42,
        double A50, double A51, double A52);

private:
    std::vector<Arm> arms_;
};