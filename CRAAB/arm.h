#pragma once
#include <array>
#include <vector>
#include <cmath>
#include <stdexcept>

typedef struct Point3D{
	double x{ 0 }, y{ 0 }, z{ 0 };
    
	Point3D(double x_ = 0, double y_ = 0, double z_ = 0)
		: x(x_), y(y_), z(z_) {}
}Point3D;

constexpr auto M_PI = 3.14159265358979323846;


// Position and orientation of a mount point on the robot
struct MountPoint {
    double x{ 0 }, y{ 0 }, z{ 0 };          // Position relative to robot core
    double yaw{ 0 }, pitch{ 0 }, roll{ 0 }; // Orientation in degrees

    MountPoint(double x_ = 0, double y_ = 0, double z_ = 0,
        double yaw_ = 0, double pitch_ = 0, double roll_ = 0)
        : x(x_), y(y_), z(z_), yaw(yaw_), pitch(pitch_), roll(roll_) {}
};


// Parameters for a single arm segment
class ArmParameters {
public:
    ArmParameters(
        double segment1Length = 10.0,
        double segment2Length = 10.0,
        double segment3Length = 10.0,
        double maxVerticalAngle = 90.0,
        double maxHorizontalAngle = 120.0,
        const MountPoint& mount = MountPoint()
    );

    // Getters for segment parameters
    double getSegment1Length() const { return segment1Length_; }
    double getSegment2Length() const { return segment2Length_; }
    double getSegment3Length() const { return segment3Length_; }
    double getMaxVerticalAngle() const { return maxVerticalAngle_; }
    double getMaxHorizontalAngle() const { return maxHorizontalAngle_; }

    // Getter and setter for mount point
    const MountPoint& getMountPoint() const { return mountPoint_; }
    void setMountPoint(const MountPoint& mount) { mountPoint_ = mount; }


private:
    double segment1Length_;
    double segment2Length_;
    double segment3Length_;
    double maxVerticalAngle_;
    double maxHorizontalAngle_;
    MountPoint mountPoint_;
};

class Arm {
public:
    

    explicit Arm(const ArmParameters& params);
    void setJointAngle(size_t jointIndex, double angle);
    const std::array<double, 3>& getJointAngles() const;
    const std::vector<Point3D>& getEndPoints() const;
    std::vector<Point3D> getGlobalEndPoints() const;
	void setMountPoint(const MountPoint& mount);

private:
    ArmParameters parameters_;
    std::array<double, 3> jointAngles_;    // [base_angle, shoulder_angle, elbow_angle]
    std::vector<Point3D> endPoints_;       // Points in arm's local coordinate system
    void updateEndPoints();
    Point3D transformToGlobal(const Point3D& localPoint) const;
};