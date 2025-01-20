#include "arm.h"


ArmParameters::ArmParameters(
    double segment1Length,
    double segment2Length,
    double segment3Length,
    double maxVerticalAngle,
    double maxHorizontalAngle,
    const MountPoint& mount
) :
    segment1Length_(segment1Length),
    segment2Length_(segment2Length),
    segment3Length_(segment3Length),
    maxVerticalAngle_(maxVerticalAngle),
    maxHorizontalAngle_(maxHorizontalAngle),
    mountPoint_(mount)
{}

Arm::Arm(const ArmParameters& params)
    : parameters_(params) {
    jointAngles_.fill(0.0);
    endPoints_.resize(4); // Origin + 3 segment endpoints
    updateEndPoints();
}

Point3D Arm::transformToGlobal(const Point3D& localPoint) const {
    const MountPoint& mount = parameters_.getMountPoint();

    // Convert to radians
    double yawRad = mount.yaw * M_PI / 180.0;
    double pitchRad = mount.pitch * M_PI / 180.0;
    double rollRad = mount.roll * M_PI / 180.0;

    // Apply rotation transformations (ZYX rotation sequence)
    double x = localPoint.x;
    double y = localPoint.y;
    double z = localPoint.z;

    // Yaw rotation (around Z)
    double x1 = x * cos(yawRad) - y * sin(yawRad);
    double y1 = x * sin(yawRad) + y * cos(yawRad);
    double z1 = z;

    // Pitch rotation (around Y)
    double x2 = x1 * cos(pitchRad) + z1 * sin(pitchRad);
    double y2 = y1;
    double z2 = -x1 * sin(pitchRad) + z1 * cos(pitchRad);

    // Roll rotation (around X)
    double x3 = x2;
    double y3 = y2 * cos(rollRad) - z2 * sin(rollRad);
    double z3 = y2 * sin(rollRad) + z2 * cos(rollRad);

    // Add mount point offset
    return Point3D(
        x3 + mount.x,
        y3 + mount.y,
        z3 + mount.z
    );
}

std::vector<Point3D> Arm::getGlobalEndPoints() const {
    std::vector<Point3D> globalPoints;
    globalPoints.reserve(endPoints_.size());

    for (const auto& point : endPoints_) {
        globalPoints.push_back(transformToGlobal(point));
    }

    return globalPoints;
}

void Arm::updateEndPoints() {
    // Calculate points in arm's local coordinate system
    endPoints_[0] = Point3D(0, 0, 0);

    double baseAngle = jointAngles_[0];
    double shoulderAngle = jointAngles_[1];
    double elbowAngle = jointAngles_[2];

    // First segment
    double xy_proj = parameters_.getSegment1Length() * cos(shoulderAngle);
    endPoints_[1] = Point3D(
        xy_proj * cos(baseAngle),
        xy_proj * sin(baseAngle),
        parameters_.getSegment1Length() * sin(shoulderAngle)
    );

    // Second segment
    double seg2_xy = parameters_.getSegment2Length() * cos(shoulderAngle + elbowAngle);
    double x2 = endPoints_[1].x + seg2_xy * cos(baseAngle);
    double y2 = endPoints_[1].y + seg2_xy * sin(baseAngle);
    double z2 = endPoints_[1].z + parameters_.getSegment2Length() * sin(shoulderAngle + elbowAngle);
    endPoints_[2] = Point3D(x2, y2, z2);

    // Third segment
    double seg3_xy = parameters_.getSegment3Length() * cos(shoulderAngle + elbowAngle);
    double x3 = endPoints_[2].x + seg3_xy * cos(baseAngle);
    double y3 = endPoints_[2].y + seg3_xy * sin(baseAngle);
    double z3 = endPoints_[2].z + parameters_.getSegment3Length() * sin(shoulderAngle + elbowAngle);
    endPoints_[3] = Point3D(x3, y3, z3);
}


// Set the mount point for the arm
void Arm::setMountPoint(const MountPoint& mount) {
	parameters_.setMountPoint(mount);
	updateEndPoints();
}

// Set the angle of a joint 0-2 (base, shoulder, elbow), +/- 90 degrees
void Arm::setJointAngle(size_t jointIndex, double angle) {
	if (jointIndex >= jointAngles_.size()) {
		throw std::out_of_range("Joint index out of range");
	}

	jointAngles_[jointIndex] = angle;
	updateEndPoints();
}

const std::array<double, 3>& Arm::getJointAngles() const {
	return jointAngles_;
}

const std::vector<Point3D>& Arm::getEndPoints() const {
	return endPoints_;
}

