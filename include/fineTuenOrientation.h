#ifndef FINETUNEORIENTATION_H
#define FINETUNEORIENTATION_H

#include <array>

// Inline definition of the fine-tuning function
inline std::array<double, 3> fineTuneOrientation(const std::array<double, 6>& joint_angles, const std::array<double, 3>& euler) {
    std::array<double, 3> fine_tuned_euler = euler;

    // Define offsets for fine-tuning
    const double rx_offset = -10.6519; // Offset to make rx = 0 at zero-pose
    const double ry_offset = 0.0;      // No offset for ry
    const double rz_offset = 0.0;      // No offset for rz

    // Apply offsets
    fine_tuned_euler[0] += rx_offset; // Adjust roll (rx)
    fine_tuned_euler[1] += ry_offset; // Adjust pitch (ry)
    fine_tuned_euler[2] += rz_offset; // Adjust yaw (rz)

    // Handle specific configurations
    if (joint_angles[0] == M_PI / 2 && joint_angles[1] == 0 && joint_angles[2] == 0 &&
        joint_angles[3] == 0 && joint_angles[4] == 0 && joint_angles[5] == 0) {
        // Joint 1 at 90 degrees: Ensure rz = 90°
        fine_tuned_euler[2] = 90.0;
    }

    return fine_tuned_euler;
}

#endif // FINETUNEORIENTATION_H