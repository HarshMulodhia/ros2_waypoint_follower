#include "maths/transform.h"\
#include<iomanip>
#include<iostream>

using namespace robotics_control;
using namespace robotics_control::constants;

// Helper function to print headers
void PrintSection(const std::string& title) {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << title << "\n";
    std::cout << std::string(70, '=') << "\n";
}

// ============================================================================
// Example 1: Quaternion Basics
// ============================================================================
void Example1_QuaternionBasics() {
    PrintSection("Example 1: Quaternion Basics");
    
    std::cout << "\n1a. Create identity quaternion:\n";
    Quaternion q_identity;
    std::cout << q_identity.ToString() << "\n";
    std::cout << "Is identity: " << (q_identity.IsIdentity() ? "YES" : "NO") << "\n";
    
    std::cout << "\n1b. Create quaternion from components:\n";
    Quaternion q(0.707, 0.707, 0, 0);  // 90° rotation around X axis
    std::cout << q.ToString() << "\n";
    
    std::cout << "\n1c. Create quaternion from axis-angle:\n";
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();  // Z axis
    double angle = PI / 4;  // 45 degrees
    Quaternion q_aa = Quaternion::FromAxisAngle(axis, angle);
    std::cout << "Axis: [0, 0, 1], Angle: 45°\n";
    std::cout << q_aa.ToString() << "\n";
    
    std::cout << "\n1d. Get back axis-angle representation:\n";
    Eigen::Vector3d axis_out;
    double angle_out;
    q_aa.ToAxisAngle(axis_out, angle_out);
    std::cout << "Reconstructed Axis: [" << axis_out(0) << ", " 
              << axis_out(1) << ", " << axis_out(2) << "]\n";
    std::cout << "Reconstructed Angle: " << RadiansToDegrees(angle_out) << "°\n";
}

// ============================================================================
// Example 2: Rotation Matrix Creation and Validation
// ============================================================================
void Example2_RotationMatrix() {
    PrintSection("Example 2: Rotation Matrix Creation");
    
    std::cout << "\n2a. Create rotation around Z by 90°:\n";
    RotationMatrix R = RotationMatrix::RotZ(PI / 2);
    std::cout << "R =\n" << R.MatrixRef() << "\n";
    
    std::cout << "Is orthogonal: " << (R.IsOrthogonal() ? "YES" : "NO") << "\n";
    std::cout << "Is proper rotation (det=1): " << (R.IsProperRotation() ? "YES" : "NO") << "\n";
    
    std::cout << "\n2b. Compose two rotations (90° Z + 45° X):\n";
    RotationMatrix R_z = RotationMatrix::RotZ(PI / 2);
    RotationMatrix R_x = RotationMatrix::RotX(PI / 4);
    RotationMatrix R_composed = R_z * R_x;
    
    std::cout << "Composed rotation properties:\n";
    std::cout << "- Orthogonal: " << (R_composed.IsOrthogonal() ? "YES" : "NO") << "\n";
    std::cout << "- Proper: " << (R_composed.IsProperRotation() ? "YES" : "NO") << "\n";
    std::cout << "- Determinant: " << R_composed.MatrixRef().determinant() << "\n";
}

// ============================================================================
// Example 3: Point Transformation
// ============================================================================
void Example3_PointTransformation() {
    PrintSection("Example 3: Point Transformation");
    
    // Create a rotation of 90° around Z axis
    RotationMatrix R = RotationMatrix::RotZ(PI / 2);
    
    // Point to rotate
    Eigen::Vector3d p(1, 0, 0);  // Point on X axis
    
    std::cout << "\n3a. Rotate point [1, 0, 0] by 90° around Z:\n";
    Eigen::Vector3d p_rotated = R * p;
    std::cout << "Original point: [" << p(0) << ", " << p(1) << ", " << p(2) << "]\n";
    std::cout << "Rotated point:  [" << std::fixed << std::setprecision(6)
              << p_rotated(0) << ", " << p_rotated(1) << ", " << p_rotated(2) << "]\n";
    std::cout << "Expected (approx): [0, 1, 0]\n";
    
    std::cout << "\n3b. Verify rotation properties:\n";
    std::cout << "- Distance from origin preserved: " << (std::abs(p.norm() - p_rotated.norm()) < 1e-10 ? "YES" : "NO") << "\n";
    std::cout << "- Original distance: " << p.norm() << "\n";
    std::cout << "- Rotated distance:  " << p_rotated.norm() << "\n";
}

// ============================================================================
// Example 4: Euler Angles and Gimbal Lock
// ============================================================================
void Example4_EulerAngles() {
    PrintSection("Example 4: Euler Angles and Conversions");
    
    std::cout << "\n4a. Create Euler angles (roll=30°, pitch=20°, yaw=45°):\n";
    double roll = DegreesToRadians(30);
    double pitch = DegreesToRadians(20);
    double yaw = DegreesToRadians(45);
    
    EulerAngles euler(roll, pitch, yaw);
    std::cout << euler.ToString() << "\n";
    std::cout << "Has gimbal lock: " << (euler.HasGimbalLock() ? "YES" : "NO") << "\n";
    
    std::cout << "\n4b. Convert to rotation matrix and back:\n";
    RotationMatrix R = euler.ToRotationMatrix();
    EulerAngles euler_reconstructed(R);
    std::cout << "Original:      " << euler.ToString() << "\n";
    std::cout << "Reconstructed: " << euler_reconstructed.ToString() << "\n";
    
    std::cout << "\n4c. Example of gimbal lock (pitch = 90°):\n";
    EulerAngles euler_gimbal(0, PI/2, 0);
    std::cout << euler_gimbal.ToString() << "\n";
    std::cout << "Note: At gimbal lock, you lose one degree of freedom\n";
}

// ============================================================================
// Example 5: Quaternion Interpolation (SLERP)
// ============================================================================
void Example5_QuaternionInterpolation() {
    PrintSection("Example 5: Quaternion Interpolation (SLERP)");
    
    std::cout << "\n5a. Create two quaternions:\n";
    Quaternion q1 = Quaternion::FromAxisAngle(Eigen::Vector3d::UnitZ(), 0);
    Quaternion q2 = Quaternion::FromAxisAngle(Eigen::Vector3d::UnitZ(), PI/2);
    
    std::cout << "q1 (0° rotation):  " << q1.ToString() << "\n";
    std::cout << "q2 (90° rotation): " << q2.ToString() << "\n";
    
    std::cout << "\n5b. Interpolate between them:\n";
    for (double t = 0; t <= 1.0; t += 0.25) {
        Quaternion q_interp = q1.Slerp(q2, t);
        Eigen::Vector3d axis;
        double angle;
        q_interp.ToAxisAngle(axis, angle);
        std::cout << "t=" << std::fixed << std::setprecision(2) << t 
                  << " -> angle=" << RadiansToDegrees(angle) << "°\n";
    }
    
    std::cout << "\n5c. Note: SLERP gives smooth, constant-velocity interpolation\n";
}

// ============================================================================
// Example 6: 3D Transformations (SE(3))
// ============================================================================
void Example6_3DTransforms() {
    PrintSection("Example 6: 3D Rigid Transformations (SE(3))");
    
    std::cout << "\n6a. Create a transformation (rotation + translation):\n";
    RotationMatrix R = RotationMatrix::RotZ(PI / 4);
    Eigen::Vector3d t(1, 2, 3);
    Transform T(R, t);
    
    std::cout << T.ToString() << "\n";
    
    std::cout << "\n6b. Transform a point:\n";
    Eigen::Vector3d p(1, 0, 0);
    Eigen::Vector3d p_transformed = T.TransformPoint(p);
    
    std::cout << "Original point: [" << p(0) << ", " << p(1) << ", " << p(2) << "]\n";
    std::cout << "Transformed point: [" << std::fixed << std::setprecision(4)
              << p_transformed(0) << ", " << p_transformed(1) << ", " 
              << p_transformed(2) << "]\n";
    
    std::cout << "\n6c. Inverse transformation:\n";
    Transform T_inv = T.Inverse();
    Eigen::Vector3d p_back = T_inv.TransformPoint(p_transformed);
    
    std::cout << "After inverse transform: [" << std::fixed << std::setprecision(6)
              << p_back(0) << ", " << p_back(1) << ", " << p_back(2) << "]\n";
    std::cout << "Difference from original: " << (p - p_back).norm() << "\n";
}

// ============================================================================
// Example 7: Transform Composition
// ============================================================================
void Example7_TransformComposition() {
    PrintSection("Example 7: Composing Transformations");
    
    std::cout << "\n7a. Create two transformations:\n";
    
    // First transform: rotate 90° around Z, translate by (1, 0, 0)
    Transform T1(RotationMatrix::RotZ(PI/2), Eigen::Vector3d(1, 0, 0));
    
    // Second transform: rotate 45° around X, translate by (0, 1, 0)
    Transform T2(RotationMatrix::RotX(PI/4), Eigen::Vector3d(0, 1, 0));
    
    std::cout << "T1 = Rotation(90° Z) + Translation(1, 0, 0)\n";
    std::cout << "T2 = Rotation(45° X) + Translation(0, 1, 0)\n";
    
    std::cout << "\n7b. Compose transformations:\n";
    Transform T_composed = T1 * T2;
    std::cout << T_composed.ToString() << "\n";
    
    std::cout << "\n7c. Apply composed transformation to a point:\n";
    Eigen::Vector3d p(1, 0, 0);
    
    // Method 1: Apply T2 then T1
    Eigen::Vector3d p_two_steps = T1.TransformPoint(T2.TransformPoint(p));
    
    // Method 2: Apply composed transformation
    Eigen::Vector3d p_composed = T_composed.TransformPoint(p);
    
    std::cout << "Result from two-step: [" << std::fixed << std::setprecision(6)
              << p_two_steps(0) << ", " << p_two_steps(1) << ", " 
              << p_two_steps(2) << "]\n";
    std::cout << "Result from composed: [" << p_composed(0) << ", " 
              << p_composed(1) << ", " << p_composed(2) << "]\n";
    std::cout << "Difference: " << (p_two_steps - p_composed).norm() << "\n";
}

// ============================================================================
// Example 8: Transform Interpolation
// ============================================================================
void Example8_TransformInterpolation() {
    PrintSection("Example 8: Transform Interpolation");
    
    std::cout << "\n8a. Create start and end transforms:\n";
    Transform T_start(RotationMatrix::RotZ(0), Eigen::Vector3d(0, 0, 0));
    Transform T_end(RotationMatrix::RotZ(PI/2), Eigen::Vector3d(2, 0, 0));
    
    std::cout << "Start: Identity transform\n";
    std::cout << "End:   90° rotation around Z + 2 units in X\n";
    
    std::cout << "\n8b. Interpolate between transforms:\n";
    std::cout << "t=0.0 -> Position: ";
    std::cout << T_start.GetTranslation().transpose() << "\n";
    
    for (double t = 0.25; t <= 0.75; t += 0.25) {
        Transform T_interp = T_start.Lerp(T_end, t);
        Eigen::Vector3d pos = T_interp.GetTranslation();
        std::cout << "t=" << std::fixed << std::setprecision(2) << t 
                  << " -> Position: [" << pos(0) << ", " << pos(1) << ", " 
                  << pos(2) << "]\n";
    }
    
    std::cout << "t=1.0 -> Position: ";
    std::cout << T_end.GetTranslation().transpose() << "\n";
    
    std::cout << "\nNote: Uses SLERP for rotation (smooth) and linear for translation\n";
}

// ============================================================================
// Example 9: Robot Kinematics (Simplified)
// ============================================================================
void Example9_RobotKinematics() {
    PrintSection("Example 9: Simplified Robot Kinematics");
    
    std::cout << "\n9a. Two-link robot arm:\n";
    std::cout << "- Joint 1: Rotation around Z by theta1\n";
    std::cout << "- Link 1: Length 1 unit along X\n";
    std::cout << "- Joint 2: Rotation around Z by theta2\n";
    std::cout << "- Link 2: Length 1 unit along X\n";
    
    double theta1 = PI / 4;   // 45°
    double theta2 = PI / 6;   // 30°
    
    std::cout << "\nFor theta1=" << RadiansToDegrees(theta1) 
              << "°, theta2=" << RadiansToDegrees(theta2) << "°:\n";
    
    // Base to joint 1
    Transform T01(RotationMatrix::RotZ(theta1), Eigen::Vector3d(1, 0, 0));
    
    // Joint 1 to joint 2 (includes rotation at joint 2)
    Transform T12(RotationMatrix::RotZ(theta2), Eigen::Vector3d(1, 0, 0));
    
    // Base to end effector
    Transform T02 = T01 * T12;
    
    std::cout << "\nEnd effector position: [" << std::fixed << std::setprecision(4)
              << T02.GetTranslation()(0) << ", " 
              << T02.GetTranslation()(1) << ", " 
              << T02.GetTranslation()(2) << "]\n";
    
    Quaternion q_eef = T02.GetQuaternion();
    std::cout << "End effector orientation: " << q_eef.ToString() << "\n";
}

// ============================================================================
// Example 10: Converting Between Representations
// ============================================================================
void Example10_ConversionChain() {
    PrintSection("Example 10: Conversion Chain Between Representations");
    
    std::cout << "\n10a. Start with Euler angles:\n";
    EulerAngles euler_original(PI/6, PI/4, PI/3);
    std::cout << "Original Euler angles: " << euler_original.ToString() << "\n";
    
    std::cout << "\n10b. Convert to rotation matrix:\n";
    RotationMatrix R = euler_original.ToRotationMatrix();
    std::cout << "Valid rotation matrix: " << (R.IsValid() ? "YES" : "NO") << "\n";
    
    std::cout << "\n10c. Convert to quaternion:\n";
    Quaternion q = R.ToQuaternion();
    std::cout << "Quaternion: " << q.ToString() << "\n";
    
    std::cout << "\n10d. Convert back to Euler angles:\n";
    EulerAngles euler_reconstructed(q);
    std::cout << "Reconstructed: " << euler_reconstructed.ToString() << "\n";
    
    std::cout << "\n10e. Check consistency:\n";
    std::cout << "Roll difference:  " << std::abs(euler_original.Roll() - euler_reconstructed.Roll()) << "\n";
    std::cout << "Pitch difference: " << std::abs(euler_original.Pitch() - euler_reconstructed.Pitch()) << "\n";
    std::cout << "Yaw difference:   " << std::abs(euler_original.Yaw() - euler_reconstructed.Yaw()) << "\n";
}

// ============================================================================
// Main Function
// ============================================================================
int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║         Robotics Transform Library - Examples Demo                  ║\n";
    std::cout << "║   Modern C++ Control Library for Autonomous Vehicles                ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";
    
    try {
        Example1_QuaternionBasics();
        Example2_RotationMatrix();
        Example3_PointTransformation();
        Example4_EulerAngles();
        Example5_QuaternionInterpolation();
        Example6_3DTransforms();
        Example7_TransformComposition();
        Example8_TransformInterpolation();
        Example9_RobotKinematics();
        Example10_ConversionChain();
        
        std::cout << "\n" << std::string(70, '=') << "\n";
        std::cout << "All examples completed successfully!\n";
        std::cout << std::string(70, '=') << "\n\n";
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    }
}