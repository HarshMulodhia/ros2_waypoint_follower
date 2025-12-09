#pragma once

#include<cmath>
#include<string>
#include<sstream>
#include<eigen3/Eigen/Dense>

namespace robotics_control {

// Constants
namespace constants {
    constexpr double PI = 3.14159265358979323846;
    constexpr double EPSILON = 1e-10;
    constexpr double QUAT_NORM_TOLERANCE = 1e-6;
    constexpr double ORTHOGONAL_TOLERANCE = 1e-6;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
}

// Helper functions

/**
 * @brief Convert degrees to radians
 */
inline double DegreesToRadians(double degrees) {
    return degrees * constants::DEG_TO_RAD;
}

/**
 * @brief Convert radians to degress
 */
inline double RadiansToDegrees(double radians) {
    return radians*constants::RAD_TO_DEG;
}

/**
 * @brief Clamp value between min and max
 */
template <typename T>
inline T Clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief Check if two floating point numbers are approximately equal
 */
inline bool ApproxEqual(double a, double b, double tolerance=constants::EPSILON) {
    return std::abs(a-b) < tolerance;
}

/**
 * @brief Normalize angle to [-pi, pi] range
 */
inline double NormalizeAngle(double angle) {
    while (angle > constants::PI) angle -= 2.0*constants::PI;
    while (angle < -constants::PI) angle += 2.0*constants::PI;
    return angle;
}

/**
 * @brief Wrap angle to [0, 2*pi] range
 */
inline double WrapAngle(double angle) {
    while (angle < 0) angle += 2.0*constants::PI;
    while (angle >= 2.0*constants::PI) angle -= 2.0*constants::PI;
    return angle;
}

// Forward Declarations
class Quaternion;
class RotationMatrix;
class EulerAngles;

// ============================================================================
// Quaternion Class
// ============================================================================

/**
 * @class Quaternion
 * @brief Represents a 3D rotation using unit quaternions
 * 
 * A quaternion q = [w, x, y, z]^T, where w is the scalar part.
 * For unit quaternions: w^2 + x^2 + y^2 + z^2 = 1
 * 
 * Advantages:
 * - No singularities
 * - Smooth interpolation (SLERP)
 * - Efficient composition of rotations
 */
class Quaternion{
private:
    Eigen::Vector4d q_; // [w, x, y, z]
    
    /**
     * @brief Normalize the quaternion to unit norm
     */
    void Normalize();

public:
    /**
     * @brief Default constructor (identity rotation)
     */
    Quaternion();

    /**
     * @brief Construct from components
     * @param w Scalar component
     * @param x, y, z Vector components
     */
    Quaternion(double w, double x, double y, double z);

    /**
     * @brief Construct from Eigen vector [w, x, y, z]
     * @param q Quaternion vector (to be normalized)
     */
    explicit Quaternion(const Eigen::Vector4d& q);

    /**
     * @brief Construct from rotation matrix
     * @param R 3X3 rotation matrix
     */
    explicit Quaternion(const RotationMatrix& R);

    /**
     * @brief Construct from axis-angle representation
     * @param axis Rotation axis (unit vector)
     * @param angle Rotation angle (radians)
     */
    static Quaternion FromAxisAngle(const Eigen::Vector3d& axis, double angle);

    // Helper Functions

    /**
     * @brief Get quaternion as [w, x, y, z] vector
     */
    Eigen::Vector4d Vector() const {return q_;}

    /**
     * @brief Get Scalar component
     */
    double W() const { return q_(0);}

    /**
     * @brief Get vector components
     */
    Eigen::Vector3d XYZ() const { return q_.tail(3);}

    /**
     * @brief Get as 3X3 rotation matrix
     */
    RotationMatrix ToRotationMatrix() const;

    /**
     * @brief Get as Euler angles
     * @param convention Euler angle convention (ZYX, XYZ, ZYZ)
     */
    EulerAngles ToEulerAngles(int convention=0) const;

    /**
     * @brief Get axis-angle represntation
     * @param axis Output rotation axis (unit vector)
     * @param angle Output roataion angle (radians)
     */
    void ToAxisAngle(Eigen::Vector3d& axis, double& angle) const;

    /**
     * @brief Get Conjugate (inverse for unit quaternions)
     */
    Quaternion Conjugate() const;

    /**
     * @brief Get Inverse
     */
    Quaternion Inverse() const {return Conjugate(); }


    // Operations

    /**
     * @brief Compose two roataions: q1*q2
     * Equivalent to applying q2 first, then q1
     */
    Quaternion operator*(const Quaternion& other) const;

    /**
     * @brief Rotate a 3D Point
     * @param p Point to rotate
     * @return Rotated Point
     */
    Eigen::Vector3d Rotate(const Eigen::Vector3d& p) const;

    /**
     * @brief Rotate in reverse direction (apply inverse)
     */
    Eigen::Vector3d RotateInverse(const Eigen::Vector3d& p) const;

    /**
     * @brief Spherical linear intepolation betweeb two rotations
     * @param other Target quaternion
     * @param t Interpolation parameter [0, 1]
     * @return Interpolated quaternion
     */
    Quaternion Slerp(const Quaternion& other, double t) const;

    /**
     * @brief Check if approx. equal to another quaternion
     * Note: q and -q represent the same rotation
     */
    bool IsApprox(const Quaternion& other, double tol=1e-6) const;

    /**
     * @brief Check if Identity rotation
     */
    bool IsIdentity( double tol=1e-8) const;

    /**
     * @brief String representation for debugging
     */
    std::string ToString() const;
};

// ============================================================================
// Rotation Matrix Class
// ============================================================================

/**
 * @class RotationMatrix
 * @brief Represent 3D rotations using 3X3 orthogonal matrices
 * 
 * Properties:
 * - R ∈ SO(3): R ∈ ℝ³ˣ³, RᵀR = I, det(R) = 1
 */
class RotationMatrix {
private:
    Eigen::Matrix3d R_;    // 3X3 rotation matrix

    /**
     * @brief Orthogonalize the matrix using Gram-Schmidt
     * Used internally for numerical stability
     */
    void Orthogonalize();

public: 
    /**
     * @brief Default constructor (identity)
     */
    RotationMatrix();

    /**
     * @brief Construct from 3X3 matrix
     * @param R 3X3 orthogonal matrix
     */
    explicit RotationMatrix(const Eigen::Matrix3d& R);

    /**
     * @brief Construct from quaternion
     */
    explicit RotationMatrix(const Quaternion& q);

    /**
     * @brief Construct from Euler angles
     */
    explicit RotationMatrix(const EulerAngles& euler);

    /**
     * @brief Construct rotation around X axis (roll)
     * @param angle Rotation angle (radians)
     */
    static RotationMatrix RotX(double angle);

    /**
     * @brief Construct rotation around Y axis (pitch)
     * @param angle Rotation angle in radians
     */
    static RotationMatrix RotY(double angle);
    
    /**
     * @brief Construct rotation around Z axis (yaw)
     * @param angle Rotation angle in radians
     */
    static RotationMatrix RotZ(double angle);

    /**
     * @brief Construct rotation around arbitrary axis
     * @param axis Rotation axis(unit vector)
     * @param angle Rotation angle (radians)
     */
    static RotationMatrix RotAxis(const Eigen::Vector3d& axis, double angle);

    // Helper functions

    /**
     * @brief Get underlying 3X3 matrix
     */
    Eigen::Matrix3d Matrix() const {return R_;}

    /**
     * @brief Get matrix as reference
     */
    const Eigen::Matrix3d& MatrixRef() const {return R_;}

    /**
     * @brief Get as quaternion
     */
    Quaternion ToQuaternion() const;

    /**
     * @brief Get as Euler angles
     */
    EulerAngles ToEulerAngles(int convention=0) const;

    /**
     * @brief Get transpose
     */
    RotationMatrix Transpose() const;

    /**
     * @brief Get Inverse (same as transpose for rotation matrices)
     */
    RotationMatrix Inverse() const {return Transpose();}

    /**
     * @brief Check orthogonality: R^T*R = I
     */
    bool IsOrthogonal(double tol = 1e-6) const;
    
    /**
     * @brief Check if proper rotation: det(R) = 1
     */
    bool IsProperRotation(double tol = 1e-6) const;

    /**
     * @brief Check if valid rotation matrix
     */
    bool IsValid(double tol = 1e-6) const;

    // Operations

    /**
     * @brief Compose rotations: R1 * R2
     */
    RotationMatrix operator*(const RotationMatrix& other) const;

    /**
     * @brief Rotate a 3D point: R * p
     */
    Eigen::Vector3d operator*(const Eigen::Vector3d& p) const;

    /**
     * @brief Check approx. equality
     */
    bool IsApprox(const RotationMatrix& other, double tol=1e-6) const;

    /**
     * @brief String representation
     */
    std::string ToString() const;
};

// ============================================================================
// Euler Angles Class
// ============================================================================

/**
 * @class EulerAngles
 * @brief Represent rotations using Euler angles
 * 
 * Conventions: ZYX (yaw, pitch, roll) - most common in robotics
 * Rotation order: Rz(yaw) * Ry(pitch) * Rx(roll)
 * 
 * Also called Tait-Bryan angles or nautical angles.
 * 
 * Angles are in radians.
 */
class EulerAngles {
private:
    double roll_;
    double pitch_;
    double yaw_;
    int convention_;    //0: ZYX, 1: XYZ, 2: ZYZ

public:
    /**
     * @brief Default constructor (zero angles)
     */
    EulerAngles();

    /**
     * @brief Construct from individual angles (radians)
     * @param roll Rotation around X axis
     * @param pitch Rotation around Y axis
     * @param yaw Rotation around Z axi
     * @param conventions Euler convention (0=ZYX, 1=XYZ, 2=ZYZ)
     */
    EulerAngles(double roll, double pitch, double yaw, int convention=0);

    /**
     * @brief Construct from quaternion
     */
    explicit EulerAngles(const Quaternion& q, int convention=0);

    /**
     * @brief Construct from rotation matrix
     */
    explicit EulerAngles(const RotationMatrix& R, int convention=0);

    // Helper Functions

    double Roll() const {return roll_;}
    double Pitch() const {return pitch_;}
    double Yaw() const {return yaw_;}
    int Convention() const {return convention_;}

    /**
     * @brief Get all angles as vector [roll, pitch, yaw]
     */
    Eigen::Vector3d Vector() const {
        return Eigen::Vector3d(roll_, pitch_, yaw_);
    }

    /**
     * @brief Get as quaternion
     */
    Quaternion ToQuaternion() const;

    /**
     * @brief Get as rotation matrix
     */
    RotationMatrix ToRotationMatrix() const;

    /**
     * @brief Check if angles cause gimbal lock (pitch = +- 90 degrees)
     */
    bool HasGimbalLock(double tol=1e-6) const;

    /**
     * @brief String representation
     */
    std::string ToString() const;
};

// ============================================================================
// Homogeneous Transform Class
// ============================================================================

/**
 * @class Transform
 * @brief 3D rigid transformation (rotation + translation)
 * 
 * Represents SE(3) transformations:
 * T = [R t]    where R ∈ SO(3) (rotation) and t ∈ ℝ³ (translation)
 *     [0 1]
 * 
 * Transforms a point as: p' = R*p + t
 * Composes with other transforms as: T1 * T2
 */
class Transform {
private:
    Eigen::Matrix4d T_;    // 4x4 homogeneous transformation matrix

public:
    /**
     * @brief Default constructor (identity transform)
     */
    Transform();

    /**
     * @brief Construct from rotation and translation
     * @param R 3X3 rotation matrix
     * @param t 3D translation vector
     */
    Transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

    /**
     * @brief Construct from RotationMatrix and translation
     * @param rotation RotationMatrix object
     * @param translation Translation vector
     */
    Transform(const RotationMatrix& rotation, const Eigen::Vector3d& translation);

    /**
     * @brief Construct from quaternion and translation
     * @param q Quaternion representing rotation
     * @param t Translation vector
     */
    Transform(const Quaternion& q, const Eigen::Vector3d& t);

    /**
     * @brief Construct from 4X4 matrix
     * @param T 4X4 homogeneous transformation matrix
     */
    explicit Transform(const Eigen::Matrix4d& T);

    // Helper Functions

    /**
     * @brief Get full 4X4 transformation matrix
     */
    Eigen::Matrix4d Matrix() const {return T_;}

    /**
     * @brief Get const reference to matrix
     */
    const Eigen::Matrix4d& MatrixRef() const {return T_;}

    /**
     * @brief Get rotation matrix
     */
    RotationMatrix GetRotation() const;

    /**
     * @brief Get translation vector
     */
    Eigen::Vector3d GetTranslation() const;

    /**
     * @brief Get rotation as quaternion
     */
    Quaternion GetQuaternion() const;
    
    /**
     * @brief Get rotation as Euler angles
     */
    EulerAngles GetEulerAngles(int convention = 0) const;

    // Operations

    /**
     * @brief Transform a 3D point
     * Computes: p' = R*p + t
     * 
     * @param p Point to transform
     * @return Transformed point
     */
    Eigen::Vector3d TransformPoint(const Eigen::Vector3d& p) const;

    /**
     * @brief Transform a 3D vector (no translation)
     * Computes: v' = R*v
     * 
     * @param v Vector to transform
     * @return Transformed vector
     */
    Eigen::Vector3d TransformVector(const Eigen::Vector3d& v) const;

    /**
     * @brief Inverse transform: apply T^{-1}
     * @param p Point to inverse transform
     * @return Point in pre-transform frame
     */
    Eigen::Vector3d InverseTransformPoint(const Eigen::Vector3d& p) const;

    /**
     * @brief Inverse transform vector (no translation)
     */
    Eigen::Vector3d InverseTransformVector(const Eigen::Vector3d& v) const;

    /**
     * @brief Compose two transforms: T1 * T2
     * Equivalent to applying T2 first, then T1
     */
    Transform operator*(const Transform& other) const;

    /**
     * @brief Transform a point using operator notation
     */
    Eigen::Vector3d operator*(const Eigen::Vector3d& p) const;

    /**
     * @brief Get inverse transform
     * T⁻¹ = [R^T  -R^T*t]
     *       [0      1   ]
     */
    Transform Inverse() const;
    
    /**
     * @brief Linear interpolation between two transforms
     * Interpolates rotation using SLERP and translation linearly
     * 
     * @param other Target transform
     * @param t Interpolation parameter [0, 1]
     * @return Interpolated transform
     */
    Transform Lerp(const Transform& other, double t) const;

    /**
     * @brief Check if this is an identity transform
     */
    bool IsIdentity(double tol = 1e-8) const;
    
    /**
     * @brief Check if transform is valid (rotation is proper)
     */
    bool IsValid(double tol = 1e-6) const;
    
    /**
     * @brief Get distance to another transform
     * Combines rotational and translational distance
     */
    double Distance(const Transform& other) const;
    
    /**
     * @brief String representation for debugging
     */
    std::string ToString() const;

};

}