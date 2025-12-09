#include "maths/transform.h"
#include <iomanip>
#include<eigen3/Eigen/Eigenvalues>

using namespace Eigen;
namespace robotics_control {

// ============================================================================
// Quaternion Implementation
// ============================================================================

Quaternion::Quaternion() : q_(1, 0, 0, 0) {}

Quaternion::Quaternion(double w, double x, double y, double z)
: q_(w, x, y, z) {
    Normalize();
}

Quaternion::Quaternion(const Vector4d& q) : q_(q) {
    Normalize();
}

Quaternion::Quaternion(const RotationMatrix& R) {
    *this = R.ToQuaternion();
}

void Quaternion::Normalize() {
    double norm = q_.norm();
    if(norm>constants::EPSILON) {
        q_ /= norm;
    } else {
        q_ = Vector4d(1, 0, 0, 0);
    }
}

Quaternion Quaternion::FromAxisAngle(const Vector3d& axis, double angle) {
    Vector3d normalized_axis = axis.normalized();
    double w = std::cos(angle/2.0);
    Vector3d xyz = std::sin(angle/2.0) * normalized_axis;
    return Quaternion(w, xyz(0), xyz(1), xyz(2));
}

RotationMatrix Quaternion::ToRotationMatrix() const {
    double w = q_(0), x = q_(1), y = q_(2), z = q_(3);
    Matrix3d R;
    R << 1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y),
             2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x),
             2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y);
    
    return RotationMatrix(R);
}

EulerAngles Quaternion::ToEulerAngles(int convention) const {
    return EulerAngles(*this, convention);
}

void Quaternion::ToAxisAngle(Vector3d& axis, double& angle) const {
    // Choose +ve w (for shorter rotation)
    double w = q_(0);
    Vector3d xyz = q_.tail(3);

    if(w<0){
        w=-w; xyz=-xyz;
    }

    //Clamp to avoid numerical issues with acos
    w=std::max(-1.0, std::min(1.0, w));
    angle = 2.0*std::acos(w);

    double sin_half = std::sin(angle/2.0);
    if(sin_half > constants::EPSILON) {
        axis = xyz / sin_half;
    } else {// Near identity rotation
        axis = Vector3d(0, 0, 1);
        angle=0;
    }
}

Quaternion Quaternion::Conjugate() const {
    return Quaternion(q_(0), -q_(1), -q_(2), -q_(3));
}

Quaternion Quaternion::operator*(const Quaternion& other) const {
    double w1 = q_(0), x1 = q_(1), y1 = q_(2), z1 = q_(3);
    double w2 = other.q_(0), x2 = other.q_(1), y2 = other.q_(2), z2 = other.q_(3);

    // Hamilton Product
    return Quaternion(
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    );
}

Vector3d Quaternion::Rotate(const Vector3d& p) const {
    // v' = q*v*q^*
    // Efficient formula: v' = v + 2*w*(xyz × v) + 2*(xyz × (xyz × v))

    double w = q_(0); Vector3d xyz = q_.tail(3);

    return p + 2*w*(xyz.cross(p)) + 2*(xyz.cross(xyz.cross(p)));
}

Vector3d Quaternion::RotateInverse(const Vector3d& p) const {
    return Conjugate().Rotate(p);
}

Quaternion Quaternion::Slerp(const Quaternion& other, double t) const {
    t = std::max(0.0, std::min(1.0, t)); // Clamp to [0, 1]

    double dot = q_.dot(other.q_);
    Quaternion other_adj = other;

    // If dot < 0, negate one quaternion to take shorter path
    if (dot < 0) {
        other_adj.q_ = -other_adj.q_; dot = -dot;
    }

    // Clamp dot to avoid numerical issues
    dot = std::max(-1.0, std::min(1.0, dot));

    double theta = std::acos(dot);
    double sin_theta = std::sin(theta);

    if (sin_theta < constants::EPSILON) {
        // Nearly identical, use linear interpolation
        return Quaternion((1-t)*q_ + t*other_adj.q_);
    }

    double w1 = std::sin((1-t)*theta) / sin_theta;
    double w2 = std::sin(t*theta) / sin_theta;

    return Quaternion(w1*q_ + w2*other_adj.q_);
}

bool Quaternion::IsApprox(const Quaternion& other, double tol) const {
    // Account for q and -q representing the same rotation
    bool same = (q_ - other.q_).norm() < tol;
    bool opposite = (q_ + other.q_).norm() < tol;
    return same || opposite;
}

bool Quaternion::IsIdentity(double tol) const {
    return std::abs(q_(0) - 1.0) < tol && q_.tail(3).norm() < tol;
}

std::string Quaternion::ToString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "Quaternion(w=" << q_(0) << ", x=" << q_(1) 
       << ", y=" << q_(2) << ", z=" << q_(3) << ")";
    return ss.str();
}

// ============================================================================
// RotationMatrix Implementation
// ============================================================================

RotationMatrix::RotationMatrix() : R_(Matrix3d::Identity()) {}

RotationMatrix::RotationMatrix(const Matrix3d& R) : R_(R) {
    Orthogonalize();
}

RotationMatrix::RotationMatrix(const Quaternion& q) {
    *this = q.ToRotationMatrix();
}

RotationMatrix::RotationMatrix(const EulerAngles& euler) {
    *this = euler.ToRotationMatrix();
}

void RotationMatrix::Orthogonalize() {
    // Using Gram-Schmidt to orthogonalize
    ColPivHouseholderQR<Matrix3d> qr(R_);
    R_ = qr.householderQ();

    // Ensure proper rotation
    if (R_.determinant() < 0) {
        R_.col(2) = -R_.col(2);
    }
}

RotationMatrix RotationMatrix::RotX(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Matrix3d R;
    R << 1,  0,  0,
         0,  c, -s,
         0,  s,  c;
    return RotationMatrix(R);
}

RotationMatrix RotationMatrix::RotY(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Matrix3d R;
    R << c,  0,  s,
         0,  1,  0,
        -s,  0,  c;
    return RotationMatrix(R);
}

RotationMatrix RotationMatrix::RotZ(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Matrix3d R;
    R << c, -s,  0,
         s,  c,  0,
         0,  0,  1;
    return RotationMatrix(R);
}

RotationMatrix RotationMatrix::RotAxis(const Vector3d& axis, double angle) {
    Quaternion q = Quaternion::FromAxisAngle(axis, angle);
    return RotationMatrix(q);
}

Quaternion RotationMatrix::ToQuaternion() const {
    // Shepperd's method for numerical stability
    double trace = R_.trace();
    double w, x, y, z;
    
    if(trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (R_(2,1) - R_(1,2)) * s;
        y = (R_(0,2) - R_(2,0)) * s;
        z = (R_(1,0) - R_(0,1)) * s;
    } else if((R_(0,0) > R_(1,1)) && (R_(0,0) > R_(2,2))) {
        double s = 2.0 * std::sqrt(1.0 + R_(0,0) - R_(1,1) - R_(2,2));
        w = (R_(2,1) - R_(1,2)) / s;
        x = 0.25 * s;
        y = (R_(0,1) + R_(1,0)) / s;
        z = (R_(0,2) + R_(2,0)) / s;
    } else if(R_(1,1) > R_(2,2)) {
        double s = 2.0 * std::sqrt(1.0 + R_(1,1) - R_(0,0) - R_(2,2));
        w = (R_(0,2) - R_(2,0)) / s;
        x = (R_(0,1) + R_(1,0)) / s;
        y = 0.25 * s;
        z = (R_(1,2) + R_(2,1)) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + R_(2,2) - R_(0,0) - R_(1,1));
        w = (R_(1,0) - R_(0,1)) / s;
        x = (R_(0,2) + R_(2,0)) / s;
        y = (R_(1,2) + R_(2,1)) / s;
        z = 0.25 * s;
    }
    
    return Quaternion(w, x, y, z);
}

EulerAngles RotationMatrix::ToEulerAngles(int convention) const {
    return EulerAngles(*this, convention);
}

RotationMatrix RotationMatrix::Transpose() const {
    return RotationMatrix(R_.transpose());
}

bool RotationMatrix::IsOrthogonal(double tol) const {
    Matrix3d Q = R_.transpose() * R_;
    return Q.isIdentity(tol);
}

bool RotationMatrix::IsProperRotation(double tol) const {
    return std::abs(R_.determinant() - 1.0) < tol;
}

bool RotationMatrix::IsValid(double tol) const {
    return IsOrthogonal(tol) && IsProperRotation(tol);
}

RotationMatrix RotationMatrix::operator*(const RotationMatrix& other) const {
    return RotationMatrix(R_ * other.R_);
}

Vector3d RotationMatrix::operator*(const Vector3d& p) const {
    return R_ * p;
}

bool RotationMatrix::IsApprox(const RotationMatrix& other, double tol) const {
    return (R_ - other.R_).norm() < tol;
}

std::string RotationMatrix::ToString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "RotationMatrix:\n" << R_;
    return ss.str();
}

// ============================================================================
// EulerAngles Implementation
// ============================================================================

EulerAngles::EulerAngles() : roll_(0), pitch_(0), yaw_(0), convention_(0) {}

EulerAngles::EulerAngles(double roll, double pitch, double yaw, int convention)
    : roll_(roll), pitch_(pitch), yaw_(yaw), convention_(convention) {}

EulerAngles::EulerAngles(const Quaternion& q, int convention): convention_(convention) {
    *this = EulerAngles(q.ToRotationMatrix().ToEulerAngles(convention));
}

EulerAngles::EulerAngles(const RotationMatrix& R, int convention): convention_(convention) {
    if (convention == 0) {  // ZYX (yaw-pitch-roll)
        const Matrix3d& m = R.MatrixRef();
        
        // Check for gimbal lock
        double sin_pitch = -m(2, 0);
        sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch));
        pitch_ = std::asin(sin_pitch);
        
        if (std::abs(std::cos(pitch_)) > constants::EPSILON) {
            roll_ = std::atan2(m(2, 1), m(2, 2));
            yaw_ = std::atan2(m(1, 0), m(0, 0));
        } else {
            // Gimbal lock: set roll = 0, solve for yaw
            roll_ = 0;
            yaw_ = std::atan2(-m(0, 1), m(1, 1));
        }
    } else if (convention == 1) {  // XYZ (roll-pitch-yaw)
        const Matrix3d& m = R.MatrixRef();
        
        double sin_pitch = m(0, 2);
        sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch));
        pitch_ = std::asin(sin_pitch);
        
        if (std::abs(std::cos(pitch_)) > constants::EPSILON) {
            roll_ = std::atan2(-m(1, 2), m(2, 2));
            yaw_ = std::atan2(-m(0, 1), m(0, 0));
        } else {
            roll_ = 0;
            yaw_ = std::atan2(m(1, 0), m(1, 1));
        }
    } else {  // ZYZ (symmetric)
        const Matrix3d& m = R.MatrixRef();
        pitch_ = std::acos(m(2, 2));
        
        if (std::abs(std::sin(pitch_)) > constants::EPSILON) {
            roll_ = std::atan2(m(2, 1), -m(2, 0));
            yaw_ = std::atan2(m(1, 2), m(0, 2));
        } else {
            roll_ = 0;
            yaw_ = std::atan2(m(0, 1), m(0, 0));
        }
    }
}

Quaternion EulerAngles::ToQuaternion() const {
    return Quaternion(ToRotationMatrix());
}

RotationMatrix EulerAngles::ToRotationMatrix() const {
    if (convention_ == 0) {  // ZYX
        return RotationMatrix(RotationMatrix::RotZ(yaw_) *
                              RotationMatrix::RotY(pitch_) *
                              RotationMatrix::RotX(roll_));
    } else if (convention_ == 1) {  // XYZ
        return RotationMatrix(RotationMatrix::RotX(roll_) *
                              RotationMatrix::RotY(pitch_) *
                              RotationMatrix::RotZ(yaw_));
    } else {  // ZYZ
        return RotationMatrix(RotationMatrix::RotZ(yaw_) *
                              RotationMatrix::RotY(pitch_) *
                              RotationMatrix::RotZ(roll_));
    }
}

bool EulerAngles::HasGimbalLock(double tol) const {
    if (convention_ == 0 || convention_ == 1) {
        return std::abs(std::abs(pitch_) - constants::PI/2.0) < tol;
    } else {  // ZYZ
        return std::abs(pitch_) < tol || std::abs(pitch_ - constants::PI) < tol;
    }
}

std::string EulerAngles::ToString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "EulerAngles(roll=" << roll_ << ", pitch=" << pitch_
       << ", yaw=" << yaw_ << ")";
    if (HasGimbalLock()) ss << " [GIMBAL LOCK]";
    return ss.str();
}

// ============================================================================
// Transform Implementation
// ============================================================================

Transform::Transform() : T_(Matrix4d::Identity()) {}

Transform::Transform(const Matrix3d& R, const Vector3d& t): T_(Matrix4d::Identity()) {
    T_.block(0, 0, 3, 3) = R;
    T_.block(0, 3, 3, 1) = t;
}

Transform::Transform(const RotationMatrix& rotation, const Vector3d& translation) : T_(Matrix4d::Identity()) {
    T_.block(0, 0, 3, 3) = rotation.MatrixRef();
    T_.block(0, 3, 3, 1) = translation;
}

Transform::Transform(const Quaternion& q, const Vector3d& t): T_(Matrix4d::Identity()) {
    T_.block(0, 0, 3, 3) = q.ToRotationMatrix().MatrixRef();
    T_.block(0, 3, 3, 1) = t;
}

Transform::Transform(const Matrix4d& T) : T_(T) {}

RotationMatrix Transform::GetRotation() const {
    return RotationMatrix(T_.block(0, 0, 3, 3));
}

Vector3d Transform::GetTranslation() const {
    return T_.block(0, 3, 3, 1);
}

Quaternion Transform::GetQuaternion() const {
    return GetRotation().ToQuaternion();
}

EulerAngles Transform::GetEulerAngles(int convention) const {
    return GetRotation().ToEulerAngles(convention);
}

Vector3d Transform::TransformPoint(const Vector3d& p) const {
    return GetRotation() * p + GetTranslation();
}

Vector3d Transform::TransformVector(const Vector3d& v) const {
    return GetRotation() * v;
}

Vector3d Transform::InverseTransformPoint(const Vector3d& p) const {
    return GetRotation().Inverse() * (p - GetTranslation());
}

Vector3d Transform::InverseTransformVector(const Vector3d& v) const {
    return GetRotation().Inverse() * v;
}

Transform Transform::operator*(const Transform& other) const {
    return Transform(T_ * other.T_);
}

Transform Transform::Inverse() const {
    RotationMatrix R = GetRotation();
    Vector3d t = GetTranslation();
    
    RotationMatrix R_inv = R.Inverse();
    Vector3d t_inv = -(R_inv * t);
    
    return Transform(R_inv, t_inv);
}

Transform Transform::Lerp(const Transform& other, double t) const {
    t = std::max(0.0, std::min(1.0, t));
    
    // Linear interpolation of translation
    Vector3d t_interp = (1 - t) * GetTranslation() + t * other.GetTranslation();
    
    // SLERP for rotation
    Quaternion q1 = GetQuaternion();
    Quaternion q2 = other.GetQuaternion();
    Quaternion q_interp = q1.Slerp(q2, t);
    
    return Transform(q_interp, t_interp);
}

bool Transform::IsIdentity(double tol) const {
    return GetRotation().MatrixRef().isIdentity() && GetTranslation().norm() < tol;
}

bool Transform::IsValid(double tol) const {
    return GetRotation().IsValid(tol) &&
           std::abs(T_(3, 0)) < tol && std::abs(T_(3, 1)) < tol &&
           std::abs(T_(3, 2)) < tol && std::abs(T_(3, 3) - 1.0) < tol;
}

double Transform::Distance(const Transform& other) const {
    // Combine translational and rotational distance
    Vector3d pos_diff = GetTranslation() - other.GetTranslation();
    double trans_dist = pos_diff.norm();
    
    // Rotational distance: angle of relative rotation
    Transform T_rel = other.Inverse() * (*this);
    double angle = 2.0 * std::acos(std::abs(T_rel.GetQuaternion().W()));
    
    return trans_dist + angle;  // Combined metric
}

std::string Transform::ToString() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "Transform:\n";
    ss << "R =\n" << T_.block(0, 0, 3, 3) << "\n";
    ss << "t = [" << T_(0, 3) << ", " << T_(1, 3) << ", " << T_(2, 3) << "]ᵀ";
    return ss.str();
}

}