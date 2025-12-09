#pragma once
#include <cmath>

namespace robotics_control {

/**
 * @class PIDController
 * @brief A discrete-time PID controller implementation
 * 
 * Implements a standard PID control law:
 * u[k] = Kp * e[k] + Ki * sum(e) + Kd * (e[k] - e[k-1])
 * 
 * Features:
 * - Integral anti-windup (output clamping)
 * - Derivative filter for noise reduction
 * - Saturation limits on control output
 */
class PIDController {
public:
    /**
     * @brief Constructor
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param dt Sampling time (seconds)
     * @param output_min Minimum control output saturation limit
     * @param output_max Maximum control output saturation limit
     */
    PIDController(double kp, double ki, double kd, double dt,
                  double output_min = -1.0, double output_max = 1.0);

    /**
     * @brief Reset controller state
     */
    void reset();

    /**
     * @brief Update controller and return control output
     * @param setpoint Desired reference value
     * @param measurement Current measured value
     * @return Control signal u[k]
     */
    double update(double setpoint, double measurement);

    /**
     * @brief Set gains
     */
    void set_gains(double kp, double ki, double kd);

    /**
     * @brief Get current error
     */
    double get_error() const { return error_; }

    /**
     * @brief Get integral term
     */
    double get_integral() const { return integral_; }

    /**
     * @brief Get derivative term
     */
    double get_derivative() const { return derivative_; }

private:
    double kp_, ki_, kd_;              // Gains
    double dt_;                        // Sampling time
    double output_min_, output_max_;   // Saturation limits
    
    // State variables
    double error_;                     // Current error
    double integral_;                  // Accumulated integral
    double prev_error_;                // Previous error for derivative
    double derivative_;
};

}