#pragma once

namespace control {

/**
 * @brief Simple PID controller class.
 * Decoupled from any hardware or RTOS.
 */
struct PIDGains {
    float kp;
    float ki;
    float kd;
    float i_limit;
};

class PID {
public:
    PID() = default;

    void set_gains(const PIDGains& gains) {
        gains_ = gains;
    }

    void reset() {
        integral_ = 0.0f;
        prev_error_ = 0.0f;
    }

    float update(float setpoint, float measurement, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - measurement;
        
        // Integral with limit
        integral_ += error * dt;
        if (gains_.i_limit > 0.0f) {
            if (integral_ > gains_.i_limit) integral_ = gains_.i_limit;
            else if (integral_ < -gains_.i_limit) integral_ = -gains_.i_limit;
        }

        // Derivative
        float derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        return (gains_.kp * error) + (gains_.ki * integral_) + (gains_.kd * derivative);
    }

private:
    PIDGains gains_{0.0f, 0.0f, 0.0f, 0.0f};
    float integral_ = 0.0f;
    float prev_error_ = 0.0f;
};

/**
 * @brief Cascaded PID controller for attitude and rate control.
 */
class CascadedPID {
public:
    struct Vector3 { float x, y, z; };

    CascadedPID() = default;

    void set_angle_gains(const PIDGains& roll, const PIDGains& pitch) {
        roll_angle_.set_gains(roll);
        pitch_angle_.set_gains(pitch);
    }

    void set_rate_gains(const PIDGains& roll, const PIDGains& pitch, const PIDGains& yaw) {
        roll_rate_.set_gains(roll);
        pitch_rate_.set_gains(pitch);
        yaw_rate_.set_gains(yaw);
    }

    void reset() {
        roll_angle_.reset();
        pitch_angle_.reset();
        roll_rate_.reset();
        pitch_rate_.reset();
        yaw_rate_.reset();
    }

    /**
     * @brief Update the cascaded PID controller.
     * @param target_euler Target attitude (roll, pitch in rad, yaw is target rate in rad/s)
     * @param current_euler Current attitude (roll, pitch, yaw in rad)
     * @param current_rates Current angular rates (rad/s)
     * @param dt Timestep in seconds
     * @return Torque corrections for roll, pitch, and yaw axes.
     */
    Vector3 update(const Vector3& target_euler, const Vector3& current_euler, 
                   const Vector3& current_rates, float dt) {
        // Outer loop: Angle to Rate
        float roll_rate_sp = roll_angle_.update(target_euler.x, current_euler.x, dt);
        float pitch_rate_sp = pitch_angle_.update(target_euler.y, current_euler.y, dt);
        
        // Inner loop: Rate to Torque
        Vector3 torque;
        torque.x = roll_rate_.update(roll_rate_sp, current_rates.x, dt);
        torque.y = pitch_rate_.update(pitch_rate_sp, current_rates.y, dt);
        torque.z = yaw_rate_.update(target_euler.z, current_rates.z, dt); // Yaw is rate-controlled
        
        return torque;
    }

private:
    PID roll_angle_, pitch_angle_;
    PID roll_rate_, pitch_rate_, yaw_rate_;
};

} // namespace control
