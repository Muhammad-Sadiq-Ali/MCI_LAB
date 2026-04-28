#include <stdint.h>
#include <stdbool.h>
 
// PID Structure Definition
typedef struct {
    // Controller Gains
    float Kp;
    float Ki;
    float Kd;
 
    // Sample time (in seconds) - e.g., 0.01 for 100Hz loop
    float dt;
 
    // Target state
    float setpoint;
 
    // Controller memory
    float integral;
    float previous_error;
 
    // Derivative filter coefficient (0.0 to 1.0)
    // Lower values = more filtering (smoother but adds delay)
    float alpha;
    float filtered_derivative;
 
    // Output limits
    float out_min;
    float out_max;
} PID_Controller;
 
/**
* @brief  Initializes the PID controller
*/
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float dt, float out_min, float out_max, float alpha) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = dt;
    pid->setpoint = 0.0f; // Usually 0 degrees for a balancing robot
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->alpha = alpha;
    pid->filtered_derivative = 0.0f;
}
 
/**
* @brief  Computes the PID output
* @param  pid: Pointer to the PID structure
* @param  current_angle: Current angle in degrees
* @param  direction: Output pointer for motor direction (1 = Forward, 0 = Backward)
* @retval Integer magnitude for PWM (e.g., 0 to 1000)
*/
int16_t PID_Compute(PID_Controller *pid, float current_angle, uint8_t *direction) {
    // 1. Calculate error
    float error = pid->setpoint - current_angle;
 
    // 2. Proportional term
    float P_out = pid->Kp * error;
 
    // 3. Integral term with anti-windup (stops integrating if output is saturated)
    pid->integral += error * pid->dt;
    float I_out = pid->Ki * pid->integral;
 
    // 4. Derivative term with Low-Pass Filter
    float derivative = (error - pid->previous_error) / pid->dt;
    pid->filtered_derivative = (pid->alpha * derivative) + ((1.0f - pid->alpha) * pid->filtered_derivative);
    float D_out = pid->Kd * pid->filtered_derivative;
 
    // 5. Total Output
    float output = P_out + I_out + D_out;
 
    // 6. Saturation and Anti-Windup
    if (output > pid->out_max) {
        output = pid->out_max;
        pid->integral -= error * pid->dt; // Undo integration
    } else if (output < pid->out_min) {
        output = pid->out_min;
        pid->integral -= error * pid->dt; // Undo integration
    }
 
    pid->previous_error = error;
 
    // 7. Extract Direction and Magnitude
    if (output >= 0) {
        *direction = 1; // Forward
        return (int16_t)output;
    } else {
        *direction = 0; // Backward
        return (int16_t)(-output); // Return absolute value for PWM magnitude
    }
}