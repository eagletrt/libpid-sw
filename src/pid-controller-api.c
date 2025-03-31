/*!
 * \file pid-controller-api.c
 * \date 2025-03-25
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 *
 * \brief PID Controller Implementation.
 *
 * This file contains the implementation of the PID controller functions, 
 * including initialization, update, computation, and reset. The functions 
 * manage the PID control logic, including error correction, integral windup, 
 * and derivative computation, to ensure effective control of systems.
 */

#include <math.h>
#include <stdio.h>
#include "pid-controller-api.h"

/*!
 * \brief Initializes the PID controller with specified parameters.
 * 
 * This function initializes the PID controller structure with the given proportional (kp), 
 * integral (ki), and derivative (kd) gains, as well as other parameters including sample time, 
 * anti-windup limit, arena for memory allocation, and the number of previous errors to store.
 * If the number of previous errors is zero, the previous error buffer is not allocated.
 * 
 * \param pid_controller A pointer to the PID controller structure.
 * \param kp The proportional gain.
 * \param ki The integral gain.
 * \param kd The derivative gain.
 * \param sample_time The sample time (in seconds) for PID updates.
 * \param anti_windUp The anti-windup limit for the integrator.
 * \param arena A pointer to the arena allocator for dynamic memory allocation.
 * \param n_prev_errors The number of previous errors to store for derivative calculation.
 */
int pid_controller_init(PidController_t *pid_controller,
              float kp,
              float ki,
              float kd,
              float sample_time,
              float anti_windUp,
              ArenaAllocatorHandler_t *arena,
              uint8_t n_prev_errors) {

    if (pid_controller == NULL || (arena == NULL && n_prev_errors > 0)) return PID_ERROR_NULL_PTR;

    pid_controller->kp = kp;
    pid_controller->ki = ki;
    pid_controller->kd = kd;
    pid_controller->integrator = 0.0f;

    pid_controller->error = 0.0f;
    pid_controller->set_point = 0.0f;
    pid_controller->sample_time = sample_time;
    pid_controller->anti_windUp = anti_windUp;

    if(n_prev_errors == 0){
        pid_controller->n_prev_errors = 0;
        pid_controller->prev_error_index = 0;
        pid_controller->prev_errors = NULL;
        return PID_SUCCESS;
    }
        
    pid_controller->n_prev_errors = n_prev_errors;
    pid_controller->prev_error_index = pid_controller->n_prev_errors - 1;
    pid_controller->prev_errors = (float*) arena_allocator_api_calloc(arena, sizeof(float), n_prev_errors);
    
    if (pid_controller->prev_errors == NULL)  return PID_ERROR_MEM_ALLOC; 

    for(int i = 0; i<n_prev_errors; i++){
        pid_controller->prev_errors[i] = 0.0f;
    }
    return PID_SUCCESS;
}

/*!
 * \brief Updates the PID controller with the current process variable (status).
 * 
 * This function calculates the error and updates the integral and derivative terms of the PID 
 * controller based on the current process variable (status) and the set point.
 * If the number of previous errors is greater than zero, the function updates the previous error buffer.
 * 
 * \param pid_controller A pointer to the PID controller structure.
 * \param status The current process variable (status) to be used in the PID calculation.
 */
void pid_controller_update(PidController_t *pid_controller, float status) {

    if(pid_controller->n_prev_errors == 0) {
        pid_controller->error = pid_controller->set_point - status;
        pid_controller->integrator += pid_controller->error * pid_controller->sample_time;
        return;
    }

    pid_controller->prev_error_index = (pid_controller->prev_error_index + 1) % pid_controller->n_prev_errors;
    pid_controller->prev_errors[pid_controller->prev_error_index] = pid_controller->error;
    pid_controller->error = pid_controller->set_point - status;
    pid_controller->integrator += pid_controller->error * pid_controller->sample_time;
    if (fabs(pid_controller->ki) > 0.0001f) {
        if (pid_controller->integrator * pid_controller->ki > pid_controller->anti_windUp) {
            pid_controller->integrator = pid_controller->anti_windUp / pid_controller->ki;
        } else if (pid_controller->integrator * pid_controller->ki < -pid_controller->anti_windUp) {
            pid_controller->integrator = -pid_controller->anti_windUp / pid_controller->ki;
        }
    }
}

/*!
 * \brief Computes the PID controller output based on the current error, integral, and derivative.
 * 
 * This function calculates the PID control signal, which includes the proportional, integral, 
 * and derivative terms, and returns the resulting output value.
 * 
 * \param pid_controller A pointer to the PID controller structure.
 * 
 * \return The computed PID control output value.
 */
float pid_controller_compute(PidController_t *pid_controller) {

    if(pid_controller->n_prev_errors == 0){
        float integral = pid_controller->ki * pid_controller->integrator;
        float value = pid_controller->kp * pid_controller->error + integral;
        return value;
    }

    uint8_t index = (pid_controller->prev_error_index + 1) % pid_controller->n_prev_errors;
    float derivative = (pid_controller->error - pid_controller->prev_errors[index]) / (pid_controller->sample_time * pid_controller->n_prev_errors);
    float integral = pid_controller->ki * pid_controller->integrator;

    float value = pid_controller->kp * pid_controller->error + integral + pid_controller->kd * derivative;
    return value;
}

/*!
 * \brief Resets the PID controller by clearing all errors and the integrator.
 * 
 * This function resets the integrator, error values, and the previous error buffer of the PID 
 * controller to prepare it for a new control cycle.
 * 
 * \param pid_controller A pointer to the PID controller structure.
 */
void pid_controller_reset(PidController_t *pid_controller) {
    pid_controller->integrator = 0.0f;
    pid_controller->error = 0.0f;
    for (int i = 0; i < pid_controller->n_prev_errors; ++i) {
        pid_controller->prev_errors[i] = 0.0f;
    }
}
