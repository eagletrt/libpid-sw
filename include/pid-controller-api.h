/*!
 * \file pid-controller-api.h
 * \date 2025-03-25
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 *
 * \brief PID Controller API header.
 *
 * This file contains the API for the PID controller, including functions for 
 * initialization, update, computation, and reset. It manages the PID algorithm 
 * parameters and integrates the error history for advanced control functionality.
 */

#ifndef PID_CONTROLLER_API_H
#define PID_CONTROLLER_API_H

#include <arena-allocator-api.h>
#include "pid-controller.h"

#define PID_SUCCESS 0
#define PID_ERROR_NULL_PTR -1
#define PID_ERROR_MEM_ALLOC -2

/*!
 * \brief Initialize the PID controller.
 * 
 * Initializes the PID controller with the provided parameters. Allocates memory for 
 * previous errors if needed.
 * 
 * \param[in] pid_controller Pointer to the PID controller structure.
 * \param[in] kp Proportional gain.
 * \param[in] ki Integral gain.
 * \param[in] kd Derivative gain.
 * \param[in] sample_time Sample time in seconds.
 * \param[in] anti_windUp Anti-windup limit for the integrator.
 * \param[in] prev_errors Pointer to the arena allocator for previous errors.
 * \param[in] n_prev_errors Number of previous errors to store for derivative calculation.
 */
int pid_controller_init(PidController_t *pid_controller,
              float kp,
              float ki,
              float kd,
              float sample_time,
              float anti_windUp,
              ArenaAllocatorHandler_t *prev_errors,
              uint8_t n_prev_errors);

/*!
 * \brief Update the PID controller with the current status.
 * 
 * Updates the error and integrator values in the PID controller based on the current 
 * status of the system.
 * 
 * \param[in] pid_controller Pointer to the PID controller structure.
 * \param[in] status Current system status (e.g., current value).
 */
void pid_controller_update(PidController_t *pid_controller, float status);

/*!
 * \brief Compute the PID control value.
 * 
 * Calculates and returns the control value based on the current error, integral, 
 * and derivative components.
 * 
 * \param[in] pid_controller Pointer to the PID controller structure.
 * 
 * \return The computed control value.
 */
float pid_controller_compute(PidController_t *pid_controller);

/*!
 * \brief Reset the PID controller.
 * 
 * Resets the integrator and error values, as well as clearing the previous errors.
 * 
 * \param[in] pid_controller Pointer to the PID controller structure.
 */
void pid_controller_reset(PidController_t *pid_controller);

#endif
