/*!
 * \file pid-controller.h
 * \date 2025-03-25
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 *
 * \brief PID Controller definition.
 *
 * This file defines the structure for the PID controller, including the 
 * parameters for proportional, integral, and derivative gains, along with 
 * the error history, sample time, and anti-windup settings. It is used in 
 * conjunction with the PID controller API to implement control functionality.
 */


#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <arena-allocator-api.h>

/*!
 * \brief PID Controller structure.
 * 
 * Contains parameters for the PID controller, including the gains (kp, ki, kd), 
 * the integrator, error, set point, and anti-windup limit. It also holds 
 * information about previous errors used for derivative calculation.
 */
typedef struct pidController_t {
    float kp;                   //Proportional gain 
    float ki;                   //Integral gain 
    float kd;                   //Derivative gain 
    float integrator;           //Integral value 
    float error;                //Current error (set_point - status) 
    float sample_time;          //Sample time (seconds) 
    float set_point;            //Desired target value 
    float anti_windUp;          //Anti-windup limit for integrator 
    uint8_t n_prev_errors;      //Number of previous errors stored for derivative 
    int prev_error_index;       //Index of the last stored error 
    float *prev_errors;         //Array of previous errors for derivative calculation 
} PidController_t;

#endif
