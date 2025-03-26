/*!
 * \file pid-controller-basic.c
 * \date 2025-03-25
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 * 
 * \brief Basic example of how to initialize and use a PID controller with an arena allocator.
 *
 * This is a basic example demonstrating how to initialize and use the PID controller
 * with an arena allocator for managing previous error values. It shows how to set up
 * the PID controller with user-defined parameters, including the proportional, integral,
 * and derivative gains (Kp, Ki, Kd), the sample time, and anti-windup parameter.
 * It also illustrates how the previous errors are stored in a dynamically allocated
 * memory block using the arena allocator.
 */

 #include "arena-allocator.h"
 #include "pid-controller.h"
 
 /*!
  * \brief Basic example of initializing the PID controller.
  * \details This function initializes a PID controller, sets the parameters for
  *      the PID computation, and allocates memory for storing previous errors using the
  *      arena allocator. It also sets an initial set point for the PID controller.
  */
 int main(void) {
     ArenaAllocatorHandler_t arena;
     PidController_t pid_controller;
     uint8_t n_prev_errors = 5;
 
     // Initialize the arena allocator
     arena_allocator_api_init(&arena);
 
     // Initialize the PID controller
     pid_init(&pid_controller, 1.0f, 0.5f, 0.1f, 0.1f, 10.0f, &arena, n_prev_errors);
 
     // Simulate setting the setpoint and running the PID loop
     pid_controller.set_point = 100.0f;
     
     // Example loop that updates the PID controller
     for (int i = 0; i < 100; i++) {
         float current_value = 90.0f + i;  // Simulated current value
         pid_controller.error = pid_controller.set_point - current_value;
         float control_signal = pid_calculate(&pid_controller);
         
         // Use the control signal in your system (e.g., motor speed, etc.)
     }
 
     // Free the allocated memory for previous errors
     arena_allocator_api_free(&arena);
 
     return 0;
 }
 