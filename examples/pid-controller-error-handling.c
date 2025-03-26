/*!
 * \file pid-controller-error-handling.c
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 * \date 2025-03-25
 *
* \brief Example of a PID controller with error handling and dynamic memory allocation.
 * 
 * This example shows how to initialize a PID controller with proper error handling
 * and dynamically allocate memory for storing the previous error values. It includes
 * error checking for both the PID controller initialization and the arena allocator's
 * memory allocation, ensuring that the system does not crash if memory allocation fails.
 */

 #include "arena-allocator.h"
 #include "pid-controller.h"
 
 /*!
  * \brief Example of initializing and using a PID controller with error handling.
  * \details This function initializes a PID controller and uses error handling
  *      to ensure that the system behaves correctly even in the event of memory allocation failure.
  *      The PID controller is then used to simulate a simple control loop.
  */
 int main(void) {
     ArenaAllocatorHandler_t arena;
     PidController_t pid_controller;
     uint8_t n_prev_errors = 5;
 
     // Initialize the arena allocator
     arena_allocator_api_init(&arena);
 
     // Initialize the PID controller with error handling
     pid_init(&pid_controller, 1.0f, 0.5f, 0.1f, 0.1f, 10.0f, &arena, n_prev_errors);
 
     // Check if the PID controller was initialized successfully
     if (pid_controller.prev_errors == NULL) {
         // Handle memory allocation failure for previous errors
         printf("Error: Memory allocation failed for previous errors!\n");
         return -1;
     }
 
     // Set a new setpoint and simulate the PID control loop
     pid_controller.set_point = 100.0f;
     
     for (int i = 0; i < 100; i++) {
         float current_value = 90.0f + i;  // Simulated current value
         pid_controller.error = pid_controller.set_point - current_value;
         float control_signal = pid_calculate(&pid_controller);
         
         // Use the control signal (e.g., adjust motor speed)
     }
 
     // Clean up memory allocated for previous errors
     arena_allocator_api_free(&arena);
 
     return 0;
 }
 