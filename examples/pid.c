/*!
 * \file example-pid-controller.c
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 * \date 2025-03-25
 *
 * \brief Example usage of a PID controller with Arena allocator.
 * 
 * This example demonstrates how to initialize and use a PID controller with dynamic 
 * memory allocation for storing previous error values. It includes robust error handling 
 * for both the PID controller initialization and memory allocation, ensuring stability 
 * in case of allocation failures.
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include "pid-controller-api.h"
 
 /**
  * \brief Main function to demonstrate the PID controller.
  *
  * \return 0 if execution is successful, -1 if an error occurs.
  */
 int main(void) {
     ArenaAllocatorHandler_t arena; /**< Dummy instance of the allocator. */
     PidController_t pid; /**< PID controller instance. */
 
     arena_allocator_api_init(&arena); /**< Initialize the arena allocator. */
 
     /**
      * \brief Initialize the PID controller.
      *
      * The controller is initialized with a sample time of 0.1s and
      * a previous error history of 5.
      *
      * \retval PID_SUCCESS if initialization is successful.
      * \retval PID_ERROR_NULL_PTR if a null pointer is encountered.
      * \retval PID_ERROR_MEM_ALLOC if memory allocation fails.
      */
     int ret = pid_controller_api_init(&pid, 
                                   1.0f,   /**< Proportional gain (Kp) */
                                   0.1f,   /**< Integral gain (Ki) */
                                   0.05f,  /**< Derivative gain (Kd) */
                                   0.1f,   /**< Sample time in seconds */
                                   10.0f,  /**< Anti-windup limit */
                                   &arena, /**< Arena allocator */
                                   5       /**< Number of previous errors stored */
                                   );
     if (ret != PID_SUCCESS) {
         /**
          * \brief Error handling based on return code.
          */
         switch (ret) {
             case PID_ERROR_NULL_PTR:
                 printf("Error: null pointer encountered.\n");
                 break;
             case PID_ERROR_MEM_ALLOC:
                 printf("Error: Memory allocation failed.\n");
                 break;
             default:
                 printf("Unknown error occurred.\n");
         }
         return -1;
     }
 
     /**
      * \brief Set the desired setpoint for the PID controller.
      */
     pid.set_point = 10.0f;
     float process_var = 0.0f; /**< Initial process variable value. */
 
     /**
      * \brief Simulation loop for the PID controller.
      *
      * Updates the controller and computes control signals over 100 iterations.
      */
     for (int i = 0; i < 100; i++) {
         pid_controller_api_update(&pid, process_var); /**< Update the PID controller with the current process value. */
         float control_signal = pid_controller_api_compute(&pid); /**< Compute the control signal. */
 
         /**
          * \brief Simulate a simple system response.
          *
          * The process variable is updated using the computed control signal.
          */
         process_var += control_signal * 0.1f;
 
         printf("Iteration %d: Control = %f, Process = %f\n", i, control_signal, process_var);
     }
 
     arena_allocator_api_free(&arena); /**< Free the memory allocated by the arena allocator. */
 
     return 0;
 }
 