/*!
 * \file pi.c
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 * \date 2025-03-31
 *
 * \brief Example usage of a PI controller without dynamic memory allocation.
 * 
 * This example demonstrates how to initialize and use a PI controller in a simplified 
 * configuration where no previous error values are stored (n_prev_errors = 0). 
 * The integral term is computed directly without requiring an external memory allocator. 
 * Error handling is implemented to ensure robustness during initialization.
 */

#include <stdio.h>
#include <stdlib.h>
#include "pid-controller-api.h"

/*!
  * \brief Main function to demonstrate the PID controller.
  *
  * \return 0 if execution is successful, -1 if an error occurs.
  */
int main(void) {
    ArenaAllocatorHandler_t arena; /*!< Dummy instance of the allocator. */
    struct PidController pi;       /*!< PI controller instance. */

    arena_allocator_api_init(&arena); /*!< Initialize the arena allocator. */

    /*!
      * \brief Initialize the PI controller.
      *
      * The controller is initialized with a sample time of 0.1s.
      *
      * \retval PID_OK if initialization is successful.
      * \retval PID_NULL_POINTER if a null pointer is encountered.
      * \retval PID_ALLOCATION_ERROR if memory allocation fails.
      */
    int ret = pid_controller_api_init(&pi,
                                      1.0f,  /*!< Proportional gain (Kp) */
                                      0.1f,  /*!< Integral gain (Ki) */
                                      0.01f, /*!< Derivative gain (Kd), redoundant in PI mode*/
                                      0.1f,  /*!< Sample time in seconds */
                                      10.0f, /*!< Anti-windup limit */
                                      NULL,  /*!< You can pass NULL parameter to use the PI mode */
                                      0      /*!< You must pass 0 to use the PI mode */
    );
    if (ret != PID_OK) {
        /*!
          * \brief Error handling based on return code.
          */
        switch (ret) {
            case PID_NULL_POINTER:
                printf("Error: null pointer encountered.\n");
                break;
            case PID_ALLOCATION_ERROR:
                printf("Error: Memory allocation failed.\n");
                break;
            default:
                printf("Unknown error occurred.\n");
        }
        return -1;
    }

    /*!
      * \brief Set the desired setpoint for the PI controller.
      */
    pi.set_point = 10.0f;
    float process_var = 0.0f; /*!< Initial process variable value. */

    /*!
      * \brief Simulation loop for the PI controller.
      *
      * Updates the controller and computes control signals over 100 iterations.
      */
    for (int i = 0; i < 100; i++) {
        pid_controller_api_update(&pi, process_var);            /*!< Update the PID controller with the current process value. */
        float control_signal = pid_controller_api_compute(&pi); /*!< Compute the control signal. */

        /*!
          * \brief Simulate a simple system response.
          *
          * The process variable is updated using the computed control signal.
          */
        process_var += control_signal * 0.1f;

        printf("Iteration %d: Control = %f, Process = %f\n", i, control_signal, process_var);
    }

    arena_allocator_api_free(&arena); /*!< Free the memory allocated by the arena allocator. */

    return 0;
}
