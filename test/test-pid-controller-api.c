/*!
 * \file test_pid_controller.c
 * \date 2025-03-25
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 *
 * \brief PID Controller Unit Tests.
 *
 * This file contains unit tests for the PID controller functions using the Unity test framework.
 * Tests are organized by groups and each test checks only one action.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "unity.h"
#include "pid-controller-api.h"

/* Global instances for PID controller tests. */
PidController_t pid;
PidController_t pid_controller_api_prev_errors;
ArenaAllocatorHandler_t arena_instance;

/* ===== SETUP and TEARDOWN ===== */

/*!
 * \brief Set up the test environment before each test.
 *
 * Initializes the PID controllers and the arena allocator.
 */
void setUp(void)
{
    arena_allocator_api_init(&arena_instance);
    /* Initialize PI mode (n_prev_errors = 0) */
    int ret1 = pid_controller_api_init(&pid, 1.0f, 0.1f, 0.01f, 0.1f, 10.0f, NULL, 0);
    TEST_ASSERT_EQUAL_INT(PID_OK, ret1);
    pid.set_point = 10.0f;
    
    /* Initialize PID mode (n_prev_errors = 3) */
    int ret2 = pid_controller_api_init(&pid_controller_api_prev_errors, 1.0f, 0.1f, 0.01f, 0.1f, 10.0f, &arena_instance, 3);
    TEST_ASSERT_EQUAL_INT(PID_OK, ret2);
    pid_controller_api_prev_errors.set_point = 10.0f;
}

/*!
 * \brief Clean up after each test.
 *
 * Resets the PID controllers and frees memory allocated by the arena allocator.
 */
void tearDown(void)
{
    pid_controller_api_reset(&pid);
    pid_controller_api_reset(&pid_controller_api_prev_errors);
    arena_allocator_api_free(&arena_instance);
}


/*============================================================================*/
/*                           PI MODE TESTS                                    */
/*============================================================================*/

/** @defgroup PID_PI_Tests PI Mode Tests
 *  @{
 */

/*!
 * \brief Test initialization of the PI controller by comparing all fields in one assert.
 */
void test_pid_initialization_PI(void)
{
    /* Expected state for PI mode after initialization */
    PidController_t expected =
    {
        .kp = 1.0f,
        .ki = 0.1f,
        .kd = 0.01f,
        .sample_time = 0.1f,
        .anti_windUp = 10.0f,
        .set_point = 10.0f,
        .integrator = 0.0f,
        .error = 0.0f,
        .prev_errors = NULL,
        .n_prev_errors = 0,
        .prev_error_index = 0
    };
    TEST_ASSERT_EQUAL_MEMORY(&expected, &pid, sizeof(PidController_t));
}

/*!
 * \brief Test update of the PI controller using input 8.0.
 *
 * Checks that the error field is updated correctly.
 */
void test_pid_update_PI_input8(void)
{
    pid_controller_api_update(&pid, 8.0f);
    float expected_error = 2.0f;
    TEST_ASSERT_EQUAL_FLOAT(expected_error, pid.error);
}

/*!
 * \brief Test update of the PI controller using input 9.0.
 *
 * Checks that the error field is updated correctly.
 */
void test_pid_update_PI_input9(void)
{
    pid_controller_api_update(&pid, 9.0f);
    float expected_error = 1.0f;
    TEST_ASSERT_EQUAL_FLOAT(expected_error, pid.error);
}

/*!
 * \brief Test compute function of the PI controller.
 *
 * Ensures that the computed output is positive.
 */
void test_pid_compute_PI(void)
{
    pid_controller_api_update(&pid, 8.0f);
    float output = pid_controller_api_compute(&pid);
    TEST_ASSERT(output > 0.0f);
}

/*!
 * \brief Test reset function of the PI controller.
 *
 * Verifies that integrator and error are reset.
 */
void test_pid_reset_PI(void)
{
    pid_controller_api_update(&pid, 8.0f);
    pid_controller_api_reset(&pid);
    /* Expected state after reset (only checking two fields) */
    PidController_t expected = pid;
    expected.integrator = 0.0f;
    expected.error = 0.0f;
    TEST_ASSERT_EQUAL_MEMORY(&expected, &pid, sizeof(PidController_t));
}

/** @} */  // end of PID_PI_Tests


/*============================================================================*/
/*                    PID MODE (PREVIOUS ERRORS) TESTS                        */
/*============================================================================*/

/** @defgroup PID_PrevErrors_Tests PID Mode with Previous Errors Tests
 *  @{
 */

/*!
 * \brief Test initialization of the PID controller with previous error tracking.
 *
 * Compares the entire structure with the expected values.
 */
void test_pid_initialization_with_prev_errors(void)
{
    PidController_t expected =
    {
        .kp = 1.0f, 
        .ki = 0.1f, 
        .kd = 0.01f, 
        .sample_time = 0.1f, 
        .anti_windUp = 10.0f, 
        .set_point = 10.0f, 
        .integrator = 0.0f, 
        .error = 0.0f, 
        .prev_errors = pid_controller_api_prev_errors.prev_errors, /* pointer same as allocated */
        .n_prev_errors = 3,
        .prev_error_index = 2
    };


    TEST_ASSERT_EQUAL_MEMORY(&expected, &pid_controller_api_prev_errors, sizeof(PidController_t));
}

/*!
 * \brief Test update of the PID controller with previous error tracking using input 8.0.
 *
 * Checks that the error field is updated.
 */
void test_pid_update_with_prev_errors_input8(void)
{
    pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);
    float expected_error = 2.0f;
    TEST_ASSERT_EQUAL_FLOAT(expected_error, pid_controller_api_prev_errors.error);
}

/*!
 * \brief Test update of the PID controller with previous error tracking using input 9.0.
 *
 * Performs two updates and confirms that the last error is as expected.
 */
void test_pid_update_with_prev_errors_input9(void)
{
    pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);
    pid_controller_api_update(&pid_controller_api_prev_errors, 9.0f);
    float expected_error = 1.0f;
    TEST_ASSERT_EQUAL_FLOAT(expected_error, pid_controller_api_prev_errors.error);
}

/*!
 * \brief Test update sequence with inputs 8.0, 9.0, and 10.0.
 *
 * Verifies that the current error becomes 0.0 and the previous error buffer is updated 
 * in a circular manner.
 */
void test_pid_update_with_prev_errors_input10(void)
{
    pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);   // error = 2.0
    pid_controller_api_update(&pid_controller_api_prev_errors, 9.0f);   // error = 1.0
    pid_controller_api_update(&pid_controller_api_prev_errors, 10.0f);  // error = 0.0
    float expected_error = 0.0f;
    TEST_ASSERT_EQUAL_FLOAT(expected_error, pid_controller_api_prev_errors.error);
}

/*!
 * \brief Test compute function of the PID controller with previous error tracking.
 *
 * Ensures that the computed output is positive.
 */
void test_pid_compute_with_prev_errors(void)
{
    pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);
    float output = pid_controller_api_compute(&pid_controller_api_prev_errors);
    TEST_ASSERT(output > 0.0f);
}

/*!
 * \brief Test reset function of the PID controller with previous error tracking.
 *
 * Checks that integrator, error, and the previous error buffer are reset.
 */
void test_pid_reset_with_prev_errors(void)
{
    pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);
    pid_controller_api_reset(&pid_controller_api_prev_errors);

    /* Prepare an expected structure with reset values. For the previous errors array we assume all zeros. */
    PidController_t expected = pid_controller_api_prev_errors;
    expected.integrator = 0.0f;
    expected.error = 0.0f;
    for (int i = 0; i < expected.n_prev_errors; i++)
        ((float *)expected.prev_errors)[i] = 0.0f;
    TEST_ASSERT_EQUAL_MEMORY(&expected, &pid_controller_api_prev_errors, sizeof(PidController_t));
}

/** @} */  // end of PID_PrevErrors_Tests


/*============================================================================*/
/*                     INTEGRATOR CLAMPING TESTS                              */
/*============================================================================*/

/** @defgroup PID_Clamping_Tests Integrator Clamping Tests
 *  @{
 */

/*!
 * \brief Test that the integrator is clamped (positive saturation).
 */
void test_pid_integrator_positive_clamp(void)
{
    PidController_t pid_test;
    ArenaAllocatorHandler_t arena_dummy;
    arena_allocator_api_init(&arena_dummy);
    int ret = pid_controller_api_init(&pid_test, 
                          1.0f,    // kp
                          10.0f,   // ki (high to trigger clamping)
                          0.0f,    // kd
                          1.0f,    // sample_time
                          5.0f,    // anti_windUp threshold
                          &arena_dummy,
                          1);      // n_prev_errors = 1 (PID mode)
    TEST_ASSERT_EQUAL_INT(PID_OK, ret);
    
    pid_test.set_point = 100.0f;
    pid_controller_api_update(&pid_test, 0.0f);
    
    /* Expected clamped integrator: antiWindup/ki = 5/10 = 0.5 */
    float expected_integrator = 0.5f;
    TEST_ASSERT_EQUAL_FLOAT(expected_integrator, pid_test.integrator);
    
    arena_allocator_api_free(&arena_dummy);
}

/*!
 * \brief Test that the integrator is clamped (negative saturation).
 */
void test_pid_integrator_negative_clamp(void)
{
    PidController_t pid_test;
    ArenaAllocatorHandler_t arena_dummy;
    arena_allocator_api_init(&arena_dummy);
    int ret = pid_controller_api_init(&pid_test, 
                          1.0f,    // kp
                          10.0f,   // ki
                          0.0f,    // kd
                          1.0f,    // sample_time
                          5.0f,    // anti_windUp threshold
                          &arena_dummy,
                          1);      // n_prev_errors = 1
    TEST_ASSERT_EQUAL_INT(PID_OK, ret);
    
    pid_test.set_point = -100.0f;
    pid_controller_api_update(&pid_test, 0.0f);
    
    /* Expected clamped integrator: -antiWindup/ki = -5/10 = -0.5 */
    float expected_integrator = -0.5f;
    TEST_ASSERT_EQUAL_FLOAT(expected_integrator, pid_test.integrator);
    
    arena_allocator_api_free(&arena_dummy);
}

/*!
 * \brief Test that anti-windup is not applied when ki is very small.
 */
void test_pid_no_antiwindup_when_ki_is_small(void)
{
    PidController_t pid_test;
    ArenaAllocatorHandler_t arena_dummy;
    arena_allocator_api_init(&arena_dummy);
    
    int ret = pid_controller_api_init(&pid_test,
                              1.0f,      // kp
                              0.00001f,  // ki (very small)
                              0.0f,      // kd
                              1.0f,      // sample_time
                              5.0f,      // anti_windUp
                              &arena_dummy,
                              1);
    TEST_ASSERT_EQUAL_INT(PID_OK, ret);
    
    pid_test.set_point = 100.0f;
    pid_controller_api_update(&pid_test, 0.0f);
    
    /* With no clamping, the integrator should equal the error (100.0). */
    TEST_ASSERT_EQUAL_FLOAT(100.0f, pid_test.integrator);
    
    arena_allocator_api_free(&arena_dummy);
}

/** @} */  // end of PID_Clamping_Tests


/*============================================================================*/
/*                             ERROR CASE TESTS                               */
/*============================================================================*/

/** @defgroup PID_Error_Tests Error Case Tests
 *  @{
 */

/*!
 * \brief Test PID controller initialization with a NULL pointer.
 */
void test_pid_init_null_ptr(void)
{
    int ret = pid_controller_api_init(NULL, 1.0f, 0.1f, 0.01f, 0.1f, 10.0f, NULL, 0);
    TEST_ASSERT_EQUAL_INT(PID_NULL_POINTER, ret);
}

/*!
 * \brief Test PID controller initialization with a NULL arena when previous error tracking is requested.
 */
void test_pid_init_null_arena(void)
{
    int ret = pid_controller_api_init(&pid, 1.0f, 0.1f, 0.01f, 0.1f, 10.0f, NULL, 3);
    TEST_ASSERT_EQUAL_INT(PID_NULL_POINTER, ret);
}

/** @} */  // end of PID_Error_Tests


/*============================================================================*/
/*                      MAIN FUNCTION TO RUN THE TESTS                        */
/*============================================================================*/

/*!
 * \brief Main function for running all PID unit tests.
 *
 * Executes all defined tests using the Unity test framework.
 *
 * \return Unity test results.
 */
int main(void)
{
    UNITY_BEGIN();

    /* PI mode tests */
    RUN_TEST(test_pid_initialization_PI);
    RUN_TEST(test_pid_update_PI_input8);
    RUN_TEST(test_pid_update_PI_input9);
    RUN_TEST(test_pid_compute_PI);
    RUN_TEST(test_pid_reset_PI);

    /* PID mode tests (with previous error tracking) */
    RUN_TEST(test_pid_initialization_with_prev_errors);
    RUN_TEST(test_pid_update_with_prev_errors_input8);
    RUN_TEST(test_pid_update_with_prev_errors_input9);
    RUN_TEST(test_pid_update_with_prev_errors_input10);
    RUN_TEST(test_pid_compute_with_prev_errors);
    RUN_TEST(test_pid_reset_with_prev_errors);

    /* Integrator clamping tests */
    RUN_TEST(test_pid_integrator_positive_clamp);
    RUN_TEST(test_pid_integrator_negative_clamp);
    RUN_TEST(test_pid_no_antiwindup_when_ki_is_small);

    /* Error case tests */
    RUN_TEST(test_pid_init_null_ptr);
    RUN_TEST(test_pid_init_null_arena);

    return UNITY_END();
}