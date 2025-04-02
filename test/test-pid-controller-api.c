/*!
 * \file test_pid_controller.c
 * \date 2025-03-25
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 *
 * \brief PID Controller Unit Tests.
 *
 * This file contains unit tests for the PID controller functions using the Unity test framework.
 * Each test function focuses on a single behavior so that the amount of code per test is minimal.
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <math.h>
 #include "unity.h"
 #include "pid-controller-api.h"
 
 /* Global instances for PID controller tests. */
 
 /*! \brief PID controller instance without previous error tracking (PI mode). */
 PidController_t pid;
 
 /*! \brief PID controller instance with previous error tracking (PID mode). */
 PidController_t pid_controller_api_prev_errors;
 
 /*! \brief Arena allocator instance for handling previous errors. */
 ArenaAllocatorHandler_t arena_instance;
 
 /* ===== SETUP and TEARDOWN ===== */
 
 /*!
  * \brief Set up the test environment before each test.
  *
  * Initializes the PID controllers and the arena allocator.
  */
 void setUp(void) {
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
 void tearDown(void) {
     pid_controller_api_reset(&pid);
     pid_controller_api_reset(&pid_controller_api_prev_errors);
     arena_allocator_api_free(&arena_instance);
 }
 
 /* ===== BASIC TESTS: Controller without previous error tracking (PI mode) ===== */
 
 /*!
  * \brief Test initialization of the PI controller.
  *
  * Verifies that all parameters are correctly initialized.
  */
 void test_pid_initialization_PI(void) {
     TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.kp);
     TEST_ASSERT_EQUAL_FLOAT(0.1f, pid.ki);
     TEST_ASSERT_EQUAL_FLOAT(0.01f, pid.kd);
     TEST_ASSERT_EQUAL_FLOAT(0.1f, pid.sample_time);
     TEST_ASSERT_EQUAL_FLOAT(10.0f, pid.anti_windUp);
     TEST_ASSERT_EQUAL_FLOAT(10.0f, pid.set_point);
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integrator);
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.error);
 }
 
 /*!
  * \brief Test update of the PI controller with input 8.0.
  *
  * Verifies that the error is calculated correctly.
  */
 void test_pid_update_PI_input8(void) {
     pid_controller_api_update(&pid, 8.0f);
     TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.error);
 }
 
 /*!
  * \brief Test update of the PI controller with input 9.0.
  *
  * Verifies that the error is calculated correctly.
  */
 void test_pid_update_PI_input9(void) {
     pid_controller_api_update(&pid, 9.0f);
     TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.error);
 }
 
 /*!
  * \brief Test compute function of the PI controller.
  *
  * Ensures that the computed output is positive.
  */
 void test_pid_compute_PI(void) {
     pid_controller_api_update(&pid, 8.0f);
     float output = pid_controller_api_compute(&pid);
     TEST_ASSERT(output > 0.0f);
 }
 
 /*!
  * \brief Test reset function of the PI controller.
  *
  * Ensures that all internal state variables are reset properly.
  */
 void test_pid_reset_PI(void) {
     pid_controller_api_update(&pid, 8.0f);
     pid_controller_api_reset(&pid);
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integrator);
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.error);
 }
 
 /* ===== EXTENDED TESTS: Controller with previous error tracking (PID mode) ===== */
 
 /*!
  * \brief Test initialization of the PID controller with previous error tracking.
  *
  * Verifies that all parameters, including the previous error buffer, are correctly initialized.
  */
 void test_pid_initialization_with_prev_errors(void) {
     TEST_ASSERT_EQUAL_FLOAT(1.0f, pid_controller_api_prev_errors.kp);
     TEST_ASSERT_EQUAL_FLOAT(0.1f, pid_controller_api_prev_errors.ki);
     TEST_ASSERT_EQUAL_FLOAT(0.01f, pid_controller_api_prev_errors.kd);
     TEST_ASSERT_EQUAL_FLOAT(0.1f, pid_controller_api_prev_errors.sample_time);
     TEST_ASSERT_EQUAL_FLOAT(10.0f, pid_controller_api_prev_errors.anti_windUp);
     TEST_ASSERT_EQUAL_FLOAT(10.0f, pid_controller_api_prev_errors.set_point);
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_controller_api_prev_errors.integrator);
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_controller_api_prev_errors.error);
     for (int i = 0; i < pid_controller_api_prev_errors.n_prev_errors; i++) {
         TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_controller_api_prev_errors.prev_errors[i]);
     }
 }
 
 /*!
  * \brief Test update of the PID controller with previous error tracking using input 8.0.
  *
  * Verifies that the error is calculated correctly and the previous error buffer is updated.
  */
 void test_pid_update_with_prev_errors_input8(void) {
     pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);
     TEST_ASSERT_EQUAL_FLOAT(2.0f, pid_controller_api_prev_errors.error);
     TEST_ASSERT(pid_controller_api_prev_errors.integrator > 0.0f);
     /* On first update, the previous error buffer should store the initial error (0.0) */
     uint8_t index = pid_controller_api_prev_errors.prev_error_index;
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_controller_api_prev_errors.prev_errors[index]);
 }
 
 /*!
  * \brief Test update of the PID controller with previous error tracking using input 9.0.
  *
  * Performs two updates and verifies that the error and previous error buffer are updated correctly.
  */
 void test_pid_update_with_prev_errors_input9(void) {
     /* First update with input 8.0 */
     pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);
     /* Second update with input 9.0 */
     pid_controller_api_update(&pid_controller_api_prev_errors, 9.0f);
     TEST_ASSERT_EQUAL_FLOAT(1.0f, pid_controller_api_prev_errors.error);
     uint8_t index = pid_controller_api_prev_errors.prev_error_index;
     /* The buffer at the current index should hold the error from the previous update (2.0) */
     TEST_ASSERT_EQUAL_FLOAT(2.0f, pid_controller_api_prev_errors.prev_errors[index]);
 }
 
 /*!
  * \brief Test update of the PID controller with previous error tracking using input 10.0.
  *
  * Performs three updates and verifies that the error becomes 0.0 and the previous error buffer is updated in a circular manner.
  */
 void test_pid_update_with_prev_errors_input10(void) {
     /* Update sequence: inputs 8.0, 9.0, then 10.0 */
     pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);   // error = 2.0
     pid_controller_api_update(&pid_controller_api_prev_errors, 9.0f);   // error = 1.0
     pid_controller_api_update(&pid_controller_api_prev_errors, 10.0f);  // error = 0.0
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_controller_api_prev_errors.error);
     /* Check that the buffer contains the previous errors in circular order */
     uint8_t index = pid_controller_api_prev_errors.prev_error_index;
     /* One of the positions should hold 1.0 (from the second update) */
     TEST_ASSERT((pid_controller_api_prev_errors.prev_errors[index] == 1.0f) ||
                 (pid_controller_api_prev_errors.prev_errors[(index+1) % pid_controller_api_prev_errors.n_prev_errors] == 1.0f));
 }
 
 /*!
  * \brief Test compute function of the PID controller with previous error tracking.
  *
  * Ensures that the computed output is positive after an update.
  */
 void test_pid_compute_with_prev_errors(void) {
     pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);
     float output = pid_controller_api_compute(&pid_controller_api_prev_errors);
     TEST_ASSERT(output > 0.0f);
 }
 
 /*!
  * \brief Test reset function of the PID controller with previous error tracking.
  *
  * Ensures that all internal state variables and the previous error buffer are reset properly.
  */
 void test_pid_reset_with_prev_errors(void) {
     pid_controller_api_update(&pid_controller_api_prev_errors, 8.0f);
     pid_controller_api_reset(&pid_controller_api_prev_errors);
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_controller_api_prev_errors.integrator);
     TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_controller_api_prev_errors.error);
     for (int i = 0; i < pid_controller_api_prev_errors.n_prev_errors; i++) {
         TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_controller_api_prev_errors.prev_errors[i]);
     }
 }

/* ===== Integrator clamping tests (requires previous error tracking) ===== */

/*!
 * \brief Test that the integrator is clamped (positive saturation).
 */
void test_pid_integrator_positive_clamp(void)
{
    // Initialize a PID controller with previous error tracking
    PidController_t pid_test;
    ArenaAllocatorHandler_t arena_dummy;
    arena_allocator_api_init(&arena_dummy);
    // Use n_prev_errors > 0 (e.g., 1) so that the anti-windup clamping code is executed.
    int ret = pid_controller_api_init(&pid_test, 
                        1.0f,    // kp
                        10.0f,   // ki (high to trigger clamping)
                        0.0f,    // kd
                        1.0f,    // sample_time
                        5.0f,    // anti_windUp threshold
                        &arena_dummy,
                        1);      // n_prev_errors = 1 (PID mode)
    TEST_ASSERT_EQUAL_INT(PID_OK, ret);
    
    pid_test.set_point = 100.0f; // set point far away to produce a large positive error

    // Force integrator to exceed anti_windUp by calling update once.
    // On first update: error = 100.0, so integrator becomes 100.
    pid_controller_api_update(&pid_test, 0.0f); 

    // The clamping code should set the integrator to anti_windUp / ki = 5/10 = 0.5.
    float product = pid_test.integrator * pid_test.ki;
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(
        5.0f, product,
        "Integrator was not clamped at the positive anti-windUp limit!"
    );

    arena_allocator_api_free(&arena_dummy);
}

/*!
 * \brief Test that the integrator is clamped (negative saturation).
 */
void test_pid_integrator_negative_clamp(void)
{
    // Initialize a PID controller with previous error tracking.
    PidController_t pid_test;
    ArenaAllocatorHandler_t arena_dummy;
    arena_allocator_api_init(&arena_dummy);
    // Use n_prev_errors > 0 (e.g., 1) so that the clamping code is executed.
    int ret = pid_controller_api_init(&pid_test, 
                        1.0f,    // kp
                        10.0f,   // ki
                        0.0f,    // kd
                        1.0f,    // sample_time
                        5.0f,    // anti_windUp threshold
                        &arena_dummy,
                        1);      // n_prev_errors = 1 (PID mode)
    TEST_ASSERT_EQUAL_INT(PID_OK, ret);
    
    pid_test.set_point = -100.0f; // set point far away to produce a large negative error

    // Force integrator to exceed negative anti_windUp by calling update once.
    // On first update: error = -100.0, so integrator becomes -100.
    pid_controller_api_update(&pid_test, 0.0f);

    // The clamping code should set the integrator to -anti_windUp / ki = -5/10 = -0.5.
    float product = pid_test.integrator * pid_test.ki;
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT_MESSAGE(
        -5.0f, product,
        "Integrator was not clamped at the negative anti-windUp limit!"
    );

    arena_allocator_api_free(&arena_dummy);
}

/*!
 * \brief Test that the integrator is not clamped when ki is small.
 *
 * This test checks that the anti-windup code is not executed when ki is very small.
 */

void test_pid_no_antiwindup_when_ki_is_small(void)
{
    PidController_t pid_test;
    ArenaAllocatorHandler_t arena_dummy;
    arena_allocator_api_init(&arena_dummy);

    // ki is very small, so the if(fabs(ki) > 0.0001f) block is not entered
    int ret = pid_controller_api_init(&pid_test,
                                  1.0f,      // kp
                                  0.00001f,  // ki
                                  0.0f,      // kd
                                  1.0f,      // sample_time
                                  5.0f,      // anti_windUp
                                  &arena_dummy,
                                  1);        // n_prev_errors > 0
    TEST_ASSERT_EQUAL_INT(PID_OK, ret);

    pid_test.set_point = 100.0f;
    // This update won't clamp integrator because the code is in the else path
    // where (fabs(ki) <= 0.0001f).
    pid_controller_api_update(&pid_test, 0.0f);

    // Check integrator is not clamped
    // integrator should be 100 in this one-step scenario
    TEST_ASSERT_EQUAL_FLOAT(100.0f, pid_test.integrator);

    arena_allocator_api_free(&arena_dummy);
}

 /* ===== ERROR CASES: Invalid Initialization ===== */
 
 /*!
  * \brief Test PID controller initialization with a NULL pointer.
  *
  * Verifies that initialization fails if the controller pointer is NULL.
  */
 void test_pid_init_null_ptr(void) {
     int ret = pid_controller_api_init(NULL, 1.0f, 0.1f, 0.01f, 0.1f, 10.0f, NULL, 0);
     TEST_ASSERT_EQUAL_INT(PID_NULL_POINTER, ret);
 }
 
 /*!
  * \brief Test PID controller initialization with a NULL arena when previous errors are requested.
  *
  * Verifies that initialization fails if previous error tracking is requested without a valid allocator.
  */
 void test_pid_init_null_arena(void) {
     int ret = pid_controller_api_init(&pid, 1.0f, 0.1f, 0.01f, 0.1f, 10.0f, NULL, 3);
     TEST_ASSERT_EQUAL_INT(PID_NULL_POINTER, ret);
 }

 /* ===== MAIN FUNCTION TO RUN THE TESTS ===== */
 
 /*!
  * \brief Main function for running all PID unit tests.
  *
  * Executes all defined tests using the Unity test framework.
  *
  * \return Unity test results.
  */
 int main(void) {
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

     RUN_TEST(test_pid_integrator_positive_clamp);
     RUN_TEST(test_pid_integrator_negative_clamp);
     RUN_TEST(test_pid_no_antiwindup_when_ki_is_small);

 
     /* Error case tests */
     RUN_TEST(test_pid_init_null_ptr);
     RUN_TEST(test_pid_init_null_arena);
 
     return UNITY_END();
 }
 