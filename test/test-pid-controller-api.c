 /*!
 * \file test-pid-controller-api.c
 * \date 2025-03-25
 * \author Valerio Cancemi [valerio.cancemi04@gmail.com]
 *
 * \brief Unit tests for the PID Controller API.
 *
 * This file contains unit tests for the PID controller implementation, 
 * ensuring correct initialization, update, computation, and reset functionality.
 */

 #include <unity.h>
 #include "pid-controller-api.h"
 
 /*! \brief PID controller instance without previous errors tracking. */
 PidController_t pid;
 
 /*! \brief PID controller instance with previous errors tracking. */
 PidController_t pid_prev_errors;
 
 /*! \brief Arena allocator instance for handling previous errors. */
 ArenaAllocatorHandler_t prev_errors;
 
 /*!
  * \brief Set up the test environment before each test.
  *
  * Initializes the PID controllers and the arena allocator.
  */
 void setUp(void) {
     arena_allocator_api_init(&prev_errors);
     pid_init(&pid, 1.0, 0.1, 0.01, 0.1, 10.0, NULL, 0);
     pid_init(&pid_prev_errors, 1.0, 0.1, 0.01, 0.1, 10.0, &prev_errors, 3);
     pid.set_point = 10.0;
     pid_prev_errors.set_point = 10.0;
 }
 
 /*!
  * \brief Clean up after each test.
  *
  * Resets the PID controllers.
  */
 void tearDown() {
     pid_reset(&pid);
     pid_reset(&pid_prev_errors);
 }
 
 /*!
  * \defgroup BasicTests Basic PID Controller Tests
  * \brief Tests for the PID controller without previous error tracking.
  * @{
  */
 
 /*!
  * \brief Test PID controller initialization.
  *
  * Ensures that all PID parameters are correctly initialized.
  */
 void test_pid_initialization() {
     TEST_ASSERT_EQUAL_FLOAT(1.0, pid.kp);
     TEST_ASSERT_EQUAL_FLOAT(0.1, pid.ki);
     TEST_ASSERT_EQUAL_FLOAT(0.01, pid.kd);
     TEST_ASSERT_EQUAL_FLOAT(0.1, pid.sample_time);
     TEST_ASSERT_EQUAL_FLOAT(10.0, pid.anti_windUp);
     TEST_ASSERT_EQUAL_FLOAT(10.0, pid.set_point);
     TEST_ASSERT_EQUAL_FLOAT(0.0, pid.integrator);
     TEST_ASSERT_EQUAL_FLOAT(0.0, pid.error);
 }
 
 /*!
  * \brief Test PID update function.
  *
  * Ensures that error calculation and integrator behavior are correct.
  */
 void test_pid_update() {
     pid_update(&pid, 8.0);
     TEST_ASSERT_EQUAL_FLOAT(2.0, pid.error);
     TEST_ASSERT(pid.integrator > 0);
     
     pid_update(&pid, 9.0);
     TEST_ASSERT_EQUAL_FLOAT(1.0, pid.error);
 }
 
 /*! @} */
 
 /*!
  * \defgroup ExtendedTests PID Controller Tests with Previous Errors
  * \brief Tests for the PID controller with previous error tracking.
  * @{
  */
 
 /*!
  * \brief Test PID initialization with previous error tracking.
  *
  * Ensures that all parameters, including previous errors, are initialized correctly.
  */
 void test_pid_initialization_with_prev_errors() {
     TEST_ASSERT_EQUAL_FLOAT(1.0, pid_prev_errors.kp);
     TEST_ASSERT_EQUAL_FLOAT(0.1, pid_prev_errors.ki);
     TEST_ASSERT_EQUAL_FLOAT(0.01, pid_prev_errors.kd);
     TEST_ASSERT_EQUAL_FLOAT(0.1, pid_prev_errors.sample_time);
     TEST_ASSERT_EQUAL_FLOAT(10.0, pid_prev_errors.anti_windUp);
     TEST_ASSERT_EQUAL_FLOAT(10.0, pid_prev_errors.set_point);
     TEST_ASSERT_EQUAL_FLOAT(0.0, pid_prev_errors.integrator);
     TEST_ASSERT_EQUAL_FLOAT(0.0, pid_prev_errors.error);
 
     for (int i = 0; i < pid_prev_errors.n_prev_errors; i++) {
         TEST_ASSERT_EQUAL_FLOAT(0.0, pid_prev_errors.prev_errors[i]);
     }
 }
 
 /*!
  * \brief Test PID update with previous error tracking.
  *
  * Ensures correct error calculation and shifting of previous errors.
  */
 void test_pid_update_with_prev_errors() {
     pid_update(&pid_prev_errors, 8.0);
     TEST_ASSERT_EQUAL_FLOAT(2.0, pid_prev_errors.error);
     TEST_ASSERT(pid_prev_errors.integrator > 0);
 
     pid_update(&pid_prev_errors, 9.0);
     TEST_ASSERT_EQUAL_FLOAT(1.0, pid_prev_errors.error);
     TEST_ASSERT_EQUAL_FLOAT(2.0, pid_prev_errors.prev_errors[pid_prev_errors.prev_error_index]);
 
     pid_update(&pid_prev_errors, 10.0);
     TEST_ASSERT_EQUAL_FLOAT(0.0, pid_prev_errors.error);
     TEST_ASSERT_EQUAL_FLOAT(1.0, pid_prev_errors.prev_errors[pid_prev_errors.prev_error_index]);
     TEST_ASSERT_EQUAL_FLOAT(2.0, pid_prev_errors.prev_errors[pid_prev_errors.prev_error_index - 1]);
     TEST_ASSERT(pid_prev_errors.integrator > 0);
 }
 
 /*! @} */
 
 /*!
  * \defgroup ComputationTests PID Computation and Reset Tests
  * \brief Tests for PID computation and reset behavior.
  * @{
  */
 
 /*!
  * \brief Test PID compute function.
  *
  * Ensures that the computed output is positive after an update.
  */
 void test_pid_compute() {
     pid_update(&pid, 8.0);
     float output = pid_compute(&pid);
     TEST_ASSERT(output > 0);
 }
 
 /*!
  * \brief Test PID compute function with previous error tracking.
  *
  * Ensures that the computed output is positive after an update.
  */
 void test_pid_compute_with_prev_errors() {
     pid_update(&pid_prev_errors, 8.0);
     float output = pid_compute(&pid_prev_errors);
     TEST_ASSERT(output > 0);
 }
 
 /*!
  * \brief Test PID reset function.
  *
  * Ensures that all internal state variables are reset properly.
  */
 void test_pid_reset() {
     pid_update(&pid, 8.0);
     pid_reset(&pid);
     TEST_ASSERT_EQUAL_FLOAT(0.0, pid.integrator);
     TEST_ASSERT_EQUAL_FLOAT(0.0, pid.error);
 }
 
 /*!
  * \brief Test PID reset function with previous error tracking.
  *
  * Ensures that all internal state variables and previous errors are reset properly.
  */
 void test_pid_reset_with_prev_errors() {
     pid_update(&pid_prev_errors, 8.0);
     pid_reset(&pid_prev_errors);
     TEST_ASSERT_EQUAL_FLOAT(0.0, pid_prev_errors.integrator);
     TEST_ASSERT_EQUAL_FLOAT(0.0, pid_prev_errors.error);
 
     for (int i = 0; i < pid_prev_errors.n_prev_errors; i++) {
         TEST_ASSERT_EQUAL_FLOAT(0.0, pid_prev_errors.prev_errors[i]);
     }
 }
 
 /*! @} */
 
 /*!
  * \brief Main function for running all PID unit tests.
  *
  * Executes all defined tests using Unity test framework.
  *
  * \return Unity test results.
  */
 int main() {
     UNITY_BEGIN();
 
     /* Run basic tests */
     RUN_TEST(test_pid_initialization);
     RUN_TEST(test_pid_update);
     RUN_TEST(test_pid_compute);
     RUN_TEST(test_pid_reset);
 
     /* Run extended tests */
     RUN_TEST(test_pid_initialization_with_prev_errors);
     RUN_TEST(test_pid_update_with_prev_errors);
     RUN_TEST(test_pid_compute_with_prev_errors);
     RUN_TEST(test_pid_reset_with_prev_errors);
 
     /* Free memory allocated by the arena allocator */
     arena_allocator_api_free(&prev_errors);
 
     return UNITY_END();
 }
 