/* test_elib_traj.c - Trajectory Planner Unit Tests */
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include "../include/elib_traj_defs.h"
#include "../src/elib_traj_util.h"

#define EPSILON 1e-4f

static int tests_run = 0;
static int tests_passed = 0;

#define RUN_TEST(fn) do { \
    printf("Test: %s... ", #fn); \
    tests_run++; \
    fn(); \
    tests_passed++; \
    printf("PASSED\n"); \
} while(0)

static void test_clamp_mid(void) {
    elib_traj_val_t result = elib_traj_util_clamp(5.0f, 0.0f, 10.0f);
    assert(fabsf(result - 5.0f) < EPSILON);
}

static void test_clamp_below_min(void) {
    elib_traj_val_t result = elib_traj_util_clamp(-5.0f, 0.0f, 10.0f);
    assert(fabsf(result - 0.0f) < EPSILON);
}

static void test_clamp_above_max(void) {
    elib_traj_val_t result = elib_traj_util_clamp(15.0f, 0.0f, 10.0f);
    assert(fabsf(result - 10.0f) < EPSILON);
}

static void test_fabs_val_positive(void) {
    elib_traj_val_t result = elib_traj_util_fabs(3.5f);
    assert(fabsf(result - 3.5f) < EPSILON);
}

static void test_fabs_val_negative(void) {
    elib_traj_val_t result = elib_traj_util_fabs(-3.5f);
    assert(fabsf(result - 3.5f) < EPSILON);
}

static void test_fabs_val_zero(void) {
    elib_traj_val_t result = elib_traj_util_fabs(0.0f);
    assert(fabsf(result - 0.0f) < EPSILON);
}

int main(void) {
    printf("=== elib-trajectory tests ===\n\n");

    printf("--- Utility functions ---\n");
    RUN_TEST(test_clamp_mid);
    RUN_TEST(test_clamp_below_min);
    RUN_TEST(test_clamp_above_max);
    RUN_TEST(test_fabs_val_positive);
    RUN_TEST(test_fabs_val_negative);
    RUN_TEST(test_fabs_val_zero);

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
