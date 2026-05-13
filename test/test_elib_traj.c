/* test_elib_traj.c - Trajectory Planner Unit Tests */
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include "../include/elib_traj_defs.h"
#include "../include/elib_traj_trapezoid.h"
#include "../include/elib_traj_scurve.h"
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

/* === Utility function tests === */
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

/* === Trapezoidal profile tests === */
static elib_traj_trapezoid_params_t make_trap_params(void) {
    elib_traj_trapezoid_params_t p = {
        .max_vel = 1000.0f, .acc = 2000.0f, .dec = 2000.0f, .target_pos = 1000.0f,
    };
    return p;
}

static void test_trap_init_valid(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    assert(elib_traj_trapezoid_init(&ctx, &p, 0.0f) == ELIB_TRAJ_TRAPEZOID_OK);
    assert(ctx.initialized == 1);
}
static void test_trap_init_null_ctx(void) {
    elib_traj_trapezoid_params_t p = make_trap_params();
    assert(elib_traj_trapezoid_init(NULL, &p, 0.0f) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_init_null_params(void) {
    elib_traj_trapezoid_ctx_t ctx;
    assert(elib_traj_trapezoid_init(&ctx, NULL, 0.0f) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_init_bad_max_vel(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.max_vel = 0.0f;
    assert(elib_traj_trapezoid_init(&ctx, &p, 0.0f) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_init_bad_acc(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.acc = -1.0f;
    assert(elib_traj_trapezoid_init(&ctx, &p, 0.0f) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_init_bad_dec(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.dec = 0.0f;
    assert(elib_traj_trapezoid_init(&ctx, &p, 0.0f) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_update_not_initialized(void) {
    elib_traj_trapezoid_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    assert(elib_traj_trapezoid_update(&ctx, 0.01f) == ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED);
}
static void test_trap_update_bad_dt(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(elib_traj_trapezoid_update(&ctx, 0.0f) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_get_state_null(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(elib_traj_trapezoid_get_state(&ctx, NULL) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_get_status_null(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(elib_traj_trapezoid_get_status(&ctx, NULL) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_zero_distance(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.target_pos = 50.0f;
    elib_traj_trapezoid_init(&ctx, &p, 50.0f);
    elib_traj_status_t status;
    elib_traj_trapezoid_get_status(&ctx, &status);
    assert(status == ELIB_TRAJ_STATUS_FINISHED);
    elib_traj_state_t state;
    elib_traj_trapezoid_get_state(&ctx, &state);
    assert(fabsf(state.pos - 50.0f) < EPSILON);
    assert(fabsf(state.vel - 0.0f) < EPSILON);
}
static void test_trap_normal_profile(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(fabsf(ctx.t_total - 1.5f) < EPSILON);
    elib_traj_trapezoid_update(&ctx, 0.25f);
    elib_traj_state_t state;
    elib_traj_trapezoid_get_state(&ctx, &state);
    assert(fabsf(state.vel - 500.0f) < EPSILON);
    assert(fabsf(state.pos - 62.5f) < EPSILON);
    assert(fabsf(state.acc - 2000.0f) < EPSILON);
}
static void test_trap_reaches_target(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    elib_traj_trapezoid_update(&ctx, 2.0f);
    elib_traj_state_t state;
    elib_traj_trapezoid_get_state(&ctx, &state);
    assert(fabsf(state.pos - 1000.0f) < EPSILON);
    assert(fabsf(state.vel - 0.0f) < EPSILON);
    elib_traj_status_t status;
    elib_traj_trapezoid_get_status(&ctx, &status);
    assert(status == ELIB_TRAJ_STATUS_FINISHED);
}
static void test_trap_negative_direction(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.target_pos = -1000.0f;
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    elib_traj_trapezoid_update(&ctx, 0.25f);
    elib_traj_state_t state;
    elib_traj_trapezoid_get_state(&ctx, &state);
    assert(state.vel < 0.0f);
}
static void test_trap_triangle_profile(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.target_pos = 100.0f;
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(ctx.t_const < EPSILON);
    elib_traj_trapezoid_update(&ctx, 1.0f);
    elib_traj_state_t state;
    elib_traj_trapezoid_get_state(&ctx, &state);
    assert(fabsf(state.pos - 100.0f) < EPSILON);
    assert(fabsf(state.vel - 0.0f) < EPSILON);
}
static void test_trap_reset(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    elib_traj_trapezoid_update(&ctx, 0.5f);
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_reset(&ctx);
    assert(err == ELIB_TRAJ_TRAPEZOID_OK);
    assert(fabsf(ctx.elapsed - 0.0f) < EPSILON);
    assert(ctx.status == ELIB_TRAJ_STATUS_RUNNING);
}
static void test_trap_deinit(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    elib_traj_trapezoid_deinit(&ctx);
    assert(ctx.initialized == 0);
    assert(elib_traj_trapezoid_update(&ctx, 0.01f) == ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED);
}
static void test_trap_generate_null_points(void) {
    elib_traj_trapezoid_params_t p = make_trap_params();
    assert(elib_traj_trapezoid_generate(&p, 0.0f, 0.01f, NULL, 100) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_generate_zero_count(void) {
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_state_t points[10];
    assert(elib_traj_trapezoid_generate(&p, 0.0f, 0.01f, points, 0) == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}
static void test_trap_generate_basic(void) {
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_state_t points[151];
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_generate(&p, 0.0f, 0.01f, points, 151);
    assert(err == ELIB_TRAJ_TRAPEZOID_OK);
    assert(fabsf(points[0].pos - 0.0f) < 1.0f);
    assert(fabsf(points[150].pos - 1000.0f) < 1.0f);
    assert(fabsf(points[150].vel - 0.0f) < 1.0f);
}

/* === S-curve profile tests === */
static elib_traj_scurve_params_t make_scurve_params(void) {
    elib_traj_scurve_params_t p = {
        .max_vel = 1000.0f, .max_acc = 5000.0f, .max_dec = 5000.0f,
        .max_jerk = 50000.0f, .target_pos = 1000.0f,
    };
    return p;
}

static void test_scurve_init_valid(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    assert(elib_traj_scurve_init(&ctx, &p, 0.0f) == ELIB_TRAJ_SCURVE_OK);
    assert(ctx.initialized == 1);
}
static void test_scurve_init_null_ctx(void) {
    elib_traj_scurve_params_t p = make_scurve_params();
    assert(elib_traj_scurve_init(NULL, &p, 0.0f) == ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM);
}
static void test_scurve_init_null_params(void) {
    elib_traj_scurve_ctx_t ctx;
    assert(elib_traj_scurve_init(&ctx, NULL, 0.0f) == ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM);
}
static void test_scurve_init_bad_jerk(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    p.max_jerk = 0.0f;
    assert(elib_traj_scurve_init(&ctx, &p, 0.0f) == ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM);
}
static void test_scurve_update_not_initialized(void) {
    elib_traj_scurve_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    assert(elib_traj_scurve_update(&ctx, 0.01f) == ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED);
}
static void test_scurve_update_bad_dt(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_scurve_init(&ctx, &p, 0.0f);
    assert(elib_traj_scurve_update(&ctx, 0.0f) == ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM);
}
static void test_scurve_zero_distance(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    p.target_pos = 50.0f;
    elib_traj_scurve_init(&ctx, &p, 50.0f);
    elib_traj_status_t status;
    elib_traj_scurve_get_status(&ctx, &status);
    assert(status == ELIB_TRAJ_STATUS_FINISHED);
}
static void test_scurve_reaches_target(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_scurve_init(&ctx, &p, 0.0f);
    elib_traj_scurve_update(&ctx, 5.0f);
    elib_traj_state_t state;
    elib_traj_scurve_get_state(&ctx, &state);
    assert(fabsf(state.pos - 1000.0f) < 1.0f);
    assert(fabsf(state.vel - 0.0f) < 1.0f);
    elib_traj_status_t status;
    elib_traj_scurve_get_status(&ctx, &status);
    assert(status == ELIB_TRAJ_STATUS_FINISHED);
}
static void test_scurve_smooth_acceleration(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_scurve_init(&ctx, &p, 0.0f);
    elib_traj_state_t state;
    elib_traj_scurve_get_state(&ctx, &state);
    assert(fabsf(state.acc - 0.0f) < EPSILON);
    assert(fabsf(state.vel - 0.0f) < EPSILON);
}
static void test_scurve_negative_direction(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    p.target_pos = -1000.0f;
    elib_traj_scurve_init(&ctx, &p, 0.0f);
    elib_traj_scurve_update(&ctx, 0.5f);
    elib_traj_state_t state;
    elib_traj_scurve_get_state(&ctx, &state);
    assert(state.vel < 0.0f);
}
static void test_scurve_reset(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_scurve_init(&ctx, &p, 0.0f);
    elib_traj_scurve_update(&ctx, 0.5f);
    assert(elib_traj_scurve_reset(&ctx) == ELIB_TRAJ_SCURVE_OK);
    assert(fabsf(ctx.elapsed - 0.0f) < EPSILON);
    assert(ctx.status == ELIB_TRAJ_STATUS_RUNNING);
}
static void test_scurve_deinit(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_scurve_init(&ctx, &p, 0.0f);
    elib_traj_scurve_deinit(&ctx);
    assert(ctx.initialized == 0);
    assert(elib_traj_scurve_update(&ctx, 0.01f) == ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED);
}
static void test_scurve_generate_basic(void) {
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_state_t points[501];
    elib_traj_scurve_err_t err = elib_traj_scurve_generate(&p, 0.0f, 0.01f, points, 501);
    assert(err == ELIB_TRAJ_SCURVE_OK);
    assert(fabsf(points[0].pos - 0.0f) < 1.0f);
    assert(fabsf(points[500].pos - 1000.0f) < 5.0f);
    assert(fabsf(points[500].vel - 0.0f) < 5.0f);
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
    printf("\n--- Trapezoidal profile ---\n");
    RUN_TEST(test_trap_init_valid);
    RUN_TEST(test_trap_init_null_ctx);
    RUN_TEST(test_trap_init_null_params);
    RUN_TEST(test_trap_init_bad_max_vel);
    RUN_TEST(test_trap_init_bad_acc);
    RUN_TEST(test_trap_init_bad_dec);
    RUN_TEST(test_trap_update_not_initialized);
    RUN_TEST(test_trap_update_bad_dt);
    RUN_TEST(test_trap_get_state_null);
    RUN_TEST(test_trap_get_status_null);
    RUN_TEST(test_trap_zero_distance);
    RUN_TEST(test_trap_normal_profile);
    RUN_TEST(test_trap_reaches_target);
    RUN_TEST(test_trap_negative_direction);
    RUN_TEST(test_trap_triangle_profile);
    RUN_TEST(test_trap_reset);
    RUN_TEST(test_trap_deinit);
    RUN_TEST(test_trap_generate_null_points);
    RUN_TEST(test_trap_generate_zero_count);
    RUN_TEST(test_trap_generate_basic);
    printf("\n--- S-curve profile ---\n");
    RUN_TEST(test_scurve_init_valid);
    RUN_TEST(test_scurve_init_null_ctx);
    RUN_TEST(test_scurve_init_null_params);
    RUN_TEST(test_scurve_init_bad_jerk);
    RUN_TEST(test_scurve_update_not_initialized);
    RUN_TEST(test_scurve_update_bad_dt);
    RUN_TEST(test_scurve_zero_distance);
    RUN_TEST(test_scurve_reaches_target);
    RUN_TEST(test_scurve_smooth_acceleration);
    RUN_TEST(test_scurve_negative_direction);
    RUN_TEST(test_scurve_reset);
    RUN_TEST(test_scurve_deinit);
    RUN_TEST(test_scurve_generate_basic);
    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
