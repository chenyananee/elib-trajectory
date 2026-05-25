/* elib_traj_scurve.c - S-Curve Velocity Profile (7-segment) Implementation */
#include "../include/elib_traj_scurve.h"
#include "elib_traj_util.h"
#include <math.h>
#include <string.h>

static void elib_traj_scurve_compute(const elib_traj_scurve_ctx_t *ctx,
                                      elib_traj_val_t t,
                                      elib_traj_state_t *state) {
    elib_traj_val_t dir = ctx->direction;
    int num_phases = ELIB_TRAJ_SCURVE_PHASES;

    int phase = -1;
    for (int i = 0; i < num_phases; i++) {
        if (t < ctx->t[i + 1]) { phase = i; break; }
    }

    if (phase < 0) {
        state->pos = ctx->params.target_pos;
        state->vel = (elib_traj_val_t)0;
        state->acc = (elib_traj_val_t)0;
        return;
    }

    elib_traj_val_t tau = t - ctx->t[phase];
    elib_traj_val_t J = ctx->jerk[phase];

    elib_traj_val_t a_norm = ctx->a[phase] + J * tau;
    elib_traj_val_t v_norm = ctx->v[phase] + ctx->a[phase] * tau +
                              (elib_traj_val_t)0.5 * J * tau * tau;
    elib_traj_val_t p_norm = ctx->p[phase] + ctx->v[phase] * tau +
                              (elib_traj_val_t)0.5 * ctx->a[phase] * tau * tau +
                              (elib_traj_val_t)(1.0 / 6.0) * J * tau * tau * tau;

    state->acc = a_norm * dir;
    state->vel = v_norm * dir;
    state->pos = ctx->start_pos + p_norm * dir;
}

static int elib_traj_scurve_precompute_full(elib_traj_scurve_ctx_t *ctx) {
    elib_traj_val_t max_vel = ctx->params.max_vel;
    elib_traj_val_t max_acc = ctx->params.max_acc;
    elib_traj_val_t max_dec = ctx->params.max_dec;
    elib_traj_val_t max_jerk = ctx->params.max_jerk;

    elib_traj_val_t T1 = max_acc / max_jerk;
    elib_traj_val_t T3 = max_acc / max_jerk;
    elib_traj_val_t v_from_jerk = max_acc * T1;
    if (max_vel < v_from_jerk) return -1;
    elib_traj_val_t T2 = (max_vel - v_from_jerk) / max_acc;

    elib_traj_val_t T5 = max_dec / max_jerk;
    elib_traj_val_t T7 = max_dec / max_jerk;
    elib_traj_val_t v_from_dec_jerk = max_dec * T5;
    if (max_vel < v_from_dec_jerk) return -1;
    elib_traj_val_t T6 = (max_vel - v_from_dec_jerk) / max_dec;

    /* Compute accel distance (phases 1+2+3) by simulation */
    elib_traj_val_t a_sim = 0, v_sim = 0, p_sim = 0;
    /* Phase 1 */
    elib_traj_val_t J1 = max_jerk;
    p_sim += v_sim * T1 + (elib_traj_val_t)0.5 * a_sim * T1 * T1 + (elib_traj_val_t)(1.0/6.0) * J1 * T1 * T1 * T1;
    v_sim += a_sim * T1 + (elib_traj_val_t)0.5 * J1 * T1 * T1;
    a_sim += J1 * T1;
    /* Phase 2 */
    p_sim += v_sim * T2 + (elib_traj_val_t)0.5 * a_sim * T2 * T2;
    v_sim += a_sim * T2;
    /* Phase 3 */
    elib_traj_val_t J3 = -max_jerk;
    p_sim += v_sim * T3 + (elib_traj_val_t)0.5 * a_sim * T3 * T3 + (elib_traj_val_t)(1.0/6.0) * J3 * T3 * T3 * T3;
    v_sim += a_sim * T3 + (elib_traj_val_t)0.5 * J3 * T3 * T3;
    a_sim += J3 * T3;
    elib_traj_val_t d_acc = p_sim;

    /* Compute decel distance (phases 5+6+7) by simulation */
    a_sim = 0; v_sim = max_vel; p_sim = 0;
    /* Phase 5 */
    elib_traj_val_t J5 = -max_jerk;
    p_sim += v_sim * T5 + (elib_traj_val_t)0.5 * a_sim * T5 * T5 + (elib_traj_val_t)(1.0/6.0) * J5 * T5 * T5 * T5;
    v_sim += a_sim * T5 + (elib_traj_val_t)0.5 * J5 * T5 * T5;
    a_sim += J5 * T5;
    /* Phase 6 */
    p_sim += v_sim * T6 + (elib_traj_val_t)0.5 * a_sim * T6 * T6;
    v_sim += a_sim * T6;
    /* Phase 7 */
    elib_traj_val_t J7 = max_jerk;
    p_sim += v_sim * T7 + (elib_traj_val_t)0.5 * a_sim * T7 * T7 + (elib_traj_val_t)(1.0/6.0) * J7 * T7 * T7 * T7;
    v_sim += a_sim * T7 + (elib_traj_val_t)0.5 * J7 * T7 * T7;
    a_sim += J7 * T7;
    elib_traj_val_t d_dec = p_sim;

    elib_traj_val_t abs_dist = ctx->distance;
    if (abs_dist < d_acc + d_dec) return -1;

    elib_traj_val_t T4 = (abs_dist - d_acc - d_dec) / max_vel;

    ctx->t[0] = 0;
    ctx->t[1] = T1;
    ctx->t[2] = T1 + T2;
    ctx->t[3] = T1 + T2 + T3;
    ctx->t[4] = T1 + T2 + T3 + T4;
    ctx->t[5] = T1 + T2 + T3 + T4 + T5;
    ctx->t[6] = T1 + T2 + T3 + T4 + T5 + T6;
    ctx->t[7] = T1 + T2 + T3 + T4 + T5 + T6 + T7;

    ctx->jerk[0] = max_jerk;
    ctx->jerk[1] = 0;
    ctx->jerk[2] = -max_jerk;
    ctx->jerk[3] = 0;
    ctx->jerk[4] = -max_jerk;
    ctx->jerk[5] = 0;
    ctx->jerk[6] = max_jerk;

    ctx->a[0] = 0; ctx->v[0] = 0; ctx->p[0] = 0;
    for (int i = 0; i < ELIB_TRAJ_SCURVE_PHASES; i++) {
        elib_traj_val_t Ti = ctx->t[i + 1] - ctx->t[i];
        elib_traj_val_t J = ctx->jerk[i];
        ctx->a[i + 1] = ctx->a[i] + J * Ti;
        ctx->v[i + 1] = ctx->v[i] + ctx->a[i] * Ti + (elib_traj_val_t)0.5 * J * Ti * Ti;
        ctx->p[i + 1] = ctx->p[i] + ctx->v[i] * Ti + (elib_traj_val_t)0.5 * ctx->a[i] * Ti * Ti +
                          (elib_traj_val_t)(1.0/6.0) * J * Ti * Ti * Ti;
    }
    return 0;
}

static void elib_traj_scurve_precompute_degenerate(elib_traj_scurve_ctx_t *ctx) {
    elib_traj_val_t abs_dist = ctx->distance;
    elib_traj_val_t max_acc = ctx->params.max_acc;
    elib_traj_val_t max_jerk = ctx->params.max_jerk;

    /* Simplified jerk triangle: no const-acc, no const-vel */
    elib_traj_val_t T_jerk = max_acc / max_jerk;

    ctx->t[0] = 0;
    ctx->t[1] = T_jerk;
    ctx->t[2] = T_jerk;
    ctx->t[3] = (elib_traj_val_t)2 * T_jerk;
    ctx->t[4] = (elib_traj_val_t)2 * T_jerk;
    ctx->t[5] = (elib_traj_val_t)3 * T_jerk;
    ctx->t[6] = (elib_traj_val_t)3 * T_jerk;
    ctx->t[7] = (elib_traj_val_t)4 * T_jerk;

    ctx->jerk[0] = max_jerk;
    ctx->jerk[1] = 0;
    ctx->jerk[2] = -max_jerk;
    ctx->jerk[3] = 0;
    ctx->jerk[4] = -max_jerk;
    ctx->jerk[5] = 0;
    ctx->jerk[6] = max_jerk;

    /* Compute boundary values by simulation */
    ctx->a[0] = 0; ctx->v[0] = 0; ctx->p[0] = 0;
    for (int i = 0; i < ELIB_TRAJ_SCURVE_PHASES; i++) {
        elib_traj_val_t Ti = ctx->t[i + 1] - ctx->t[i];
        elib_traj_val_t J = ctx->jerk[i];
        ctx->a[i + 1] = ctx->a[i] + J * Ti;
        ctx->v[i + 1] = ctx->v[i] + ctx->a[i] * Ti + (elib_traj_val_t)0.5 * J * Ti * Ti;
        ctx->p[i + 1] = ctx->p[i] + ctx->v[i] * Ti + (elib_traj_val_t)0.5 * ctx->a[i] * Ti * Ti +
                          (elib_traj_val_t)(1.0/6.0) * J * Ti * Ti * Ti;
    }

    /* Scale to match actual distance */
    elib_traj_val_t computed_dist = ctx->p[ELIB_TRAJ_SCURVE_PHASES];
    if (computed_dist > (elib_traj_val_t)0) {
        elib_traj_val_t scale = abs_dist / computed_dist;
        elib_traj_val_t t_scale = sqrtf(scale);
        for (int i = 0; i <= ELIB_TRAJ_SCURVE_PHASES; i++) {
            ctx->p[i] *= scale;
            ctx->v[i] *= t_scale;
            ctx->t[i] *= t_scale;
        }
    }
}

elib_traj_scurve_err_t elib_traj_scurve_init(
    elib_traj_scurve_ctx_t *ctx,
    const elib_traj_scurve_params_t *params,
    elib_traj_val_t start_pos) {

    if (ctx == NULL || params == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (params->max_vel <= (elib_traj_val_t)0) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (params->max_acc <= (elib_traj_val_t)0) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (params->max_dec <= (elib_traj_val_t)0) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (params->max_jerk <= (elib_traj_val_t)0) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;

    memset(ctx, 0, sizeof(elib_traj_scurve_ctx_t));
    memcpy(&ctx->params, params, sizeof(elib_traj_scurve_params_t));
    ctx->start_pos = start_pos;

    elib_traj_val_t distance = params->target_pos - start_pos;
    elib_traj_val_t abs_distance = elib_traj_util_fabs(distance);

    if (distance > (elib_traj_val_t)0) {
        ctx->direction = (elib_traj_val_t)1;
    } else if (distance < (elib_traj_val_t)0) {
        ctx->direction = (elib_traj_val_t)-1;
    } else {
        ctx->direction = (elib_traj_val_t)1;
        ctx->distance = 0;
        ctx->state.pos = start_pos; ctx->state.vel = 0; ctx->state.acc = 0;
        ctx->status = ELIB_TRAJ_STATUS_FINISHED;
        ctx->bit_flags.initialized = 1;
        return ELIB_TRAJ_SCURVE_OK;
    }

    ctx->distance = abs_distance;

    if (elib_traj_scurve_precompute_full(ctx) != 0) {
        elib_traj_scurve_precompute_degenerate(ctx);
    }

    ctx->elapsed = 0;
    ctx->state.pos = start_pos; ctx->state.vel = 0; ctx->state.acc = 0;
    ctx->status = ELIB_TRAJ_STATUS_RUNNING;
    ctx->bit_flags.initialized = 1;
    return ELIB_TRAJ_SCURVE_OK;
}

void elib_traj_scurve_deinit(elib_traj_scurve_ctx_t *ctx) {
    if (ctx == NULL) return;
    ctx->bit_flags.initialized = 0;
}

elib_traj_scurve_err_t elib_traj_scurve_reset(elib_traj_scurve_ctx_t *ctx) {
    if (ctx == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (!ctx->bit_flags.initialized) return ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED;
    ctx->elapsed = 0;
    ctx->state.pos = ctx->start_pos; ctx->state.vel = 0; ctx->state.acc = 0;
    ctx->status = ELIB_TRAJ_STATUS_RUNNING;
    return ELIB_TRAJ_SCURVE_OK;
}

elib_traj_scurve_err_t elib_traj_scurve_update(
    elib_traj_scurve_ctx_t *ctx, elib_traj_val_t dt) {
    if (ctx == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (!ctx->bit_flags.initialized) return ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED;
    if (dt <= (elib_traj_val_t)0) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (ctx->status == ELIB_TRAJ_STATUS_FINISHED) return ELIB_TRAJ_SCURVE_OK;

    ctx->elapsed += dt;
    elib_traj_val_t t_total = ctx->t[ELIB_TRAJ_SCURVE_PHASES];
    if (ctx->elapsed >= t_total) {
        ctx->elapsed = t_total;
        ctx->status = ELIB_TRAJ_STATUS_FINISHED;
    } else {
        ctx->status = ELIB_TRAJ_STATUS_RUNNING;
    }
    elib_traj_scurve_compute(ctx, ctx->elapsed, &ctx->state);
    return ELIB_TRAJ_SCURVE_OK;
}

elib_traj_scurve_err_t elib_traj_scurve_get_state(
    const elib_traj_scurve_ctx_t *ctx, elib_traj_state_t *state) {
    if (ctx == NULL || state == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED;
    *state = ctx->state;
    return ELIB_TRAJ_SCURVE_OK;
}

elib_traj_scurve_err_t elib_traj_scurve_get_status(
    const elib_traj_scurve_ctx_t *ctx, elib_traj_status_t *status) {
    if (ctx == NULL || status == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED;
    *status = ctx->status;
    return ELIB_TRAJ_SCURVE_OK;
}

elib_traj_scurve_err_t elib_traj_scurve_generate(
    const elib_traj_scurve_params_t *params, elib_traj_val_t start_pos,
    elib_traj_val_t dt, elib_traj_state_t *points, uint32_t num_points) {
    if (params == NULL || points == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (num_points == 0) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (dt <= (elib_traj_val_t)0) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;

    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_err_t err = elib_traj_scurve_init(&ctx, params, start_pos);
    if (err != ELIB_TRAJ_SCURVE_OK) return err;

    for (uint32_t i = 0; i < num_points; i++) {
        elib_traj_val_t t = (elib_traj_val_t)i * dt;
        elib_traj_scurve_compute(&ctx, t, &points[i]);
    }
    return ELIB_TRAJ_SCURVE_OK;
}
