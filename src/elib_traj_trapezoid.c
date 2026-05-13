/* elib_traj_trapezoid.c - Trapezoidal Velocity Profile Implementation */
#include "../include/elib_traj_trapezoid.h"
#include "elib_traj_util.h"
#include <math.h>
#include <string.h>

static void elib_traj_trapezoid_compute(const elib_traj_trapezoid_ctx_t *ctx,
                                         elib_traj_val_t t,
                                         elib_traj_state_t *state) {
    elib_traj_val_t dir = ctx->direction;

    if (t >= ctx->t_total) {
        state->pos = ctx->params.target_pos;
        state->vel = (elib_traj_val_t)0;
        state->acc = (elib_traj_val_t)0;
        return;
    }

    if (t < ctx->t_acc) {
        state->acc = ctx->params.acc * dir;
        state->vel = ctx->params.acc * t * dir;
        state->pos = ctx->start_pos +
                     (elib_traj_val_t)0.5 * ctx->params.acc * t * t * dir;
    } else if (t < ctx->t_acc + ctx->t_const) {
        elib_traj_val_t dt_const = t - ctx->t_acc;
        state->acc = (elib_traj_val_t)0;
        state->vel = ctx->v_peak * dir;
        state->pos = ctx->start_pos +
                     (ctx->d_acc + ctx->v_peak * dt_const) * dir;
    } else {
        elib_traj_val_t dt_dec = t - ctx->t_acc - ctx->t_const;
        state->acc = -ctx->params.dec * dir;
        state->vel = (ctx->v_peak - ctx->params.dec * dt_dec) * dir;
        state->pos = ctx->start_pos +
                     (ctx->d_acc + ctx->d_const +
                      ctx->v_peak * dt_dec -
                      (elib_traj_val_t)0.5 * ctx->params.dec * dt_dec * dt_dec) * dir;
    }
}

elib_traj_trapezoid_err_t elib_traj_trapezoid_init(
    elib_traj_trapezoid_ctx_t *ctx,
    const elib_traj_trapezoid_params_t *params,
    elib_traj_val_t start_pos) {

    if (ctx == NULL || params == NULL) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (params->max_vel <= (elib_traj_val_t)0) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (params->acc <= (elib_traj_val_t)0) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (params->dec <= (elib_traj_val_t)0) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;

    memset(ctx, 0, sizeof(elib_traj_trapezoid_ctx_t));
    memcpy(&ctx->params, params, sizeof(elib_traj_trapezoid_params_t));
    ctx->start_pos = start_pos;

    elib_traj_val_t distance = params->target_pos - start_pos;
    elib_traj_val_t abs_distance = elib_traj_util_fabs(distance);

    if (distance > (elib_traj_val_t)0) {
        ctx->direction = (elib_traj_val_t)1;
    } else if (distance < (elib_traj_val_t)0) {
        ctx->direction = (elib_traj_val_t)-1;
    } else {
        ctx->direction = (elib_traj_val_t)1;
        ctx->distance = (elib_traj_val_t)0;
        ctx->t_acc = 0; ctx->t_const = 0; ctx->t_dec = 0; ctx->t_total = 0;
        ctx->v_peak = 0; ctx->d_acc = 0; ctx->d_const = 0;
        ctx->elapsed = 0;
        ctx->state.pos = start_pos; ctx->state.vel = 0; ctx->state.acc = 0;
        ctx->status = ELIB_TRAJ_STATUS_FINISHED;
        ctx->initialized = 1;
        return ELIB_TRAJ_TRAPEZOID_OK;
    }

    ctx->distance = abs_distance;

    elib_traj_val_t t_to_max = params->max_vel / params->acc;
    elib_traj_val_t d_acc = (elib_traj_val_t)0.5 * params->acc * t_to_max * t_to_max;
    elib_traj_val_t t_to_stop = params->max_vel / params->dec;
    elib_traj_val_t d_dec = (elib_traj_val_t)0.5 * params->dec * t_to_stop * t_to_stop;

    if (abs_distance >= d_acc + d_dec) {
        ctx->t_acc = t_to_max;
        ctx->t_dec = t_to_stop;
        ctx->v_peak = params->max_vel;
        ctx->d_acc = d_acc;
        ctx->d_const = abs_distance - d_acc - d_dec;
        ctx->t_const = ctx->d_const / params->max_vel;
    } else {
        elib_traj_val_t v_peak_sq =
            (elib_traj_val_t)2 * abs_distance * params->acc * params->dec /
            (params->acc + params->dec);
        ctx->v_peak = sqrtf(v_peak_sq);
        ctx->t_acc = ctx->v_peak / params->acc;
        ctx->t_dec = ctx->v_peak / params->dec;
        ctx->d_acc = (elib_traj_val_t)0.5 * params->acc * ctx->t_acc * ctx->t_acc;
        ctx->d_const = (elib_traj_val_t)0;
        ctx->t_const = (elib_traj_val_t)0;
    }

    ctx->t_total = ctx->t_acc + ctx->t_const + ctx->t_dec;
    ctx->elapsed = (elib_traj_val_t)0;
    ctx->state.pos = start_pos;
    ctx->state.vel = (elib_traj_val_t)0;
    ctx->state.acc = (elib_traj_val_t)0;
    ctx->status = ELIB_TRAJ_STATUS_RUNNING;
    ctx->initialized = 1;
    return ELIB_TRAJ_TRAPEZOID_OK;
}

void elib_traj_trapezoid_deinit(elib_traj_trapezoid_ctx_t *ctx) {
    if (ctx == NULL) return;
    ctx->initialized = 0;
}

elib_traj_trapezoid_err_t elib_traj_trapezoid_reset(elib_traj_trapezoid_ctx_t *ctx) {
    if (ctx == NULL) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED;
    ctx->elapsed = (elib_traj_val_t)0;
    ctx->state.pos = ctx->start_pos;
    ctx->state.vel = (elib_traj_val_t)0;
    ctx->state.acc = (elib_traj_val_t)0;
    ctx->status = ELIB_TRAJ_STATUS_RUNNING;
    return ELIB_TRAJ_TRAPEZOID_OK;
}

elib_traj_trapezoid_err_t elib_traj_trapezoid_update(
    elib_traj_trapezoid_ctx_t *ctx, elib_traj_val_t dt) {
    if (ctx == NULL) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED;
    if (dt <= (elib_traj_val_t)0) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (ctx->status == ELIB_TRAJ_STATUS_FINISHED) return ELIB_TRAJ_TRAPEZOID_OK;
    ctx->elapsed += dt;
    if (ctx->elapsed >= ctx->t_total) {
        ctx->elapsed = ctx->t_total;
        ctx->status = ELIB_TRAJ_STATUS_FINISHED;
    } else {
        ctx->status = ELIB_TRAJ_STATUS_RUNNING;
    }
    elib_traj_trapezoid_compute(ctx, ctx->elapsed, &ctx->state);
    return ELIB_TRAJ_TRAPEZOID_OK;
}

elib_traj_trapezoid_err_t elib_traj_trapezoid_get_state(
    const elib_traj_trapezoid_ctx_t *ctx, elib_traj_state_t *state) {
    if (ctx == NULL || state == NULL) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED;
    *state = ctx->state;
    return ELIB_TRAJ_TRAPEZOID_OK;
}

elib_traj_trapezoid_err_t elib_traj_trapezoid_get_status(
    const elib_traj_trapezoid_ctx_t *ctx, elib_traj_status_t *status) {
    if (ctx == NULL || status == NULL) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED;
    *status = ctx->status;
    return ELIB_TRAJ_TRAPEZOID_OK;
}

elib_traj_trapezoid_err_t elib_traj_trapezoid_generate(
    const elib_traj_trapezoid_params_t *params, elib_traj_val_t start_pos,
    elib_traj_val_t dt, elib_traj_state_t *points, uint32_t num_points) {
    if (params == NULL || points == NULL) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (num_points == 0) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (dt <= (elib_traj_val_t)0) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_init(&ctx, params, start_pos);
    if (err != ELIB_TRAJ_TRAPEZOID_OK) return err;
    for (uint32_t i = 0; i < num_points; i++) {
        elib_traj_val_t t = (elib_traj_val_t)i * dt;
        elib_traj_trapezoid_compute(&ctx, t, &points[i]);
    }
    return ELIB_TRAJ_TRAPEZOID_OK;
}
