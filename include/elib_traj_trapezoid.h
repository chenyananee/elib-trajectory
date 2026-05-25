/* elib_traj_trapezoid.h - Trapezoidal Velocity Profile */
#ifndef ELIB_TRAJ_TRAPEZOID_H
#define ELIB_TRAJ_TRAPEZOID_H

#include "elib_traj_defs.h"
#include "elib_traj_trapezoid_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    elib_traj_val_t max_vel;
    elib_traj_val_t acc;
    elib_traj_val_t dec;
    elib_traj_val_t target_pos;
} elib_traj_trapezoid_params_t;

typedef struct {
    elib_traj_trapezoid_params_t params;
    elib_traj_val_t start_pos;
    elib_traj_val_t direction;
    elib_traj_val_t distance;
    elib_traj_val_t t_acc;
    elib_traj_val_t t_const;
    elib_traj_val_t t_dec;
    elib_traj_val_t t_total;
    elib_traj_val_t v_peak;
    elib_traj_val_t d_acc;
    elib_traj_val_t d_const;
    elib_traj_val_t elapsed;
    elib_traj_state_t state;
    elib_traj_status_t status;
    struct {
        uint8_t initialized : 1;
    } bit_flags;
} elib_traj_trapezoid_ctx_t;

elib_traj_trapezoid_err_t elib_traj_trapezoid_init(
    elib_traj_trapezoid_ctx_t *ctx,
    const elib_traj_trapezoid_params_t *params,
    elib_traj_val_t start_pos);

void elib_traj_trapezoid_deinit(elib_traj_trapezoid_ctx_t *ctx);

elib_traj_trapezoid_err_t elib_traj_trapezoid_reset(
    elib_traj_trapezoid_ctx_t *ctx);

elib_traj_trapezoid_err_t elib_traj_trapezoid_update(
    elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_val_t dt);

elib_traj_trapezoid_err_t elib_traj_trapezoid_get_state(
    const elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_state_t *state);

elib_traj_trapezoid_err_t elib_traj_trapezoid_get_status(
    const elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_status_t *status);

elib_traj_trapezoid_err_t elib_traj_trapezoid_generate(
    const elib_traj_trapezoid_params_t *params,
    elib_traj_val_t start_pos,
    elib_traj_val_t dt,
    elib_traj_state_t *points,
    uint32_t num_points);

#ifdef __cplusplus
}
#endif

#endif /* ELIB_TRAJ_TRAPEZOID_H */
