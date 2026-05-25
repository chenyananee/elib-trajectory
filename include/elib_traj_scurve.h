/* elib_traj_scurve.h - S-Curve Velocity Profile (7-segment) */
#ifndef ELIB_TRAJ_SCURVE_H
#define ELIB_TRAJ_SCURVE_H

#include "elib_traj_defs.h"
#include "elib_traj_scurve_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    elib_traj_val_t max_vel;
    elib_traj_val_t max_acc;
    elib_traj_val_t max_dec;
    elib_traj_val_t max_jerk;
    elib_traj_val_t target_pos;
} elib_traj_scurve_params_t;

#define ELIB_TRAJ_SCURVE_PHASES 7

typedef struct {
    elib_traj_scurve_params_t params;
    elib_traj_val_t start_pos;
    elib_traj_val_t direction;
    elib_traj_val_t distance;
    elib_traj_val_t t[ELIB_TRAJ_SCURVE_PHASES + 1];
    elib_traj_val_t v[ELIB_TRAJ_SCURVE_PHASES + 1];
    elib_traj_val_t a[ELIB_TRAJ_SCURVE_PHASES + 1];
    elib_traj_val_t p[ELIB_TRAJ_SCURVE_PHASES + 1];
    elib_traj_val_t jerk[ELIB_TRAJ_SCURVE_PHASES];
    elib_traj_val_t elapsed;
    elib_traj_state_t state;
    elib_traj_status_t status;
    struct {
        uint8_t initialized : 1;
    } bit_flags;
} elib_traj_scurve_ctx_t;

elib_traj_scurve_err_t elib_traj_scurve_init(
    elib_traj_scurve_ctx_t *ctx,
    const elib_traj_scurve_params_t *params,
    elib_traj_val_t start_pos);

void elib_traj_scurve_deinit(elib_traj_scurve_ctx_t *ctx);

elib_traj_scurve_err_t elib_traj_scurve_reset(
    elib_traj_scurve_ctx_t *ctx);

elib_traj_scurve_err_t elib_traj_scurve_update(
    elib_traj_scurve_ctx_t *ctx,
    elib_traj_val_t dt);

elib_traj_scurve_err_t elib_traj_scurve_get_state(
    const elib_traj_scurve_ctx_t *ctx,
    elib_traj_state_t *state);

elib_traj_scurve_err_t elib_traj_scurve_get_status(
    const elib_traj_scurve_ctx_t *ctx,
    elib_traj_status_t *status);

elib_traj_scurve_err_t elib_traj_scurve_generate(
    const elib_traj_scurve_params_t *params,
    elib_traj_val_t start_pos,
    elib_traj_val_t dt,
    elib_traj_state_t *points,
    uint32_t num_points);

#ifdef __cplusplus
}
#endif

#endif /* ELIB_TRAJ_SCURVE_H */
