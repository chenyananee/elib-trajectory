/* elib_traj_defs.h - Trajectory Planner Common Definitions */
#ifndef ELIB_TRAJ_DEFS_H
#define ELIB_TRAJ_DEFS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* User-overridable value type (default: float) */
#ifndef ELIB_TRAJ_VAL_T
#define ELIB_TRAJ_VAL_T float
#endif

typedef ELIB_TRAJ_VAL_T elib_traj_val_t;

/* Trajectory output state: position, velocity, acceleration */
typedef struct {
    elib_traj_val_t pos;
    elib_traj_val_t vel;
    elib_traj_val_t acc;
} elib_traj_state_t;

/* Planner status */
typedef enum {
    ELIB_TRAJ_STATUS_IDLE = 0,
    ELIB_TRAJ_STATUS_RUNNING,
    ELIB_TRAJ_STATUS_FINISHED,
} elib_traj_status_t;

#ifdef __cplusplus
}
#endif

#endif /* ELIB_TRAJ_DEFS_H */
