/* elib_traj_util.h - Trajectory Planner Internal Utilities */
#ifndef ELIB_TRAJ_UTIL_H
#define ELIB_TRAJ_UTIL_H

#include "../include/elib_traj_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline elib_traj_val_t elib_traj_util_clamp(elib_traj_val_t val,
                                                     elib_traj_val_t min_val,
                                                     elib_traj_val_t max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static inline elib_traj_val_t elib_traj_util_fabs(elib_traj_val_t val) {
    if (val < (elib_traj_val_t)0) return -val;
    return val;
}

#ifdef __cplusplus
}
#endif

#endif /* ELIB_TRAJ_UTIL_H */
