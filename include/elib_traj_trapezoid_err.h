/* elib_traj_trapezoid_err.h - Trapezoidal Trajectory Error Codes */
#ifndef ELIB_TRAJ_TRAPEZOID_ERR_H
#define ELIB_TRAJ_TRAPEZOID_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ELIB_TRAJ_TRAPEZOID_OK = 0,
    ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM,
    ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED,
} elib_traj_trapezoid_err_t;

#ifdef __cplusplus
}
#endif

#endif /* ELIB_TRAJ_TRAPEZOID_ERR_H */
