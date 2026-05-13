/* elib_traj_scurve_err.h - S-Curve Trajectory Error Codes */
#ifndef ELIB_TRAJ_SCURVE_ERR_H
#define ELIB_TRAJ_SCURVE_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ELIB_TRAJ_SCURVE_OK = 0,
    ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM,
    ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED,
} elib_traj_scurve_err_t;

#ifdef __cplusplus
}
#endif

#endif /* ELIB_TRAJ_SCURVE_ERR_H */
