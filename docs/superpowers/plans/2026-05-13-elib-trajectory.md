# elib-trajectory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an embedded trajectory planning library with trapezoidal and S-curve velocity profiles, following elib family conventions.

**Architecture:** Two decoupled algorithms (trapezoid/scurve) share common types and utility functions. Each algorithm has a core `compute(ctx, t, &state)` function; online mode wraps it with dt accumulation, offline mode wraps it with loop + array fill. Value type is user-definable via `ELIB_TRAJ_VAL_T` macro (default `float`).

**Tech Stack:** C99, no external dependencies, no build system (drop-in source library), requires `<math.h>` for `sqrt`.

---

### Task 1: Project scaffold

**Files:**
- Create: `LICENSE`
- Create: `.gitattributes`
- Create: `scripts/setup-push-remote.sh`
- Create: `scripts/setup-push-remote.bat`
- Create: `include/` directory
- Create: `src/` directory
- Create: `test/` directory

- [ ] **Step 1: Create directory structure**

```bash
cd D:/Prj/Open/github/elib-trajectory
mkdir -p include src test scripts
```

- [ ] **Step 2: Create LICENSE**

Write `LICENSE`:

```
MIT License

Copyright (c) 2026 ChenYanan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

- [ ] **Step 3: Create .gitattributes**

Write `.gitattributes`:

```
# Ensure shell scripts use LF line endings on all platforms
*.sh text eol=lf
```

- [ ] **Step 4: Create scripts/setup-push-remote.sh**

Write `scripts/setup-push-remote.sh`:

```bash
#!/bin/bash
# Setup dual push remotes (GitHub + Gitee) for current repository

REPO_NAME=$(basename "$(git rev-parse --show-toplevel)")
GITHUB_URL="https://github.com/chenyananee/${REPO_NAME}.git"
GITEE_URL="https://gitee.com/chenyananee/${REPO_NAME}.git"

git remote set-url --add --push origin "$GITEE_URL"
git remote set-url --add --push origin "$GITHUB_URL"

echo "Dual push remotes configured:"
git remote -v | grep push
```

- [ ] **Step 5: Create scripts/setup-push-remote.bat**

Write `scripts/setup-push-remote.bat`:

```bat
@echo off
REM Setup dual push remotes (GitHub + Gitee) for current repository

for /f %%I in ('git rev-parse --show-toplevel') do set REPO_NAME=%%~nxI
set GITHUB_URL=https://github.com/chenyananee/%REPO_NAME%.git
set GITEE_URL=https://gitee.com/chenyananee/%REPO_NAME%.git

git remote set-url --add --push origin %GITEE_URL%
git remote set-url --add --push origin %GITHUB_URL%

echo Dual push remotes configured:
git remote -v | findstr push
```

- [ ] **Step 6: Commit**

```bash
git add LICENSE .gitattributes scripts/
git commit -m "chore: initialize project scaffold"
```

---

### Task 2: Common definitions header

**Files:**
- Create: `include/elib_traj_defs.h`

- [ ] **Step 1: Create elib_traj_defs.h**

Write `include/elib_traj_defs.h`:

```c
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
```

- [ ] **Step 2: Commit**

```bash
git add include/elib_traj_defs.h
git commit -m "feat: add common trajectory definitions and types"
```

---

### Task 3: Utility functions with tests

**Files:**
- Create: `src/elib_traj_util.h`
- Create: `src/elib_traj_util.c`
- Create: `test/test_elib_traj.c`

- [ ] **Step 1: Write failing tests for utility functions**

Write `test/test_elib_traj.c`:

```c
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `gcc -o test_elib_traj test/test_elib_traj.c -Iinclude -lm && ./test_elib_traj`
Expected: Compilation error — `elib_traj_util.h` not found.

- [ ] **Step 3: Implement utility functions**

Write `src/elib_traj_util.h`:

```c
/* elib_traj_util.h - Trajectory Planner Internal Utilities */
#ifndef ELIB_TRAJ_UTIL_H
#define ELIB_TRAJ_UTIL_H

#include "../include/elib_traj_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Clamp value to [min_val, max_val]
 */
static inline elib_traj_val_t elib_traj_util_clamp(elib_traj_val_t val,
                                                     elib_traj_val_t min_val,
                                                     elib_traj_val_t max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief Absolute value
 */
static inline elib_traj_val_t elib_traj_util_fabs(elib_traj_val_t val) {
    if (val < (elib_traj_val_t)0) return -val;
    return val;
}

#ifdef __cplusplus
}
#endif

#endif /* ELIB_TRAJ_UTIL_H */
```

Write `src/elib_traj_util.c`:

```c
/* elib_traj_util.c - Trajectory Planner Internal Utilities */
#include "elib_traj_util.h"
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `gcc -o test_elib_traj test/test_elib_traj.c -Iinclude -lm && ./test_elib_traj`
Expected: All 6 tests PASSED.

- [ ] **Step 5: Commit**

```bash
git add src/elib_traj_util.h src/elib_traj_util.c test/test_elib_traj.c
git commit -m "feat: add utility functions (clamp, fabs) with tests"
```

---

### Task 4: Trapezoid headers

**Files:**
- Create: `include/elib_traj_trapezoid_err.h`
- Create: `include/elib_traj_trapezoid.h`

- [ ] **Step 1: Create elib_traj_trapezoid_err.h**

Write `include/elib_traj_trapezoid_err.h`:

```c
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
```

- [ ] **Step 2: Create elib_traj_trapezoid.h**

Write `include/elib_traj_trapezoid.h`:

```c
/* elib_traj_trapezoid.h - Trapezoidal Velocity Profile */
#ifndef ELIB_TRAJ_TRAPEZOID_H
#define ELIB_TRAJ_TRAPEZOID_H

#include "elib_traj_defs.h"
#include "elib_traj_trapezoid_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Trapezoidal profile parameters */
typedef struct {
    elib_traj_val_t max_vel;    /* Maximum velocity, must be > 0 */
    elib_traj_val_t acc;        /* Acceleration, must be > 0 */
    elib_traj_val_t dec;        /* Deceleration, must be > 0 */
    elib_traj_val_t target_pos; /* Target position */
} elib_traj_trapezoid_params_t;

/* Trapezoidal profile context (statically allocated by user) */
typedef struct {
    elib_traj_trapezoid_params_t params;
    elib_traj_val_t start_pos;      /* Start position */
    elib_traj_val_t direction;      /* +1 or -1 */
    elib_traj_val_t distance;       /* |target_pos - start_pos| */
    elib_traj_val_t t_acc;          /* Acceleration phase duration */
    elib_traj_val_t t_const;        /* Constant velocity phase duration */
    elib_traj_val_t t_dec;          /* Deceleration phase duration */
    elib_traj_val_t t_total;        /* Total duration */
    elib_traj_val_t v_peak;         /* Actual peak velocity (may be < max_vel) */
    elib_traj_val_t d_acc;          /* Distance covered during acceleration */
    elib_traj_val_t d_const;        /* Distance covered during constant velocity */
    elib_traj_val_t elapsed;        /* Elapsed time */
    elib_traj_state_t state;        /* Current output state */
    elib_traj_status_t status;      /* Planner status */
    int initialized;
} elib_traj_trapezoid_ctx_t;

/**
 * @brief Initialize trapezoidal velocity profile
 * @param ctx User-allocated context pointer
 * @param params Trapezoidal parameters (copied into context)
 * @param start_pos Starting position
 * @return elib_traj_trapezoid_err_t error code
 */
elib_traj_trapezoid_err_t elib_traj_trapezoid_init(
    elib_traj_trapezoid_ctx_t *ctx,
    const elib_traj_trapezoid_params_t *params,
    elib_traj_val_t start_pos);

/**
 * @brief Deinitialize
 * @param ctx Context pointer
 */
void elib_traj_trapezoid_deinit(elib_traj_trapezoid_ctx_t *ctx);

/**
 * @brief Reset to start position (can restart)
 * @param ctx Context pointer
 * @return elib_traj_trapezoid_err_t error code
 */
elib_traj_trapezoid_err_t elib_traj_trapezoid_reset(
    elib_traj_trapezoid_ctx_t *ctx);

/**
 * @brief Online mode: advance by dt, update internal state
 * @param ctx Context pointer
 * @param dt Time step, must be > 0
 * @return elib_traj_trapezoid_err_t error code
 */
elib_traj_trapezoid_err_t elib_traj_trapezoid_update(
    elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_val_t dt);

/**
 * @brief Get current pos/vel/acc
 * @param ctx Context pointer
 * @param state Output state pointer
 * @return elib_traj_trapezoid_err_t error code
 */
elib_traj_trapezoid_err_t elib_traj_trapezoid_get_state(
    const elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_state_t *state);

/**
 * @brief Get planner status
 * @param ctx Context pointer
 * @param status Output status pointer
 * @return elib_traj_trapezoid_err_t error code
 */
elib_traj_trapezoid_err_t elib_traj_trapezoid_get_status(
    const elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_status_t *status);

/**
 * @brief Offline mode: generate complete trajectory in one call
 * @param params Trapezoidal parameters
 * @param start_pos Starting position
 * @param dt Time step, must be > 0
 * @param points User-allocated output array
 * @param num_points Array size, must be > 0
 * @return elib_traj_trapezoid_err_t error code
 */
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
```

- [ ] **Step 3: Commit**

```bash
git add include/elib_traj_trapezoid_err.h include/elib_traj_trapezoid.h
git commit -m "feat: add trapezoidal profile public headers"
```

---

### Task 5: Trapezoid tests

**Files:**
- Modify: `test/test_elib_traj.c`

- [ ] **Step 1: Add trapezoid tests**

Append trapezoid test section to `test/test_elib_traj.c` (add include and test functions before `main`). The updated file:

```c
/* test_elib_traj.c - Trajectory Planner Unit Tests */
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include "../include/elib_traj_defs.h"
#include "../include/elib_traj_trapezoid.h"
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
        .max_vel = 1000.0f,
        .acc = 2000.0f,
        .dec = 2000.0f,
        .target_pos = 1000.0f,
    };
    return p;
}

static void test_trap_init_valid(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(err == ELIB_TRAJ_TRAPEZOID_OK);
    assert(ctx.initialized == 1);
}

static void test_trap_init_null_ctx(void) {
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_init(NULL, &p, 0.0f);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}

static void test_trap_init_null_params(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_init(&ctx, NULL, 0.0f);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}

static void test_trap_init_bad_max_vel(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.max_vel = 0.0f;
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}

static void test_trap_init_bad_acc(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.acc = -1.0f;
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}

static void test_trap_init_bad_dec(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.dec = 0.0f;
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}

static void test_trap_update_not_initialized(void) {
    elib_traj_trapezoid_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_update(&ctx, 0.01f);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED);
}

static void test_trap_update_bad_dt(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_update(&ctx, 0.0f);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}

static void test_trap_get_state_null(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_get_state(&ctx, NULL);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}

static void test_trap_get_status_null(void) {
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_get_status(&ctx, NULL);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
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
    /* max_vel=1000, acc=2000, dec=2000, target=1000, start=0
     * t_acc = 1000/2000 = 0.5s, d_acc = 0.5*2000*0.25 = 250
     * t_dec = 1000/2000 = 0.5s, d_dec = 250
     * d_const = 1000 - 500 = 500, t_const = 500/1000 = 0.5s
     * t_total = 1.5s */
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(err == ELIB_TRAJ_TRAPEZOID_OK);
    assert(fabsf(ctx.t_total - 1.5f) < EPSILON);

    /* At t=0.25s (mid acc phase): vel = 2000*0.25 = 500, pos = 0.5*2000*0.0625 = 62.5 */
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

    /* Run past t_total */
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
    assert(state.vel < 0.0f);  /* Moving in negative direction */
}

static void test_trap_triangle_profile(void) {
    /* Short distance that can't reach max_vel
     * max_vel=1000, acc=2000, dec=2000, target=100
     * d_acc + d_dec at max_vel = 250+250 = 500 > 100
     * v_peak = sqrt(2*100*2000*2000/(2000+2000)) = sqrt(200000) = ~447.2 */
    elib_traj_trapezoid_ctx_t ctx;
    elib_traj_trapezoid_params_t p = make_trap_params();
    p.target_pos = 100.0f;
    elib_traj_trapezoid_init(&ctx, &p, 0.0f);
    assert(ctx.t_const < EPSILON);  /* No constant velocity phase */

    /* Run to completion */
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

    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_update(&ctx, 0.01f);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED);
}

static void test_trap_generate_null_points(void) {
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_generate(
        &p, 0.0f, 0.01f, NULL, 100);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}

static void test_trap_generate_zero_count(void) {
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_state_t points[10];
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_generate(
        &p, 0.0f, 0.01f, points, 0);
    assert(err == ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM);
}

static void test_trap_generate_basic(void) {
    elib_traj_trapezoid_params_t p = make_trap_params();
    elib_traj_state_t points[151];  /* 1.5s / 0.01 = 150 + 1 */
    elib_traj_trapezoid_err_t err = elib_traj_trapezoid_generate(
        &p, 0.0f, 0.01f, points, 151);
    assert(err == ELIB_TRAJ_TRAPEZOID_OK);

    /* First point should be near start */
    assert(fabsf(points[0].pos - 0.0f) < 1.0f);

    /* Last point should be near target */
    assert(fabsf(points[150].pos - 1000.0f) < 1.0f);
    assert(fabsf(points[150].vel - 0.0f) < 1.0f);
}

int main(void) {
    printf("=== elib-trajectory tests ===\n\n");

    /* Utility tests */
    printf("--- Utility functions ---\n");
    RUN_TEST(test_clamp_mid);
    RUN_TEST(test_clamp_below_min);
    RUN_TEST(test_clamp_above_max);
    RUN_TEST(test_fabs_val_positive);
    RUN_TEST(test_fabs_val_negative);
    RUN_TEST(test_fabs_val_zero);

    /* Trapezoidal tests */
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

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `gcc -o test_elib_traj test/test_elib_traj.c -Iinclude -lm && ./test_elib_traj`
Expected: Compilation error — `elib_traj_trapezoid.h` included but functions not defined.

- [ ] **Step 3: Commit**

```bash
git add test/test_elib_traj.c
git commit -m "test: add trapezoidal profile test cases"
```

---

### Task 6: Trapezoid implementation

**Files:**
- Create: `src/elib_traj_trapezoid.c`

- [ ] **Step 1: Implement trapezoidal velocity profile**

Write `src/elib_traj_trapezoid.c`:

```c
/* elib_traj_trapezoid.c - Trapezoidal Velocity Profile Implementation */
#include "../include/elib_traj_trapezoid.h"
#include "elib_traj_util.h"
#include <math.h>
#include <string.h>

/**
 * @brief Compute pos/vel/acc at time t for a trapezoidal profile
 * @param ctx Context with precomputed segment times
 * @param t Absolute time from start
 * @param state Output state
 */
static void elib_traj_trapezoid_compute(const elib_traj_trapezoid_ctx_t *ctx,
                                         elib_traj_val_t t,
                                         elib_traj_state_t *state) {
    elib_traj_val_t dir = ctx->direction;

    if (t >= ctx->t_total) {
        /* Finished */
        state->pos = ctx->params.target_pos;
        state->vel = (elib_traj_val_t)0;
        state->acc = (elib_traj_val_t)0;
        return;
    }

    if (t < ctx->t_acc) {
        /* Phase 1: Acceleration */
        state->acc = ctx->params.acc * dir;
        state->vel = ctx->params.acc * t * dir;
        state->pos = ctx->start_pos +
                     (elib_traj_val_t)0.5 * ctx->params.acc * t * t * dir;
    } else if (t < ctx->t_acc + ctx->t_const) {
        /* Phase 2: Constant velocity */
        elib_traj_val_t dt_const = t - ctx->t_acc;
        state->acc = (elib_traj_val_t)0;
        state->vel = ctx->v_peak * dir;
        state->pos = ctx->start_pos +
                     (ctx->d_acc + ctx->v_peak * dt_const) * dir;
    } else {
        /* Phase 3: Deceleration */
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

    if (ctx == NULL || params == NULL) {
        return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    }
    if (params->max_vel <= (elib_traj_val_t)0) {
        return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    }
    if (params->acc <= (elib_traj_val_t)0) {
        return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    }
    if (params->dec <= (elib_traj_val_t)0) {
        return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    }

    memset(ctx, 0, sizeof(elib_traj_trapezoid_ctx_t));
    memcpy(&ctx->params, params, sizeof(elib_traj_trapezoid_params_t));
    ctx->start_pos = start_pos;

    elib_traj_val_t distance = params->target_pos - start_pos;
    elib_traj_val_t abs_distance = elib_traj_util_fabs(distance);

    /* Direction: +1 or -1 */
    if (distance > (elib_traj_val_t)0) {
        ctx->direction = (elib_traj_val_t)1;
    } else if (distance < (elib_traj_val_t)0) {
        ctx->direction = (elib_traj_val_t)-1;
    } else {
        /* Zero distance — already at target */
        ctx->direction = (elib_traj_val_t)1;
        ctx->distance = (elib_traj_val_t)0;
        ctx->t_acc = (elib_traj_val_t)0;
        ctx->t_const = (elib_traj_val_t)0;
        ctx->t_dec = (elib_traj_val_t)0;
        ctx->t_total = (elib_traj_val_t)0;
        ctx->v_peak = (elib_traj_val_t)0;
        ctx->d_acc = (elib_traj_val_t)0;
        ctx->d_const = (elib_traj_val_t)0;
        ctx->elapsed = (elib_traj_val_t)0;
        ctx->state.pos = start_pos;
        ctx->state.vel = (elib_traj_val_t)0;
        ctx->state.acc = (elib_traj_val_t)0;
        ctx->status = ELIB_TRAJ_STATUS_FINISHED;
        ctx->initialized = 1;
        return ELIB_TRAJ_TRAPEZOID_OK;
    }

    ctx->distance = abs_distance;

    /* Time to accelerate to max_vel */
    elib_traj_val_t t_to_max = params->max_vel / params->acc;
    /* Distance during acceleration */
    elib_traj_val_t d_acc = (elib_traj_val_t)0.5 * params->acc * t_to_max * t_to_max;
    /* Time to decelerate from max_vel to 0 */
    elib_traj_val_t t_to_stop = params->max_vel / params->dec;
    /* Distance during deceleration */
    elib_traj_val_t d_dec = (elib_traj_val_t)0.5 * params->dec * t_to_stop * t_to_stop;

    if (abs_distance >= d_acc + d_dec) {
        /* Normal trapezoidal profile */
        ctx->t_acc = t_to_max;
        ctx->t_dec = t_to_stop;
        ctx->v_peak = params->max_vel;
        ctx->d_acc = d_acc;
        ctx->d_const = abs_distance - d_acc - d_dec;
        ctx->t_const = ctx->d_const / params->max_vel;
    } else {
        /* Triangle profile — can't reach max_vel */
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

elib_traj_trapezoid_err_t elib_traj_trapezoid_reset(
    elib_traj_trapezoid_ctx_t *ctx) {

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
    elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_val_t dt) {

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
    const elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_state_t *state) {

    if (ctx == NULL || state == NULL) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED;

    *state = ctx->state;
    return ELIB_TRAJ_TRAPEZOID_OK;
}

elib_traj_trapezoid_err_t elib_traj_trapezoid_get_status(
    const elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_status_t *status) {

    if (ctx == NULL || status == NULL) return ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED;

    *status = ctx->status;
    return ELIB_TRAJ_TRAPEZOID_OK;
}

elib_traj_trapezoid_err_t elib_traj_trapezoid_generate(
    const elib_traj_trapezoid_params_t *params,
    elib_traj_val_t start_pos,
    elib_traj_val_t dt,
    elib_traj_state_t *points,
    uint32_t num_points) {

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
```

- [ ] **Step 2: Run tests to verify they pass**

Run: `gcc -o test_elib_traj test/test_elib_traj.c src/elib_traj_trapezoid.c src/elib_traj_util.c -Iinclude -lm && ./test_elib_traj`
Expected: All 26 tests PASSED.

- [ ] **Step 3: Commit**

```bash
git add src/elib_traj_trapezoid.c
git commit -m "feat: implement trapezoidal velocity profile with triangle fallback"
```

---

### Task 7: S-curve headers

**Files:**
- Create: `include/elib_traj_scurve_err.h`
- Create: `include/elib_traj_scurve.h`

- [ ] **Step 1: Create elib_traj_scurve_err.h**

Write `include/elib_traj_scurve_err.h`:

```c
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
```

- [ ] **Step 2: Create elib_traj_scurve.h**

Write `include/elib_traj_scurve.h`:

```c
/* elib_traj_scurve.h - S-Curve Velocity Profile (7-segment) */
#ifndef ELIB_TRAJ_SCURVE_H
#define ELIB_TRAJ_SCURVE_H

#include "elib_traj_defs.h"
#include "elib_traj_scurve_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* S-curve profile parameters */
typedef struct {
    elib_traj_val_t max_vel;     /* Maximum velocity, must be > 0 */
    elib_traj_val_t max_acc;     /* Maximum acceleration, must be > 0 */
    elib_traj_val_t max_dec;     /* Maximum deceleration, must be > 0 */
    elib_traj_val_t max_jerk;    /* Maximum jerk, must be > 0 */
    elib_traj_val_t target_pos;  /* Target position */
} elib_traj_scurve_params_t;

#define ELIB_TRAJ_SCURVE_PHASES 7

/* S-curve profile context (statically allocated by user) */
typedef struct {
    elib_traj_scurve_params_t params;
    elib_traj_val_t start_pos;
    elib_traj_val_t direction;            /* +1 or -1 */
    elib_traj_val_t t[ELIB_TRAJ_SCURVE_PHASES + 1]; /* Phase time boundaries */
    elib_traj_val_t v[ELIB_TRAJ_SCURVE_PHASES + 1]; /* Phase boundary velocities */
    elib_traj_val_t a[ELIB_TRAJ_SCURVE_PHASES + 1]; /* Phase boundary accelerations */
    elib_traj_val_t p[ELIB_TRAJ_SCURVE_PHASES + 1]; /* Phase boundary positions */
    elib_traj_val_t jerk[ELIB_TRAJ_SCURVE_PHASES];  /* Jerk value per phase */
    elib_traj_val_t elapsed;
    elib_traj_state_t state;
    elib_traj_status_t status;
    int initialized;
} elib_traj_scurve_ctx_t;

/**
 * @brief Initialize S-curve velocity profile
 * @param ctx User-allocated context pointer
 * @param params S-curve parameters (copied into context)
 * @param start_pos Starting position
 * @return elib_traj_scurve_err_t error code
 */
elib_traj_scurve_err_t elib_traj_scurve_init(
    elib_traj_scurve_ctx_t *ctx,
    const elib_traj_scurve_params_t *params,
    elib_traj_val_t start_pos);

/**
 * @brief Deinitialize
 * @param ctx Context pointer
 */
void elib_traj_scurve_deinit(elib_traj_scurve_ctx_t *ctx);

/**
 * @brief Reset to start position
 * @param ctx Context pointer
 * @return elib_traj_scurve_err_t error code
 */
elib_traj_scurve_err_t elib_traj_scurve_reset(
    elib_traj_scurve_ctx_t *ctx);

/**
 * @brief Online mode: advance by dt, update internal state
 * @param ctx Context pointer
 * @param dt Time step, must be > 0
 * @return elib_traj_scurve_err_t error code
 */
elib_traj_scurve_err_t elib_traj_scurve_update(
    elib_traj_scurve_ctx_t *ctx,
    elib_traj_val_t dt);

/**
 * @brief Get current pos/vel/acc
 * @param ctx Context pointer
 * @param state Output state pointer
 * @return elib_traj_scurve_err_t error code
 */
elib_traj_scurve_err_t elib_traj_scurve_get_state(
    const elib_traj_scurve_ctx_t *ctx,
    elib_traj_state_t *state);

/**
 * @brief Get planner status
 * @param ctx Context pointer
 * @param status Output status pointer
 * @return elib_traj_scurve_err_t error code
 */
elib_traj_scurve_err_t elib_traj_scurve_get_status(
    const elib_traj_scurve_ctx_t *ctx,
    elib_traj_status_t *status);

/**
 * @brief Offline mode: generate complete trajectory in one call
 * @param params S-curve parameters
 * @param start_pos Starting position
 * @param dt Time step, must be > 0
 * @param points User-allocated output array
 * @param num_points Array size, must be > 0
 * @return elib_traj_scurve_err_t error code
 */
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
```

- [ ] **Step 3: Commit**

```bash
git add include/elib_traj_scurve_err.h include/elib_traj_scurve.h
git commit -m "feat: add S-curve profile public headers"
```

---

### Task 8: S-curve tests

**Files:**
- Modify: `test/test_elib_traj.c`

- [ ] **Step 1: Add S-curve tests**

Add S-curve test section to `test/test_elib_traj.c`. Add `#include "../include/elib_traj_scurve.h"` after the existing includes, add test functions before `main`, and add them to `main`. Key test cases:

```c
/* Add include at top */
#include "../include/elib_traj_scurve.h"

/* === S-curve profile tests === */

static elib_traj_scurve_params_t make_scurve_params(void) {
    elib_traj_scurve_params_t p = {
        .max_vel = 1000.0f,
        .max_acc = 5000.0f,
        .max_dec = 5000.0f,
        .max_jerk = 50000.0f,
        .target_pos = 1000.0f,
    };
    return p;
}

static void test_scurve_init_valid(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_scurve_err_t err = elib_traj_scurve_init(&ctx, &p, 0.0f);
    assert(err == ELIB_TRAJ_SCURVE_OK);
    assert(ctx.initialized == 1);
}

static void test_scurve_init_null_ctx(void) {
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_scurve_err_t err = elib_traj_scurve_init(NULL, &p, 0.0f);
    assert(err == ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM);
}

static void test_scurve_init_null_params(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_err_t err = elib_traj_scurve_init(&ctx, NULL, 0.0f);
    assert(err == ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM);
}

static void test_scurve_init_bad_jerk(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    p.max_jerk = 0.0f;
    elib_traj_scurve_err_t err = elib_traj_scurve_init(&ctx, &p, 0.0f);
    assert(err == ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM);
}

static void test_scurve_update_not_initialized(void) {
    elib_traj_scurve_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    elib_traj_scurve_err_t err = elib_traj_scurve_update(&ctx, 0.01f);
    assert(err == ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED);
}

static void test_scurve_update_bad_dt(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_scurve_init(&ctx, &p, 0.0f);
    elib_traj_scurve_err_t err = elib_traj_scurve_update(&ctx, 0.0f);
    assert(err == ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM);
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

    /* Run past total time */
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

    /* Check that acc starts at 0 (smooth start) */
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
    elib_traj_scurve_err_t err = elib_traj_scurve_reset(&ctx);
    assert(err == ELIB_TRAJ_SCURVE_OK);
    assert(fabsf(ctx.elapsed - 0.0f) < EPSILON);
    assert(ctx.status == ELIB_TRAJ_STATUS_RUNNING);
}

static void test_scurve_deinit(void) {
    elib_traj_scurve_ctx_t ctx;
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_scurve_init(&ctx, &p, 0.0f);

    elib_traj_scurve_deinit(&ctx);
    assert(ctx.initialized == 0);

    elib_traj_scurve_err_t err = elib_traj_scurve_update(&ctx, 0.01f);
    assert(err == ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED);
}

static void test_scurve_generate_basic(void) {
    elib_traj_scurve_params_t p = make_scurve_params();
    elib_traj_state_t points[501];  /* Large enough for the profile */
    elib_traj_scurve_err_t err = elib_traj_scurve_generate(
        &p, 0.0f, 0.01f, points, 501);
    assert(err == ELIB_TRAJ_SCURVE_OK);

    /* First point near start */
    assert(fabsf(points[0].pos - 0.0f) < 1.0f);

    /* Last point near target */
    assert(fabsf(points[500].pos - 1000.0f) < 5.0f);
    assert(fabsf(points[500].vel - 0.0f) < 5.0f);
}

/* Add to main() inside the S-curve section: */
/*
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
*/
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `gcc -o test_elib_traj test/test_elib_traj.c src/elib_traj_trapezoid.c src/elib_traj_util.c -Iinclude -lm && ./test_elib_traj`
Expected: Compilation error — `elib_traj_scurve.h` included but functions not defined.

- [ ] **Step 3: Commit**

```bash
git add test/test_elib_traj.c
git commit -m "test: add S-curve profile test cases"
```

---

### Task 9: S-curve implementation

**Files:**
- Create: `src/elib_traj_scurve.c`

- [ ] **Step 1: Implement S-curve velocity profile**

Write `src/elib_traj_scurve.c`:

```c
/* elib_traj_scurve.c - S-Curve Velocity Profile (7-segment) Implementation */
#include "../include/elib_traj_scurve.h"
#include "elib_traj_util.h"
#include <math.h>
#include <string.h>

/**
 * @brief Compute pos/vel/acc at time t for an S-curve profile
 *
 * For phase k (0-indexed), with boundary values at t[k]:
 *   jerk = J_k (constant per phase)
 *   acc(τ) = a[k] + J_k * τ
 *   vel(τ) = v[k] + a[k] * τ + 0.5 * J_k * τ²
 *   pos(τ) = p[k] + v[k] * τ + 0.5 * a[k] * τ² + (1/6) * J_k * τ³
 */
static void elib_traj_scurve_compute(const elib_traj_scurve_ctx_t *ctx,
                                      elib_traj_val_t t,
                                      elib_traj_state_t *state) {
    elib_traj_val_t dir = ctx->direction;
    int num_phases = ELIB_TRAJ_SCURVE_PHASES;

    /* Find which phase t falls into */
    int phase = -1;
    for (int i = 0; i < num_phases; i++) {
        if (t < ctx->t[i + 1]) {
            phase = i;
            break;
        }
    }

    if (phase < 0) {
        /* Past the end — finished */
        state->pos = ctx->params.target_pos;
        state->vel = (elib_traj_val_t)0;
        state->acc = (elib_traj_val_t)0;
        return;
    }

    elib_traj_val_t tau = t - ctx->t[phase];  /* Time within this phase */
    elib_traj_val_t J = ctx->jerk[phase];

    /* Compute in normalized space (positive direction), then apply direction */
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

/**
 * @brief Precompute 7-segment boundaries for full S-curve
 * @return 0 on success, -1 if can't fit (needs degeneration)
 */
static int elib_traj_scurve_precompute_full(elib_traj_scurve_ctx_t *ctx) {
    elib_traj_val_t max_vel = ctx->params.max_vel;
    elib_traj_val_t max_acc = ctx->params.max_acc;
    elib_traj_val_t max_dec = ctx->params.max_dec;
    elib_traj_val_t max_jerk = ctx->params.max_jerk;

    /* Phase 1: jerk up, acc 0 → max_acc */
    elib_traj_val_t T1 = max_acc / max_jerk;
    /* Phase 2: const acc at max_acc */
    /* Phase 3: jerk down, acc max_acc → 0, reaching max_vel */
    elib_traj_val_t T3 = max_acc / max_jerk;
    /* Velocity gained in phase 1: 0.5 * max_acc * T1 */
    /* Velocity gained in phase 3: same */
    /* Velocity gained in phase 2: max_vel - max_acc*T1 */
    elib_traj_val_t v_from_jerk = max_acc * T1;  /* Total vel from phases 1+3 */
    if (max_vel < v_from_jerk) {
        return -1;  /* max_vel too low for this jerk/acc — need degeneration */
    }
    elib_traj_val_t T2 = (max_vel - v_from_jerk) / max_acc;

    /* Phase 5: jerk down, acc 0 → -max_dec */
    elib_traj_val_t T5 = max_dec / max_jerk;
    /* Phase 6: const dec at -max_dec */
    elib_traj_val_t T7 = max_dec / max_jerk;
    elib_traj_val_t v_from_dec_jerk = max_dec * T5;
    if (max_vel < v_from_dec_jerk) {
        return -1;
    }
    elib_traj_val_t T6 = (max_vel - v_from_dec_jerk) / max_dec;

    /* Phase 4: const vel (will be computed after distance check) */

    /* Compute distance for accel phase (1+2+3) */
    elib_traj_val_t d_acc = (elib_traj_val_t)0;
    {
        /* Phase 1 */
        elib_traj_val_t a0 = 0, v0 = 0, p0 = 0;
        elib_traj_val_t a1 = max_acc;
        elib_traj_val_t v1 = v0 + (elib_traj_val_t)0.5 * max_acc * T1;
        elib_traj_val_t p1 = p0 + v0 * T1 + (elib_traj_val_t)(1.0/6.0) * max_acc * T1 * T1 * T1;
        /* Phase 2 */
        elib_traj_val_t v2 = v1 + max_acc * T2;
        elib_traj_val_t p2 = p1 + v1 * T2 + (elib_traj_val_t)0.5 * max_acc * T2 * T2;
        /* Phase 3 */
        elib_traj_val_t J3 = -max_jerk;
        elib_traj_val_t v3 = v2 + max_acc * T3 + (elib_traj_val_t)0.5 * J3 * T3 * T3;
        elib_traj_val_t p3 = p2 + v2 * T3 + (elib_traj_val_t)0.5 * max_acc * T3 * T3 +
                              (elib_traj_val_t)(1.0/6.0) * J3 * T3 * T3 * T3;
        (void)a0; (void)a1; (void)v3;
        d_acc = p3;
    }

    /* Compute distance for decel phase (5+6+7) */
    elib_traj_val_t d_dec = (elib_traj_val_t)0;
    {
        /* Phase 5 */
        elib_traj_val_t v0 = max_vel, a0 = 0, p0 = 0;
        elib_traj_val_t J5 = -max_jerk;
        elib_traj_val_t v1 = v0 + (elib_traj_val_t)0.5 * J5 * T5 * T5;
        elib_traj_val_t p1 = p0 + v0 * T5 + (elib_traj_val_t)(1.0/6.0) * J5 * T5 * T5 * T5;
        /* Phase 6 */
        elib_traj_val_t v2 = v1 + (-max_dec) * T6;
        elib_traj_val_t p2 = p1 + v1 * T6 + (elib_traj_val_t)0.5 * (-max_dec) * T6 * T6;
        /* Phase 7 */
        elib_traj_val_t J7 = max_jerk;
        elib_traj_val_t v3 = v2 + (-max_dec) * T7 + (elib_traj_val_t)0.5 * J7 * T7 * T7;
        elib_traj_val_t p3 = p2 + v2 * T7 + (elib_traj_val_t)0.5 * (-max_dec) * T7 * T7 +
                              (elib_traj_val_t)(1.0/6.0) * J7 * T7 * T7 * T7;
        (void)a0; (void)v3;
        d_dec = p3;
    }

    elib_traj_val_t abs_dist = ctx->distance;
    elib_traj_val_t d_total_needed = d_acc + d_dec;

    if (abs_dist < d_total_needed) {
        return -1;  /* Not enough distance for full S-curve */
    }

    /* Phase 4 duration */
    elib_traj_val_t T4 = (abs_dist - d_acc - d_dec) / max_vel;

    /* Store time boundaries */
    ctx->t[0] = 0;
    ctx->t[1] = T1;
    ctx->t[2] = T1 + T2;
    ctx->t[3] = T1 + T2 + T3;
    ctx->t[4] = T1 + T2 + T3 + T4;
    ctx->t[5] = T1 + T2 + T3 + T4 + T5;
    ctx->t[6] = T1 + T2 + T3 + T4 + T5 + T6;
    ctx->t[7] = T1 + T2 + T3 + T4 + T5 + T6 + T7;

    /* Store jerk per phase */
    ctx->jerk[0] = max_jerk;    /* Phase 1: jerk up */
    ctx->jerk[1] = 0;           /* Phase 2: const acc */
    ctx->jerk[2] = -max_jerk;   /* Phase 3: jerk down */
    ctx->jerk[3] = 0;           /* Phase 4: const vel */
    ctx->jerk[4] = -max_jerk;   /* Phase 5: jerk down to dec */
    ctx->jerk[5] = 0;           /* Phase 6: const dec */
    ctx->jerk[6] = max_jerk;    /* Phase 7: jerk up to zero */

    /* Compute boundary values by simulation */
    ctx->a[0] = 0;
    ctx->v[0] = 0;
    ctx->p[0] = 0;

    for (int i = 0; i < ELIB_TRAJ_SCURVE_PHASES; i++) {
        elib_traj_val_t Ti = ctx->t[i + 1] - ctx->t[i];
        elib_traj_val_t J = ctx->jerk[i];
        ctx->a[i + 1] = ctx->a[i] + J * Ti;
        ctx->v[i + 1] = ctx->v[i] + ctx->a[i] * Ti +
                          (elib_traj_val_t)0.5 * J * Ti * Ti;
        ctx->p[i + 1] = ctx->p[i] + ctx->v[i] * Ti +
                          (elib_traj_val_t)0.5 * ctx->a[i] * Ti * Ti +
                          (elib_traj_val_t)(1.0/6.0) * J * Ti * Ti * Ti;
    }

    return 0;
}

/**
 * @brief Degenerate S-curve: reduce to trapezoidal-like with smooth jerk
 * When distance is too short for full 7-segment, remove const-acc and/or
 * const-vel phases and use a simplified profile.
 */
static void elib_traj_scurve_precompute_degenerate(elib_traj_scurve_ctx_t *ctx) {
    elib_traj_val_t abs_dist = ctx->distance;
    elib_traj_val_t max_acc = ctx->params.max_acc;
    elib_traj_val_t max_dec = ctx->params.max_dec;
    elib_traj_val_t max_jerk = ctx->params.max_jerk;

    /* Use a simplified approach: find max achievable velocity
     * with symmetric jerk-limited accel/decel.
     *
     * For symmetric case (acc=dec, jerk same):
     *   Phase 1 (jerk): T_j = a_max / j_max
     *   Distance for jerk up+down at velocity v:
     *     d_jerk = v * T_j (approx)
     *   If no const-acc phase: v_peak = a_max * T_j = a_max^2 / j_max
     *   Distance = v_peak * (T_j + T_j) = 2 * v_peak * T_j
     *
     * Simplified: use trapezoidal with the available distance,
     * but cap acc/dec by what jerk allows.
     */

    /* Find peak velocity achievable with jerk-limited triangle profile */
    /* For a jerk-limited triangle: v_peak^3 / (jerk * v_peak) ... */
    /* Use iterative approach: try v_peak, compute distance */

    /* Simple approach: use trapezoidal fallback with effective acc/dec
     * limited by jerk. T_jerk = a_max / j_max */
    elib_traj_val_t T_jerk = max_acc / max_jerk;
    elib_traj_val_t T_jerk_dec = max_dec / max_jerk;

    /* For the degenerate case, use a 5-segment or 3-segment profile.
     * Simplified: compute v_peak from distance equation for symmetric case.
     *
     * For symmetric jerk profile (no const-acc, no const-vel):
     *   d = v_peak * (T_j + T_j) = 2 * v_peak * T_jerk
     *   v_peak = d / (2 * T_jerk)
     *
     * But this may exceed max_acc. Cap it.
     */
    elib_traj_val_t v_peak = abs_dist / ((elib_traj_val_t)2 * T_jerk);

    /* Check if this v_peak needs less than max_acc */
    elib_traj_val_t acc_needed = v_peak / T_jerk;
    if (acc_needed > max_acc) {
        /* Need to reduce v_peak or use const-acc phase */
        /* For simplicity, cap at max_acc and compute T_acc */
        elib_traj_val_t T_acc = (abs_dist - max_acc * T_jerk * T_jerk) / max_acc;
        if (T_acc < (elib_traj_val_t)0) T_acc = 0;
        v_peak = max_acc * T_jerk + max_acc * T_acc;

        /* 5-segment: jerk_up, const_acc, jerk_down, jerk_down, const_dec, jerk_up */
        /* Simplify to 5 segments with symmetric accel/decel */
        elib_traj_val_t T1 = T_jerk;
        elib_traj_val_t T2 = T_acc;
        elib_traj_val_t T3 = T_jerk;
        elib_traj_val_t T4 = 0;  /* No const-vel */
        elib_traj_val_t T5 = T_jerk_dec;
        elib_traj_val_t T6 = T_acc;  /* Symmetric */
        elib_traj_val_t T7 = T_jerk_dec;

        /* Verify distance */
        elib_traj_val_t d_check = v_peak * (T1 + T2 + T3);  /* Approx */
        if (d_check > abs_dist * (elib_traj_val_t)1.01) {
            /* Still too much, reduce T_acc */
            T2 = 0;
            T6 = 0;
            v_peak = max_acc * T_jerk;
        }

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
    } else {
        /* Pure jerk triangle (no const-acc, no const-vel) */
        ctx->t[0] = 0;
        ctx->t[1] = T_jerk;
        ctx->t[2] = T_jerk;           /* No const-acc */
        ctx->t[3] = (elib_traj_val_t)2 * T_jerk;
        ctx->t[4] = (elib_traj_val_t)2 * T_jerk;  /* No const-vel */
        ctx->t[5] = (elib_traj_val_t)2 * T_jerk + T_jerk_dec;
        ctx->t[6] = (elib_traj_val_t)2 * T_jerk + T_jerk_dec;  /* No const-dec */
        ctx->t[7] = (elib_traj_val_t)2 * T_jerk + (elib_traj_val_t)2 * T_jerk_dec;

        ctx->jerk[0] = max_jerk;
        ctx->jerk[1] = 0;
        ctx->jerk[2] = -max_jerk;
        ctx->jerk[3] = 0;
        ctx->jerk[4] = -max_jerk;
        ctx->jerk[5] = 0;
        ctx->jerk[6] = max_jerk;
    }

    /* Compute boundary values by simulation */
    ctx->a[0] = 0;
    ctx->v[0] = 0;
    ctx->p[0] = 0;

    for (int i = 0; i < ELIB_TRAJ_SCURVE_PHASES; i++) {
        elib_traj_val_t Ti = ctx->t[i + 1] - ctx->t[i];
        elib_traj_val_t J = ctx->jerk[i];
        ctx->a[i + 1] = ctx->a[i] + J * Ti;
        ctx->v[i + 1] = ctx->v[i] + ctx->a[i] * Ti +
                          (elib_traj_val_t)0.5 * J * Ti * Ti;
        ctx->p[i + 1] = ctx->p[i] + ctx->v[i] * Ti +
                          (elib_traj_val_t)0.5 * ctx->a[i] * Ti * Ti +
                          (elib_traj_val_t)(1.0/6.0) * J * Ti * Ti * Ti;
    }

    /* Scale positions to match actual distance */
    elib_traj_val_t computed_dist = ctx->p[ELIB_TRAJ_SCURVE_PHASES];
    if (computed_dist > (elib_traj_val_t)0) {
        elib_traj_val_t scale = abs_dist / computed_dist;
        for (int i = 0; i <= ELIB_TRAJ_SCURVE_PHASES; i++) {
            ctx->p[i] *= scale;
            ctx->v[i] *= sqrtf(scale);
        }
        ctx->t[7] *= sqrtf(scale);
        /* Recompute time boundaries proportionally */
        for (int i = 1; i < ELIB_TRAJ_SCURVE_PHASES; i++) {
            ctx->t[i] *= sqrtf(scale);
        }
    }
}

elib_traj_scurve_err_t elib_traj_scurve_init(
    elib_traj_scurve_ctx_t *ctx,
    const elib_traj_scurve_params_t *params,
    elib_traj_val_t start_pos) {

    if (ctx == NULL || params == NULL) {
        return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    }
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
        /* Zero distance */
        ctx->direction = (elib_traj_val_t)1;
        ctx->distance = 0;
        ctx->state.pos = start_pos;
        ctx->state.vel = 0;
        ctx->state.acc = 0;
        ctx->status = ELIB_TRAJ_STATUS_FINISHED;
        ctx->initialized = 1;
        return ELIB_TRAJ_SCURVE_OK;
    }

    ctx->distance = abs_distance;

    /* Try full 7-segment profile first */
    if (elib_traj_scurve_precompute_full(ctx) != 0) {
        /* Fall back to degenerate profile */
        elib_traj_scurve_precompute_degenerate(ctx);
    }

    ctx->elapsed = 0;
    ctx->state.pos = start_pos;
    ctx->state.vel = 0;
    ctx->state.acc = 0;
    ctx->status = ELIB_TRAJ_STATUS_RUNNING;
    ctx->initialized = 1;

    return ELIB_TRAJ_SCURVE_OK;
}

void elib_traj_scurve_deinit(elib_traj_scurve_ctx_t *ctx) {
    if (ctx == NULL) return;
    ctx->initialized = 0;
}

elib_traj_scurve_err_t elib_traj_scurve_reset(
    elib_traj_scurve_ctx_t *ctx) {

    if (ctx == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED;

    ctx->elapsed = 0;
    ctx->state.pos = ctx->start_pos;
    ctx->state.vel = 0;
    ctx->state.acc = 0;
    ctx->status = ELIB_TRAJ_STATUS_RUNNING;

    return ELIB_TRAJ_SCURVE_OK;
}

elib_traj_scurve_err_t elib_traj_scurve_update(
    elib_traj_scurve_ctx_t *ctx,
    elib_traj_val_t dt) {

    if (ctx == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED;
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
    const elib_traj_scurve_ctx_t *ctx,
    elib_traj_state_t *state) {

    if (ctx == NULL || state == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED;

    *state = ctx->state;
    return ELIB_TRAJ_SCURVE_OK;
}

elib_traj_scurve_err_t elib_traj_scurve_get_status(
    const elib_traj_scurve_ctx_t *ctx,
    elib_traj_status_t *status) {

    if (ctx == NULL || status == NULL) return ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM;
    if (!ctx->initialized) return ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED;

    *status = ctx->status;
    return ELIB_TRAJ_SCURVE_OK;
}

elib_traj_scurve_err_t elib_traj_scurve_generate(
    const elib_traj_scurve_params_t *params,
    elib_traj_val_t start_pos,
    elib_traj_val_t dt,
    elib_traj_state_t *points,
    uint32_t num_points) {

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
```

- [ ] **Step 2: Run all tests**

Run: `gcc -o test_elib_traj test/test_elib_traj.c src/elib_traj_trapezoid.c src/elib_traj_scurve.c src/elib_traj_util.c -Iinclude -lm && ./test_elib_traj`
Expected: All 39 tests PASSED.

- [ ] **Step 3: Commit**

```bash
git add src/elib_traj_scurve.c
git commit -m "feat: implement S-curve 7-segment velocity profile with degeneration"
```

---

### Task 10: Umbrella header

**Files:**
- Create: `include/elib_traj.h`

- [ ] **Step 1: Create elib_traj.h**

Write `include/elib_traj.h`:

```c
/* elib_traj.h - Trajectory Planner Main Header */
#ifndef ELIB_TRAJ_H
#define ELIB_TRAJ_H

#include "elib_traj_defs.h"
#include "elib_traj_trapezoid.h"
#include "elib_traj_scurve.h"

#endif /* ELIB_TRAJ_H */
```

- [ ] **Step 2: Run all tests**

Run: `gcc -o test_elib_traj test/test_elib_traj.c src/elib_traj_trapezoid.c src/elib_traj_scurve.c src/elib_traj_util.c -Iinclude -lm && ./test_elib_traj`
Expected: All 39 tests PASSED.

- [ ] **Step 3: Commit**

```bash
git add include/elib_traj.h
git commit -m "feat: add umbrella header"
```

---

### Task 11: README

**Files:**
- Create: `README.md`

- [ ] **Step 1: Create README.md**

Write `README.md`:

```markdown
# elib-trajectory

嵌入式轨迹规划算法库，支持梯形速度规划和 S 曲线速度规划。

## Features

- **梯形速度规划** - 匀加速-匀速-匀减速，自动退化为三角形
- **S 曲线速度规划** - 7 段 jerk 限制，运动平滑无冲击
- **双模式** - 在线模式（周期调用）和离线模式（一次性生成）
- **输出完整** - 同时输出位置、速度、加速度
- **零动态内存** - 所有资源由用户静态分配
- **类型可配置** - 默认 float，可宏覆盖为定点数

## Quick Start

### 1. 梯形速度规划

```c
#include "elib_traj.h"

/* 声明上下文 */
elib_traj_trapezoid_ctx_t ctx;

/* 初始化 */
elib_traj_trapezoid_params_t params = {
    .max_vel = 1000.0f,    /* 最大速度 */
    .acc = 2000.0f,        /* 加速度 */
    .dec = 2000.0f,        /* 减速度 */
    .target_pos = 1000.0f, /* 目标位置 */
};
elib_traj_trapezoid_init(&ctx, &params, 0.0f);

/* 在线模式：每周期调用 */
elib_traj_trapezoid_update(&ctx, 0.01f);  /* 10ms */
elib_traj_state_t state;
elib_traj_trapezoid_get_state(&ctx, &state);
printf("pos=%.1f vel=%.1f acc=%.1f\n", state.pos, state.vel, state.acc);
```

### 2. S 曲线速度规划

```c
elib_traj_scurve_ctx_t ctx;
elib_traj_scurve_params_t params = {
    .max_vel = 1000.0f,
    .max_acc = 5000.0f,
    .max_dec = 5000.0f,
    .max_jerk = 50000.0f,
    .target_pos = 1000.0f,
};
elib_traj_scurve_init(&ctx, &params, 0.0f);

/* 在线模式 */
elib_traj_scurve_update(&ctx, 0.01f);
elib_traj_state_t state;
elib_traj_scurve_get_state(&ctx, &state);
```

### 3. 离线模式

```c
elib_traj_state_t trajectory[151];
elib_traj_trapezoid_generate(&params, 0.0f, 0.01f, trajectory, 151);
/* trajectory[0]..trajectory[150] 包含完整轨迹 */
```

### 4. 配合 PID 闭环使用

```c
/* 轨迹规划器作为 PID 的前级 */
elib_traj_trapezoid_update(&traj_ctx, dt);
elib_traj_state_t setpoint;
elib_traj_trapezoid_get_state(&traj_ctx, &setpoint);

float actual = read_sensor();
float output;
elib_pid_pos_compute(&pid_ctx, setpoint.pos, actual, &output);
actuator_set(output);
```

## API Reference

### 梯形速度规划 (`elib_traj_trapezoid.h`)

- `elib_traj_trapezoid_init(ctx, params, start_pos)` - 初始化
- `elib_traj_trapezoid_deinit(ctx)` - 反初始化
- `elib_traj_trapezoid_reset(ctx)` - 重置
- `elib_traj_trapezoid_update(ctx, dt)` - 在线模式更新
- `elib_traj_trapezoid_get_state(ctx, &state)` - 获取状态
- `elib_traj_trapezoid_get_status(ctx, &status)` - 获取规划状态
- `elib_traj_trapezoid_generate(params, start_pos, dt, points[], num)` - 离线生成

### S 曲线速度规划 (`elib_traj_scurve.h`)

- `elib_traj_scurve_init(ctx, params, start_pos)` - 初始化
- `elib_traj_scurve_deinit(ctx)` - 反初始化
- `elib_traj_scurve_reset(ctx)` - 重置
- `elib_traj_scurve_update(ctx, dt)` - 在线模式更新
- `elib_traj_scurve_get_state(ctx, &state)` - 获取状态
- `elib_traj_scurve_get_status(ctx, &status)` - 获取规划状态
- `elib_traj_scurve_generate(params, start_pos, dt, points[], num)` - 离线生成

## Build

无构建系统，将 `include/` 和 `src/` 加入你的项目：

```bash
# 只用梯形规划
gcc app.c src/elib_traj_trapezoid.c src/elib_traj_util.c -Iinclude -lm

# 只用 S 曲线规划
gcc app.c src/elib_traj_scurve.c src/elib_traj_util.c -Iinclude -lm

# 两者都用
gcc app.c src/elib_traj_trapezoid.c src/elib_traj_scurve.c src/elib_traj_util.c -Iinclude -lm
```

## License

MIT License
```

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: add README with usage examples and API reference"
```

---

### Task 12: Final verification

- [ ] **Step 1: Run full test suite**

Run: `gcc -o test_elib_traj test/test_elib_traj.c src/elib_traj_trapezoid.c src/elib_traj_scurve.c src/elib_traj_util.c -Iinclude -lm && ./test_elib_traj`
Expected: All 39 tests PASSED, exit code 0.

- [ ] **Step 2: Verify file structure**

Run: `find . -not -path './.git/*' -type f | sort`
Expected output:
```
.gitattributes
LICENSE
README.md
docs/superpowers/plans/2026-05-13-elib-trajectory.md
docs/superpowers/specs/2026-05-13-elib-trajectory-design.md
include/elib_traj.h
include/elib_traj_defs.h
include/elib_traj_scurve.h
include/elib_traj_scurve_err.h
include/elib_traj_trapezoid.h
include/elib_traj_trapezoid_err.h
scripts/setup-push-remote.bat
scripts/setup-push-remote.sh
src/elib_traj_scurve.c
src/elib_traj_trapezoid.c
src/elib_traj_util.c
src/elib_traj_util.h
test/test_elib_traj.c
```

- [ ] **Step 3: Verify only trapezoid compiles independently**

Run: `gcc -o test_trap_only test/test_elib_traj.c src/elib_traj_trapezoid.c src/elib_traj_util.c -Iinclude -lm`
Note: This will fail because test file includes scurve header. The important verification is that `elib_traj_scurve.c` is not needed for trapezoid. Verify by checking that the `.c` files are independent.

- [ ] **Step 4: Final commit**

```bash
git add -A
git commit -m "chore: complete elib-trajectory library"
```
