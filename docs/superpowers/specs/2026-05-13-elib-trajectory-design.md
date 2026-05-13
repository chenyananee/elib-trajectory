# elib-trajectory 设计规格

## 概述

嵌入式轨迹规划算法库，支持多种运动轨迹生成算法，遵循 elib 家族库规范。每个算法独立一个 .c 文件，用户按需编译。支持在线模式（周期调用获取下一目标点）和离线模式（一次性生成完整轨迹）。

## 需求

1. 包含梯形速度规划和 S 曲线速度规划两种算法
2. 每个算法独立源文件，可单独编译
3. 用户只需包含一个伞形头文件 `elib_traj.h`
4. 禁止动态内存分配，所有资源由用户静态分配
5. 支持在线模式（周期传入 dt，获取当前 pos/vel/acc）
6. 支持离线模式（一次性生成完整轨迹到用户提供的数组）
7. 输出位置、速度、加速度三个量
8. 标量输出（单轴），多轴用户对每个轴独立创建实例

## 数据类型

库内使用 typedef 宏定义值类型，默认 `float`，用户可宏覆盖：

```c
#ifndef ELIB_TRAJ_VAL_T
#define ELIB_TRAJ_VAL_T float
#endif
typedef ELIB_TRAJ_VAL_T elib_traj_val_t;
```

用户在编译时通过 `-DELIB_TRAJ_VAL_T=int32_t` 或在 include 前 `#define ELIB_TRAJ_VAL_T int32_t` 切换为定点数。

## 目录结构

```
elib-trajectory/
  include/
    elib_traj.h                  # 总头文件，include 所有子头文件
    elib_traj_defs.h             # 公共定义：值类型、输出结构体、状态枚举
    elib_traj_trapezoid.h        # 梯形速度规划公开 API
    elib_traj_trapezoid_err.h    # 梯形错误码
    elib_traj_scurve.h           # S 曲线速度规划公开 API
    elib_traj_scurve_err.h       # S 曲线错误码
  src/
    elib_traj_util.h             # 内部工具函数（限幅等）
    elib_traj_util.c             # 工具实现
    elib_traj_trapezoid.c        # 梯形实现
    elib_traj_scurve.c           # S 曲线实现
  test/
    test_elib_traj.c             # 单元测试
  LICENSE
  README.md
  .gitattributes
```

## 公共类型 (`elib_traj_defs.h`)

### 输出状态结构体

```c
typedef struct {
    elib_traj_val_t pos;   /* 位置 */
    elib_traj_val_t vel;   /* 速度 */
    elib_traj_val_t acc;   /* 加速度 */
} elib_traj_state_t;
```

### 规划状态枚举

```c
typedef enum {
    ELIB_TRAJ_STATUS_IDLE = 0,     /* 未启动 */
    ELIB_TRAJ_STATUS_RUNNING,      /* 规划中 */
    ELIB_TRAJ_STATUS_FINISHED,     /* 已完成 */
} elib_traj_status_t;
```

## 梯形速度规划 (`elib_traj_trapezoid.h`)

### 错误码

```c
typedef enum {
    ELIB_TRAJ_TRAPEZOID_OK = 0,
    ELIB_TRAJ_TRAPEZOID_ERR_INVALID_PARAM,
    ELIB_TRAJ_TRAPEZOID_ERR_NOT_INITIALIZED,
} elib_traj_trapezoid_err_t;
```

### 参数结构体

```c
typedef struct {
    elib_traj_val_t max_vel;    /* 最大速度，必须 > 0 */
    elib_traj_val_t acc;        /* 加速度，必须 > 0 */
    elib_traj_val_t dec;        /* 减速度，必须 > 0（可不等于 acc） */
    elib_traj_val_t target_pos; /* 目标位置 */
} elib_traj_trapezoid_params_t;
```

### 上下文结构体

```c
typedef struct {
    elib_traj_trapezoid_params_t params;
    elib_traj_val_t start_pos;      /* 起始位置 */
    elib_traj_val_t t_acc;          /* 加速段时间 */
    elib_traj_val_t t_const;        /* 匀速段时间 */
    elib_traj_val_t t_dec;          /* 减速段时间 */
    elib_traj_val_t t_total;        /* 总时间 */
    elib_traj_val_t elapsed;        /* 已经过时间 */
    elib_traj_state_t state;        /* 当前输出状态 */
    elib_traj_status_t status;      /* 规划状态 */
    int initialized;
} elib_traj_trapezoid_ctx_t;
```

### API

```c
/* 初始化梯形速度规划 */
elib_traj_trapezoid_err_t elib_traj_trapezoid_init(
    elib_traj_trapezoid_ctx_t *ctx,
    const elib_traj_trapezoid_params_t *params
);

/* 反初始化 */
void elib_traj_trapezoid_deinit(elib_traj_trapezoid_ctx_t *ctx);

/* 重置到起始状态（可重新启动） */
elib_traj_trapezoid_err_t elib_traj_trapezoid_reset(
    elib_traj_trapezoid_ctx_t *ctx
);

/* 在线模式：传入时间步长，更新内部状态 */
elib_traj_trapezoid_err_t elib_traj_trapezoid_update(
    elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_val_t dt
);

/* 获取当前 pos/vel/acc */
elib_traj_trapezoid_err_t elib_traj_trapezoid_get_state(
    const elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_state_t *state
);

/* 获取规划状态 */
elib_traj_trapezoid_err_t elib_traj_trapezoid_get_status(
    const elib_traj_trapezoid_ctx_t *ctx,
    elib_traj_status_t *status
);

/* 离线模式：一次性生成完整轨迹 */
elib_traj_trapezoid_err_t elib_traj_trapezoid_generate(
    const elib_traj_trapezoid_params_t *params,
    elib_traj_val_t start_pos,
    elib_traj_val_t dt,
    elib_traj_state_t *points,
    uint32_t num_points
);
```

### 梯形速度规划原理

假设从 start_pos 到 target_pos，距离为 D = |target_pos - start_pos|。

**时间计算**：

1. 加速到 max_vel 所需时间：`t_to_max = max_vel / acc`
2. 加速段距离：`d_acc = 0.5 * acc * t_to_max²`
3. 从 max_vel 减速到 0 所需时间：`t_to_stop = max_vel / dec`
4. 减速段距离：`d_dec = 0.5 * dec * t_to_stop²`

**情况判断**：

- **正常梯形**（D >= d_acc + d_dec）：有匀速段
  - `t_acc = t_to_max`
  - `t_dec = t_to_stop`
  - `t_const = (D - d_acc - d_dec) / max_vel`

- **三角形**（D < d_acc + d_dec 且 D > 0）：无匀速段，速度达不到 max_vel
  - 通过速度连续方程求解实际最大速度 v_peak
  - `v_peak = sqrt(2 * D * acc * dec / (acc + dec))`
  - `t_acc = v_peak / acc`
  - `t_dec = v_peak / dec`
  - `t_const = 0`

- **零距离**（D == 0）：直接完成

**分段计算**（以正方向为例，start_pos = 0）：

```
Phase 1: t ∈ [0, t_acc)
    vel(t) = acc * t
    pos(t) = 0.5 * acc * t²
    acc(t) = acc

Phase 2: t ∈ [t_acc, t_acc + t_const)
    vel(t) = max_vel
    pos(t) = d_acc + max_vel * (t - t_acc)
    acc(t) = 0

Phase 3: t ∈ [t_acc + t_const, t_total)
    t_dec_elapsed = t - t_acc - t_const
    vel(t) = max_vel - dec * t_dec_elapsed
    pos(t) = d_acc + max_vel * t_const + max_vel * t_dec_elapsed - 0.5 * dec * t_dec_elapsed²
    acc(t) = -dec

Phase 4: t >= t_total
    vel = 0
    pos = target_pos
    acc = 0
```

反方向（target_pos < start_pos）同理，速度和加速度取反。

## S 曲线速度规划 (`elib_traj_scurve.h`)

### 错误码

```c
typedef enum {
    ELIB_TRAJ_SCURVE_OK = 0,
    ELIB_TRAJ_SCURVE_ERR_INVALID_PARAM,
    ELIB_TRAJ_SCURVE_ERR_NOT_INITIALIZED,
} elib_traj_scurve_err_t;
```

### 参数结构体

```c
typedef struct {
    elib_traj_val_t max_vel;     /* 最大速度，必须 > 0 */
    elib_traj_val_t max_acc;     /* 最大加速度，必须 > 0 */
    elib_traj_val_t max_dec;     /* 最大减速度，必须 > 0 */
    elib_traj_val_t max_jerk;    /* 最大加加速度，必须 > 0 */
    elib_traj_val_t target_pos;  /* 目标位置 */
} elib_traj_scurve_params_t;
```

### 上下文结构体

```c
typedef struct {
    elib_traj_scurve_params_t params;
    elib_traj_val_t start_pos;       /* 起始位置 */
    elib_traj_val_t t[8];            /* 7 段时间边界：t[0]=0, t[1]..t[7] */
    elib_traj_val_t v[8];            /* 各段边界速度 */
    elib_traj_val_t a[8];            /* 各段边界加速度 */
    elib_traj_val_t p[8];            /* 各段边界位置 */
    elib_traj_val_t elapsed;         /* 已经过时间 */
    elib_traj_state_t state;         /* 当前输出状态 */
    elib_traj_status_t status;       /* 规划状态 */
    int initialized;
} elib_traj_scurve_ctx_t;
```

### API

```c
/* 初始化 S 曲线速度规划 */
elib_traj_scurve_err_t elib_traj_scurve_init(
    elib_traj_scurve_ctx_t *ctx,
    const elib_traj_scurve_params_t *params
);

/* 反初始化 */
void elib_traj_scurve_deinit(elib_traj_scurve_ctx_t *ctx);

/* 重置到起始状态 */
elib_traj_scurve_err_t elib_traj_scurve_reset(
    elib_traj_scurve_ctx_t *ctx
);

/* 在线模式：传入时间步长，更新内部状态 */
elib_traj_scurve_err_t elib_traj_scurve_update(
    elib_traj_scurve_ctx_t *ctx,
    elib_traj_val_t dt
);

/* 获取当前 pos/vel/acc */
elib_traj_scurve_err_t elib_traj_scurve_get_state(
    const elib_traj_scurve_ctx_t *ctx,
    elib_traj_state_t *state
);

/* 获取规划状态 */
elib_traj_scurve_err_t elib_traj_scurve_get_status(
    const elib_traj_scurve_ctx_t *ctx,
    elib_traj_status_t *status
);

/* 离线模式：一次性生成完整轨迹 */
elib_traj_scurve_err_t elib_traj_scurve_generate(
    const elib_traj_scurve_params_t *params,
    elib_traj_val_t start_pos,
    elib_traj_val_t dt,
    elib_traj_state_t *points,
    uint32_t num_points
);
```

### S 曲线 7 段式原理

S 曲线通过限制加加速度（jerk）使加速度平滑变化，减少机械冲击。

**7 个阶段**：

```
Phase 1 (jerk up):     jerk = +max_jerk   → acc 从 0 线性增加到 max_acc
Phase 2 (const acc):   jerk = 0           → acc 保持 max_acc（匀加速）
Phase 3 (jerk down):   jerk = -max_jerk   → acc 从 max_acc 线性减到 0（达到 max_vel）
Phase 4 (const vel):   jerk = 0, acc = 0  → vel = max_vel（匀速）
Phase 5 (jerk down):   jerk = -max_jerk   → acc 从 0 线性减到 -max_dec
Phase 6 (const dec):   jerk = 0           → acc 保持 -max_dec（匀减速）
Phase 7 (jerk up):     jerk = +max_jerk   → acc 从 -max_dec 线性增加到 0（vel 归零）
```

**每段的数学表达式**（以 Phase k 为例，t_k 为段起始时间，T_k = t_{k+1} - t_k 为段时长，τ = t - t_k 为段内时间）：

```
jerk = J_k（常数）
acc(τ) = a_k + J_k * τ
vel(τ) = v_k + a_k * τ + 0.5 * J_k * τ²
pos(τ) = p_k + v_k * τ + 0.5 * a_k * τ² + (1/6) * J_k * τ³
```

**退化策略**：

1. 如果距离不足以达到 max_vel → 缩短或去掉 Phase 2（匀加速段）
2. 如果仍然不够 → 缩短或去掉 Phase 4（匀速段）
3. 如果仍然不够 → 减小 jerk 段时长，使 7 段退化为更少段数
4. 极端情况下退化为三角形速度曲线

**预计算流程**（init 时）：

1. 计算加速到 max_vel 所需的各段时间（Phase 1+2+3）
2. 计算减速到 0 所需的各段时间（Phase 5+6+7）
3. 计算加速+减速总距离
4. 如果总距离 > target_pos 距离，执行退化
5. 计算匀速段时间（Phase 4）
6. 计算各段边界 t[0]..t[7]、v[0]..v[7]、a[0]..a[7]、p[0]..p[7]

## 参数校验规则

### 梯形

- `ctx` 不能为 NULL
- `params` 不能为 NULL（init 时）
- `state` / `status` 不能为 NULL（get 时）
- `dt` 必须 > 0（update 时）
- `max_vel > 0`
- `acc > 0`
- `dec > 0`
- `points` 不能为 NULL（generate 时）
- `num_points > 0`（generate 时）
- `start_pos` 不能等于 `target_pos`（generate 时，零距离直接返回 OK 但不生成轨迹）

### S 曲线

- 同梯形的通用校验
- `max_jerk > 0`

## 头文件规范

- 文件头单行注释：`/* elib_traj_defs.h - Trajectory Planner Common Definitions */`
- 头文件保护：`#ifndef ELIB_TRAJ_DEFS_H` / `#define ELIB_TRAJ_DEFS_H` / `#endif /* ELIB_TRAJ_DEFS_H */`
- `extern "C"` 包裹
- 命名：`elib_traj_*` 前缀，类型 `_t` 后缀，宏 `ELIB_TRAJ_*`

## 构建方式

无构建系统。用户将 `include/` 和 `src/` 加入自己的构建系统，编译需要的 `.c` 文件：

```bash
# 只用梯形规划
gcc -o app app.c src/elib_traj_trapezoid.c src/elib_traj_util.c -Iinclude -lm

# 只用 S 曲线规划
gcc -o app app.c src/elib_traj_scurve.c src/elib_traj_util.c -Iinclude -lm

# 两者都用
gcc -o app app.c src/elib_traj_trapezoid.c src/elib_traj_scurve.c src/elib_traj_util.c -Iinclude -lm
```

## 许可证

MIT License, Copyright (c) 2026 ChenYanan
