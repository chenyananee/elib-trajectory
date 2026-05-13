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

elib_traj_trapezoid_ctx_t ctx;
elib_traj_trapezoid_params_t params = {
    .max_vel = 1000.0f,
    .acc = 2000.0f,
    .dec = 2000.0f,
    .target_pos = 1000.0f,
};
elib_traj_trapezoid_init(&ctx, &params, 0.0f);

elib_traj_trapezoid_update(&ctx, 0.01f);
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

elib_traj_scurve_update(&ctx, 0.01f);
elib_traj_state_t state;
elib_traj_scurve_get_state(&ctx, &state);
```

### 3. 离线模式

```c
elib_traj_state_t trajectory[151];
elib_traj_trapezoid_generate(&params, 0.0f, 0.01f, trajectory, 151);
```

### 4. 配合 PID 闭环使用

```c
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
