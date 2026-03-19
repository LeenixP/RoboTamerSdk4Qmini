# RoboTamerSdk4Qmini 调试报告

> 版本：v6.0 | 日期：2026-03-19
> 适用：Qmini 双足机器人（无手臂，10关节）
> **特殊条件**：腿部已更换为更重的3D打印材料

---

## 一、问题诊断

### 三个核心问题

| # | 问题 | 根因 | 解决方案 |
|---|------|------|----------|
| 1 | 切换时倾斜+左腿外八 | `reset()`无平滑过渡；`kp_yaw_ctrl`偏小 | 源码改平滑过渡；参数调大 |
| 2 | 左腿抬不起来 | 腿变重，`kp=0.5`不足，`tau_ff=0` | 参数增大kp |
| 3 | 行走不稳、"发疯" | 阻尼太低、积分饱和、无安全限制 | **源码加安全限制** |

---

## 二、参数调整方案（config.yaml）

### 推荐配置

```yaml
# 偏航保持（问题1）
kp_yaw_ctrl: 3.0

# PD 增益（问题2+3）
kp: [1.0, 0.7, 1.5, 1.0, 1.0, 1.0, 0.7, 1.5, 1.0, 1.0]
kd: [0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10]

# 动作限制（问题3）
act_inc_high: [3.5, 12.0]
act_inc_low: [0.5, -12.0]

# 速度范围（问题3）
vx_cmd_range: [-0.2, 0.3]
yr_cmd_range: [-0.5, 0.5]
```

### 参数调整步骤

| 步骤 | 修改 | 验证 |
|------|------|------|
| 1 | `kd: 0.05 → 0.10` | mode '2' 站立60秒，观察振动 |
| 2 | `kp[1,6]: 0.5 → 0.7` | mode '2' 站立30秒，观察下沉 |
| 3 | `kp_yaw_ctrl: 2.0 → 3.0` | mode '3' 行走，观察偏航 |
| 4 | `act_inc: 15 → 12` | mode '3' 行走，观察"发疯" |

---

## 三、源码安全限制（防止"发疯"）

### 已修改的文件

1. `include/user/rl_controller.h` - 添加安全参数和函数声明
2. `source/user/rl_controller.cpp` - 实现安全检查逻辑

### 安全限制层次

```
┌─────────────────────────────────────────────────────────────┐
│                    安全限制层次结构                           │
├─────────────────────────────────────────────────────────────┤
│ 第1层：网络输出检查                                          │
│   - 检测 NaN/Inf                                            │
│   - 检测异常大值（>50）                                       │
│   - 连续3次错误 → 紧急停止                                   │
├─────────────────────────────────────────────────────────────┤
│ 第2层：关节速度限制                                          │
│   - 最大速度：5.0 rad/s                                      │
│   - 超过则截断                                               │
├─────────────────────────────────────────────────────────────┤
│ 第3层：积分抗饱和                                            │
│   - 接近限位（±0.05 rad）时衰减增量                          │
│   - 阻止继续向限位方向积分                                    │
├─────────────────────────────────────────────────────────────┤
│ 第4层：位置误差检查                                          │
│   - 最大误差：0.5 rad                                        │
│   - 超过 → 紧急停止                                          │
├─────────────────────────────────────────────────────────────┤
│ 第5层：模式切换平滑过渡                                       │
│   - 300ms 渐变                                              │
│   - 避免突然跳变                                             │
├─────────────────────────────────────────────────────────────┤
│ 第6层：紧急停止                                              │
│   - 低增益保持当前位置                                        │
│   - 等待手动恢复                                             │
└─────────────────────────────────────────────────────────────┘
```

### 新增安全参数

```cpp
// 在 rl_controller.h 中添加
float MAX_JOINT_VELOCITY = 5.0;       // 最大关节速度 rad/s
float MAX_POSITION_ERROR = 0.5;       // 最大位置误差 rad
float MAX_NETWORK_OUTPUT = 50.0;      // 网络输出最大值
int MAX_CONSECUTIVE_ERRORS = 3;       // 最大连续错误次数
float TRANSITION_DURATION = 0.3f;     // 模式切换过渡时间

bool _emergency_stop = false;         // 紧急停止标志
int _consecutive_errors = 0;          // 连续错误计数
Vec10<float> _last_joint_act;         // 上一次位置（速度限制）
float _transition_progress = 0.0f;    // 过渡进度
bool _in_transition = false;          // 过渡标志
```

### 新增安全函数

```cpp
// 检查网络输出是否安全
bool RLController::check_network_output_safe(const Matrix<float, Dynamic, 1>& output) {
    for (int i = 0; i < output.size(); i++) {
        if (std::isnan(output(i)) || std::isinf(output(i))) {
            cerr << "[SAFETY] Network output NaN/Inf at index " << i << endl;
            return false;
        }
        if (std::abs(output(i)) > MAX_NETWORK_OUTPUT) {
            cerr << "[SAFETY] Network output too large: " << output(i) << endl;
            return false;
        }
    }
    return true;
}

// 检查位置误差
bool RLController::check_position_error_safe() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (std::abs(joint_act[i] - joint_pos[i]) > MAX_POSITION_ERROR) {
            cerr << "[SAFETY] Position error too large at joint " << i << endl;
            return false;
        }
    }
    return true;
}

// 限制关节速度
void RLController::limit_joint_velocity() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        float delta = joint_act[i] - _last_joint_act[i];
        float max_delta = MAX_JOINT_VELOCITY * _rl_time_step;
        if (std::abs(delta) > max_delta) {
            joint_act[i] = _last_joint_act[i] + std::copysign(max_delta, delta);
        }
    }
    _last_joint_act = joint_act;
}
```

### 修改后的控制流程

```cpp
void RLController::rl_control() {
    // 0. 紧急停止检查
    if (_emergency_stop) return;
    
    // 1. 平滑过渡处理
    if (_in_transition && _transition_progress < 1.0f) {
        _transition_progress += _rl_time_step / TRANSITION_DURATION;
        _transition_progress = std::min(_transition_progress, 1.0f);
    }
    
    // 2. 获取网络输出
    net_out = onnxInference.inference(...);
    
    // 3. 安全检查（第1层）
    if (!check_network_output_safe(net_out)) {
        _consecutive_errors++;
        if (_consecutive_errors >= 3) _emergency_stop = true;
        return;
    }
    
    // 4. 转换并衰减增量（第5层）
    action_increment = transform(net_out);
    if (_in_transition) action_increment *= _transition_progress;
    
    // 5. 执行控制（第3层抗饱和）
    joint_increment_control(action_increment);
    
    // 6. 速度限制（第2层）
    limit_joint_velocity();
    
    // 7. 位置误差检查（第4层）
    if (!check_position_error_safe()) {
        _emergency_stop = true;
    }
}
```

### 紧急停止行为

```cpp
void RLController::set_rl_joint_act2dds_motor_command(char mode) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (_emergency_stop) {
            // 紧急停止：保持当前位置，低增益
            motor_command_tmp.q_target[i] = joint_pos[i];
            motor_command_tmp.kp[i] = 0.5;
            motor_command_tmp.kd[i] = 0.1;
        } else {
            // 正常控制
            motor_command_tmp.q_target[i] = joint_act[i];
            motor_command_tmp.kp[i] = _kp[i];
            motor_command_tmp.kd[i] = _kd[i];
        }
    }
}
```

---

## 四、调试操作流程

### 步骤1：编译更新后的代码

```bash
cd /home/lipeng/tmp/RoboTamerSdk4Qmini
mkdir -p build && cd build
cmake .. && make -j4
```

### 步骤2：参数调试

```bash
cd /home/lipeng/tmp/RoboTamerSdk4Qmini/bin
# 备份原配置
cp config.yaml config.yaml.bak
# 修改 config.yaml（按上面的推荐配置）
./run_interface
```

### 步骤3：验证安全限制

1. **测试网络输出检查**：观察终端是否有 `[SAFETY]` 警告
2. **测试紧急停止**：如果出现异常，观察是否自动进入安全模式
3. **测试模式切换**：观察切换时是否平滑（300ms渐变）

---

## 五、安全日志说明

当触发安全机制时，终端会打印以下日志：

| 日志 | 含义 | 触发条件 |
|------|------|----------|
| `[SAFETY] Network output NaN/Inf` | 网络输出异常 | NaN或Inf值 |
| `[SAFETY] Network output too large` | 网络输出过大 | 绝对值>50 |
| `[SAFETY] Position error too large` | 位置误差过大 | 误差>0.5rad |
| `[SAFETY] Emergency stop triggered` | 紧急停止 | 连续3次错误或位置误差过大 |
| `[INFO] Transition completed` | 过渡完成 | 300ms后 |

---

## 六、关键参数速查

### 关节映射

| ID | 关节 | kp | kd | 备注 |
|----|------|-----|-----|------|
| 0 | L-HipRoll | 1.0 | 0.10 | |
| 1 | **L-HipPitch** | **0.7** | 0.10 | 抬腿关键 |
| 2 | L-HipYaw | 1.5 | 0.10 | |
| 3 | L-Knee | 1.0 | 0.10 | |
| 4 | L-Ankle | 1.0 | 0.10 | |
| 5 | R-HipRoll | 1.0 | 0.10 | |
| 6 | **R-HipPitch** | **0.7** | 0.10 | 抬腿关键 |
| 7 | R-HipYaw | 1.5 | 0.10 | |
| 8 | R-Knee | 1.0 | 0.10 | |
| 9 | R-Ankle | 1.0 | 0.10 | |

### 传动比

- 普通关节：6.33
- Hip Pitch (ID=1,6)：**18.99** (6.33 × 3.0)

### Startq（不要修改）

```cpp
{0.13, 0.11, 0.58, 0.55, 0.10, 0.55, 0.16, 0.47, 0.05, 0.65}
```

---

## 七、故障排除

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| 高频振荡（>5Hz）| kp过高 | 减小kp 20% |
| 关节过热（>60°C）| 持续高力矩 | 减小kp，检查机械 |
| 终端显示 `[SAFETY]` | 安全机制触发 | 检查具体原因，按 '2' 重新站立 |
| 切换后立即停止 | 过渡期间网络异常 | 检查策略文件是否正确 |
| 行走时突然停止 | 位置误差过大 | 检查关节是否有卡顿 |

---

## 八、恢复紧急停止

如果触发了紧急停止（`_emergency_stop = true`），需要：

1. 按 '2' 重新进入站立模式（会调用 `reset()` 重置状态）
2. 检查终端日志确认触发原因
3. 排除问题后再按 '3' 进入行走模式

---

*修改文件清单：*
- `include/user/rl_controller.h` - 已修改
- `source/user/rl_controller.cpp` - 已修改
- `bin/config.yaml` - 需手动修改

*所有修改前请备份原始文件。*