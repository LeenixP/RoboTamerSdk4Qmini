#include "user/rl_controller.h"
#include <algorithm>
#include <chrono>

void RLController::init() {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "q1");
    Ort::SessionOptions session_options;
    motion_session = new Ort::Session(env, "policy.onnx", session_options);
    _offset_joint_act.setZero();
    onnxInference.init(configParams.num_observations, configParams.num_actions, configParams.num_stacks);
    jointIndex2Sim << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
    base_rpy.setZero();
    base_vel.setZero();
    joint_pos.setZero();
    joint_vel.setZero();
    joint_tau.setZero();
    joint_acc.setZero();
    base_acc.setZero();
    base_quat << 1, 0, 0, 0;
    base_rpy_rate.setZero();
    target_command.setZero();
    joint_pos_error.setZero();
    pm_f.setConstant(0.5f);
    _pm_phase << 0, 0;
    pm_phase_sin_cos.setZero();
    action_increment.resize(onnxInference.output_dim);
    action_increment.setZero();

    for (int i = 0; i < NUM_JOINTS; ++i) {
        act_pos_low[i] = configParams.act_pos_low.at(i);
        act_pos_high[i] = configParams.act_pos_high.at(i);
        _ref_joint_act[i] = configParams.ref_joint_act.at(i);
        _kp[i] = configParams.kp.at(i);
        _kd[i] = configParams.kd.at(i);
        _kp_soft[i] = configParams.kp_soft.at(i);
        _kd_soft[i] = configParams.kd_soft.at(i);
    }

    MAX_JOINT_VELOCITY = configParams.max_joint_velocity;
    MAX_POSITION_ERROR = configParams.max_position_error;
    MAX_NETWORK_OUTPUT = configParams.max_network_output;
    MAX_CONSEC_ERRORS = configParams.max_consecutive_errors;
    TRANSITION_DURATION = configParams.transition_duration;

    joint_act = _ref_joint_act;
    _last_joint_act = _ref_joint_act;
    observation.resize(onnxInference.input_dim * onnxInference.stack_dim);
    observation.setZero();
    obs_stack.resize(onnxInference.stack_dim);
    for (int i(0); i < onnxInference.stack_dim; i++)obs_stack.at(i).resize(onnxInference.input_dim);
    for (int i(0); i < onnxInference.stack_dim; i++)obs_stack.at(i).setZero(onnxInference.input_dim);

}

void RLController::reset(bool is_test_local) {
    pm_f.setConstant(0.5f);
    _pm_phase << 0, 0;
    target_command.setZero();
    _is_first_run = true;
    counter_rl = 0;
    _emergency_stop     = false;
    _consecutive_errors = 0;
    _transition_progress = 0.0f;
    _in_transition       = true;
    
    if (is_test_local) {
        init_joint_act = joint_act;
        joint_pos = joint_act;
    } else {
        convert_dds_state2rl_state();
        
        // ========== 安全过渡逻辑 ==========
        // 1. 将joint_pos限制在安全范围内
        Vec10<float> safe_joint_pos = joint_pos.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
        
        // 2. 检测是否接近限位边缘（距离限位小于0.15 rad = 8.6°）
        bool near_limit = false;
        int near_limit_joint = -1;
        float near_limit_distance = 0.15f; // 0.15 rad ≈ 8.6°
        
        for (int i = 0; i < NUM_JOINTS; i++) {
            float dist_to_low = joint_pos[i] - act_pos_low[i];
            float dist_to_high = act_pos_high[i] - joint_pos[i];
            if (dist_to_low < near_limit_distance || dist_to_high < near_limit_distance) {
                near_limit = true;
                near_limit_joint = i;
                break;
            }
        }
        
        // 3. 如果接近限位，调整 safe_joint_pos 使其远离限位
        //    然后作为平滑过渡的起点（不再直接跳到 ref_joint_act！）
        if (near_limit) {
            // 将关节位置向安全方向稍微移动，避免限位碰撞
            for (int i = 0; i < NUM_JOINTS; i++) {
                float dist_to_low = safe_joint_pos[i] - act_pos_low[i];
                float dist_to_high = act_pos_high[i] - safe_joint_pos[i];
                
                // 如果接近下限位，向中间方向移动
                if (dist_to_low < near_limit_distance) {
                    safe_joint_pos[i] = act_pos_low[i] + near_limit_distance;
                    log_warn("RESET", "Joint " + std::to_string(i) + " near low limit, adjusted +" + 
                             std::to_string(near_limit_distance) + " rad");
                }
                // 如果接近上限位，向中间方向移动
                else if (dist_to_high < near_limit_distance) {
                    safe_joint_pos[i] = act_pos_high[i] - near_limit_distance;
                    log_warn("RESET", "Joint " + std::to_string(i) + " near high limit, adjusted -" + 
                             std::to_string(near_limit_distance) + " rad");
                }
            }
        }
        
        // 【重要】始终使用 safe_joint_pos 作为平滑过渡的起点
        // 让 stand_control() 负责 3 秒的平滑过渡，而不是瞬间跳变
        joint_act = safe_joint_pos;
        init_joint_act = safe_joint_pos;
        
        _last_joint_act = joint_act;
        _record_yaw = base_rpy[2];
        
        log_info("RESET", "Controller reset complete");
        log_info("RESET", "joint_pos: " + format_vec10(joint_pos));
        log_info("RESET", "joint_act: " + format_vec10(joint_act));
        log_info("RESET", "base_rpy: " + format_vec3(base_rpy) + " (deg: " + 
                 std::to_string(base_rpy[0] * 180 / M_PI).substr(0, 6) + ", " +
                 std::to_string(base_rpy[1] * 180 / M_PI).substr(0, 6) + ", " +
                 std::to_string(base_rpy[2] * 180 / M_PI).substr(0, 6) + ")");
        log_info("RESET", "Safety params: MAX_POS_ERR=" + std::to_string(MAX_POSITION_ERROR) + 
                 " MAX_JOINT_VEL=" + std::to_string(MAX_JOINT_VELOCITY));
        if (near_limit) {
            log_info("RESET", "SAFE_MODE: Started from ref_joint_act to avoid limit collision");
        }
    }
    auto o = get_observation();
}


void RLController::rl_control() {
    if (_emergency_stop) return;

    counter_rl++;
    _rl_time_step = get_true_loop_period();

    if (_in_transition) {
        _transition_progress += _rl_time_step / TRANSITION_DURATION;
        if (_transition_progress >= 1.0f) {
            _transition_progress = 1.0f;
            _in_transition = false;
            log_info("RL", "Transition completed, entering full RL control");
        }
    }

    Matrix<float, Dynamic, 1> net_out;
    net_out = onnxInference.inference(motion_session, get_observation());

    if (!check_network_output_safe(net_out)) {
        _consecutive_errors++;
        log_error("SAFETY", "Network output unsafe, consecutive_errors=" + std::to_string(_consecutive_errors));
        if (_consecutive_errors >= MAX_CONSEC_ERRORS) {
            _emergency_stop = true;
            log_error("SAFETY", "EMERGENCY STOP triggered: too many consecutive errors");
        }
        return;
    }
    _consecutive_errors = 0;

    action_increment = transform(net_out);
    if (_in_transition)
        action_increment.segment(NUM_LEGS, NUM_ACTUAT_JOINTS) *= _transition_progress;

    joint_increment_control(action_increment);

    limit_joint_velocity();

    if (!check_position_error_safe()) {
        _emergency_stop = true;
        log_error("SAFETY", "EMERGENCY STOP triggered: position error exceeded " + std::to_string(MAX_POSITION_ERROR) + " rad");
        log_error("SAFETY", "joint_act: " + format_vec10(joint_act));
        log_error("SAFETY", "joint_pos: " + format_vec10(joint_pos));
    }
    
    // 每500次循环记录一次状态摘要
    if (counter_rl % 500 == 0) {
        float max_err = 0;
        int max_err_idx = 0;
        for (int i = 0; i < NUM_JOINTS; i++) {
            float err = std::abs(joint_act[i] - joint_pos[i]);
            if (err > max_err) { max_err = err; max_err_idx = i; }
        }
        log_info("RL", "Status: counter=" + std::to_string(counter_rl) + 
                 " dt=" + std::to_string(_rl_time_step * 1000).substr(0, 5) + "ms" +
                 " max_pos_err=" + std::to_string(max_err).substr(0, 5) + "rad@joint" + std::to_string(max_err_idx) +
                 " cmd=" + std::to_string(target_command[0]).substr(0, 5) + "," + std::to_string(target_command[1]).substr(0, 5));
    }
}

void RLController::joint_increment_control(Matrix<float, Dynamic, 1> increment) {
    pm_f = increment.segment(0, NUM_LEGS);
    compute_pm_phase(pm_f);
    joint_act.segment(0, NUM_ACTUAT_JOINTS) += increment.segment(NUM_LEGS, NUM_ACTUAT_JOINTS) * _rl_time_step;
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
    // cout << "joint_act: " << joint_act.transpose() << endl;
    // exit(1);
}


Matrix<float, Dynamic, 1> RLController::get_observation() {
    Matrix<float, Dynamic, 1> obs;
    Vec2<float> con_1;
    con_1.setOnes();
    obs.resize(onnxInference.input_dim);
    obs.setZero();
    pthread_mutex_lock(&_rl_state_mutex);
    joint_pos_error = joint_act - joint_pos;
    for (int i(0); i < NUM_LEGS; i++) {
        pm_phase_sin_cos(i) = sin(_pm_phase[i]);
        pm_phase_sin_cos(NUM_LEGS + i) = cos(_pm_phase[i]);
    }
    joystick_command_process();
    if (sqrt(pow(target_command(0), 2) + pow(target_command(1), 2)) < 0.15)
        static_flag = 0.f;
    else
        static_flag = 1.f;
    obs << target_command,
            base_rpy.segment(0, 2),
            base_rpy_rate * 0.5,
            joint_pos.segment(0, NUM_ACTUAT_JOINTS) - _ref_joint_act,
            joint_vel.segment(0, NUM_ACTUAT_JOINTS) * 0.1f,
            joint_pos_error.segment(0, NUM_ACTUAT_JOINTS),
            pm_phase_sin_cos * static_flag,
            (pm_f * 0.3 - con_1) * static_flag;
    obs = obs.cwiseMax(-3.).cwiseMin(3.);


    pthread_mutex_unlock(&_rl_state_mutex);
    if (int(observation.size()) != onnxInference.input_dim * onnxInference.stack_dim) {
        cout << "The dimension of the input size observation is error!!!" << endl;
        cout << "True state size:" << observation.size() << "Policy input size:" << onnxInference.input_dim * onnxInference.stack_dim << endl;
        exit(1);
    }

    if (_is_first_run) {
        for (int i(0); i < onnxInference.stack_dim; i++) {
            obs_stack.erase(obs_stack.begin());
            obs_stack.push_back(obs);
        }
        _is_first_run = false;
        cout << endl << "Reset observation history: Done!" << endl;
    } else {
        obs_stack.erase(obs_stack.begin());
        obs_stack.push_back(obs);
    }
    for (int i(0); i < onnxInference.stack_dim; i++) {
        for (int j(0); j < onnxInference.input_dim; j++) {
            observation[onnxInference.input_dim * i + j] = obs_stack.at(i)[j];
        }
    }
    return observation;
}


void RLController::joystick_command_process() {
    float vx_cmd = 0,  yr_cmd = 0;
    auto yr_max = configParams.yr_cmd_range.at(1);
    auto vx_min = configParams.vx_cmd_range.at(0);
    auto vx_max = configParams.vx_cmd_range.at(1);
    if (task_mode == 3 or task_mode == 4) {
        ///stand
        vx_cmd = -vx_max * jsreader->Axis[1];
        yr_cmd = -yr_max * jsreader->Axis[2];

        if (fabs(yr_cmd) > 0.1 or configParams.kp_yaw_ctrl < 1e-2 or static_flag < 0.1) {
            _record_yaw = base_rpy[2];//todo
        } else {
            yr_cmd = configParams.kp_yaw_ctrl * smallest_signed_angle_between(base_rpy[2], _record_yaw);
        }

        yr_cmd = std::clamp(yr_cmd, -yr_max, yr_max);
        vx_cmd = std::clamp(vx_cmd, vx_min, vx_max);
    }
    target_command << vx_cmd, yr_cmd;
}

void RLController::set_rl_joint_act2dds_motor_command(char mode) {
    MotorCommand motor_command_tmp;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (_emergency_stop) {
            // 紧急停止：保持当前位置，低增益
            motor_command_tmp.q_target[i] = joint_pos[jointIndex2Sim[i]];
            motor_command_tmp.kp[i] = 0.5;
            motor_command_tmp.kd[i] = 0.1;
        } else if (mode == 'q') {
            motor_command_tmp.q_target[i] = joint_act[jointIndex2Sim[i]];
            motor_command_tmp.kp[i] = 0.;
            motor_command_tmp.kd[i] = 0.;
        } else if (mode == '1') {
            motor_command_tmp.q_target[i] = joint_act[jointIndex2Sim[i]];
            motor_command_tmp.kp[i] = _kp_soft[jointIndex2Sim[i]];
            motor_command_tmp.kd[i] = _kd_soft[jointIndex2Sim[i]];
        } else {
            motor_command_tmp.q_target[i] = joint_act[jointIndex2Sim[i]];
            motor_command_tmp.kp[i] = _kp[jointIndex2Sim[i]];
            motor_command_tmp.kd[i] = _kd[jointIndex2Sim[i]];
        }
        motor_command_tmp.tau_ff[i] = 0.;
        motor_command_tmp.dq_target[i] = 0.;
    }
    dds_motor_command->SetData(motor_command_tmp);
}

void RLController::convert_dds_state2rl_state() {
    Vec3<float> trans_axis(-1., 1, -1);
    if (dds_motor_state->GetData()) {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joint_pos[jointIndex2Sim[i]] = exp_filter(joint_pos[jointIndex2Sim[i]], dds_motor_state->GetData()->q[i], 0.2);
            joint_vel[jointIndex2Sim[i]] = exp_filter(joint_vel[jointIndex2Sim[i]], dds_motor_state->GetData()->dq[i], 0.1);
            joint_tau[jointIndex2Sim[i]] = dds_motor_state->GetData()->tau_est[i];
            joint_acc[jointIndex2Sim[i]] = dds_motor_state->GetData()->ddq[i];
        }
    }
    if (dds_base_state->GetData()) {
        for (int i(0); i < 3; i++) {
            base_rpy(i) = exp_filter(base_rpy(i), fmod(dds_base_state->GetData()->rpy.at(i) * trans_axis(i), 2 * M_PI), 0.2);
            base_rpy_rate(i) = exp_filter(base_rpy_rate(i), dds_base_state->GetData()->omega.at(i) * trans_axis(i), 0.1);
            base_acc(i) = exp_filter(base_acc(i), dds_base_state->GetData()->acc.at(i) * trans_axis(i), 0.1);
        }
    }
    counter_print++;
}


void RLController::compute_pm_phase(Vec2<float> f) {
    for (int leg(0); leg < NUM_LEGS; leg++) {
        _pm_phase[leg] += 2. * M_PI * f[leg] * _rl_time_step;
        _pm_phase[leg] = fmod(_pm_phase[leg], 2 * M_PI);
    }
}

Matrix<float, Dynamic, -1> RLController::transform(Matrix<float, Dynamic, -1> data) {
    auto net = (data.array() + 1.) / 2.;
    int ii = 0;
    for (int i(0); i < onnxInference.output_dim; i++) {
        if (i < NUM_LEGS)
            ii = 0;
        else if (i < NUM_LEGS + NUM_ACTUAT_JOINTS + 1)
            ii = 1;
        else
            ii = 2;
        action_increment(i) = net(i) * (configParams.act_inc_high[ii] - configParams.act_inc_low[ii]) + configParams.act_inc_low[ii];
    }
    return action_increment;
}


void RLController::smooth_joint_action(float ratio, const Vec10<float> &end_joint_act) {
    joint_act = (1 - ratio) * init_joint_act + ratio * end_joint_act;
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}

float RLController::exp_filter(float history, float present, float weight) {
    auto result = history * weight + present * (1. - weight);
    return result;
}


void RLController::sin_control(float amplitude, float f, float motion_time) {
    Vec10<float> sin_joint_act;
    sin_joint_act.setConstant(amplitude * sin(2.f * M_PI * f * motion_time));
    if (configParams.sin_joint_idx == -1)
        joint_act.segment(0, NUM_ACTUAT_JOINTS) = init_joint_act.segment(0, NUM_ACTUAT_JOINTS) + sin_joint_act;
    else
        joint_act[configParams.sin_joint_idx] = init_joint_act[configParams.sin_joint_idx] + sin_joint_act[configParams.sin_joint_idx];
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}


float RLController::get_true_loop_period() {
    static struct timeval last_time;
    static struct timeval now_time;
    static bool first_get_time = true;
    if (first_get_time) {
        gettimeofday(&last_time, nullptr);
        first_get_time = false;
    }
    gettimeofday(&now_time, nullptr);
    auto d_time = (float) (now_time.tv_sec - last_time.tv_sec) +
                  (float) (now_time.tv_usec - last_time.tv_usec) / 1000000;
    last_time = now_time;
    if (fabs(d_time - _rl_time_step) * 1000. > 2.)
        cout << "True period: " << d_time * 1000. << " ms" << endl;
    return d_time;
}

void RLController::stand_control(float ratio) {
    smooth_joint_action(ratio, _ref_joint_act);
}

void RLController::sim_gait_control() {
    static int data_index = 0;
    if (data_index <= sim_gait_data.size() - 2)
        data_index++;
    for (int i(0); i < NUM_ACTUAT_JOINTS; i++) {
        joint_act(i) = sim_gait_data.at(data_index).at(i);
    }
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}

Vec3<float> RLController::convert_world_frame_to_base_frame(const Vec3<float> &world_vec, const Vec3<float> &rpy) {
    return ori::rpy_to_rotMat(rpy) * world_vec;
}

Vec3<float> RLController::quat_rotate_inverse(Vec4<float> q, Vec3<float> v) {
    Vec3<float> a, b, c;
    // q={w,x,y,z}
    // q << 0.99981624, -0.013256095, 0.012793032, -0.005295367; //todo: in isaac gym: x,y,z,w ;here IMU(q): w,x,y,z !!
    // v << -0.13947208, -0.08728597, 0.19939381;
    // result: lin_v(-0.14353749,-0.09399233,0.19336908)
    float q_w = q[0];
    Vec3<float> q_vec(q[1], q[2], q[3]);
    a = v * (2.0 * q_w * q_w - 1.0);
    b = q_vec.cross(v) * q_w * 2.0;
    c = q_vec * q_vec.transpose() * v * 2.0;
    // cout << "quat_rotate_inverse: " << (a - b + c).transpose() << endl << endl;
    return a - b + c;
}

/*!
 * Take the product of two quaternions
 */
Vec4<float> RLController::quat_product(Vec4<float> &q1, Vec4<float> &q2) {
    float r1 = q1[0];
    float r2 = q2[0];

    Vec3<float> v1(q1[1], q1[2], q1[3]);
    Vec3<float> v2(q2[1], q2[2], q2[3]);

    float r = r1 * r2 - v1.dot(v2);
    Vec3<float> v = r1 * v2 + r2 * v1 + v1.cross(v2);
    Vec4<float> q(r, v[0], v[1], v[2]);
    return q;
}

float RLController::smallest_signed_angle_between(float alpha, float beta) {
    auto a = beta - alpha;
    a += (a > M_PI) ? -2 * M_PI : (a < -M_PI) ? 2 * M_PI : 0.;
    return a;
}

Vec4<float> RLController::rpy_to_quat(const Vec3<float> &rpy) {
    auto R = ori::rpy_to_rotMat(rpy);
    auto q = ori::rotMat_to_quat(R);
    return q;
}

bool RLController::check_network_output_safe(const Matrix<float, Dynamic, 1> &output) {
    for (int i = 0; i < output.size(); i++) {
        if (std::isnan(output(i)) || std::isinf(output(i))) {
            cerr << "[SAFETY] Network output NaN/Inf at index " << i << endl;
            return false;
        }
        if (std::abs(output(i)) > MAX_NETWORK_OUTPUT) {
            cerr << "[SAFETY] Network output too large: " << output(i) << " at index " << i << endl;
            return false;
        }
    }
    return true;
}

bool RLController::check_position_error_safe() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        float err = std::abs(joint_act[i] - joint_pos[i]);
        if (err > MAX_POSITION_ERROR) {
            cerr << "[SAFETY] Position error too large at joint " << i
                 << ": " << err << " rad" << endl;
            return false;
        }
    }
    return true;
}

void RLController::limit_joint_velocity() {
    float max_delta = MAX_JOINT_VELOCITY * _rl_time_step;
    for (int i = 0; i < NUM_JOINTS; i++) {
        float delta = joint_act[i] - _last_joint_act[i];
        if (std::abs(delta) > max_delta)
            joint_act[i] = _last_joint_act[i] + std::copysign(max_delta, delta);
    }
    _last_joint_act = joint_act;
}

void RLController::begin_transition() {
    _transition_progress = 0.0f;
    _in_transition = true;
}

Vec4<float> RLController::quat_mul(Vec4<float> a, Vec4<float> b) {
    float x1 = a[1];
    float y1 = a[2];
    float z1 = a[3];
    float w1 = a[0];

    float x2 = b[1];
    float y2 = b[2];
    float z2 = b[3];
    float w2 = b[0];

    float ww = (z1 + x1) * (x2 + y2);
    float yy = (w1 - y1) * (w2 + z2);
    float zz = (w1 + y1) * (w2 - z2);
    float xx = ww + yy + zz;
    float qq = 0.5 * (xx + (z1 - x1) * (x2 - y2));

    float w = qq - ww + (z1 - y1) * (y2 - z2);
    float x = qq - xx + (x1 + w1) * (x2 + w2);
    float y = qq - yy + (w1 - x1) * (y2 + z2);
    float z = qq - zz + (z1 + y1) * (w2 - x2);

    Vec4<float> quat(w, x, y, z);

    return quat;
}

std::string RLController::get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&timer);
    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y-%m-%d %H:%M:%S") << "." 
        << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

std::string RLController::format_vec10(const Vec10<float>& v, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[";
    for (int i = 0; i < 10; i++) {
        if (i > 0) oss << ", ";
        oss << v[i];
    }
    oss << "]";
    return oss.str();
}

std::string RLController::format_vec3(const Vec3<float>& v, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    return oss.str();
}

void RLController::log_debug(const char* tag, const std::string& msg) {
    std::cout << "[" << get_timestamp() << "] [DEBUG] [" << tag << "] " << msg << std::endl;
}

void RLController::log_info(const char* tag, const std::string& msg) {
    std::cout << "[" << get_timestamp() << "] [INFO] [" << tag << "] " << msg << std::endl;
}

void RLController::log_warn(const char* tag, const std::string& msg) {
    std::cerr << "[" << get_timestamp() << "] [WARN] [" << tag << "] " << msg << std::endl;
}

void RLController::log_error(const char* tag, const std::string& msg) {
    std::cerr << "[" << get_timestamp() << "] [ERROR] [" << tag << "] " << msg << std::endl;
}