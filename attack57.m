%% =================================================================================================
% 脚本说明
% =================================================================================================
% 该脚本通过模拟一系列电力系统场景来评估虚假数据注入攻击（FDIA）的影响。
% 主要流程包括：
% 1. 加载不同时段的负荷数据。
% 2. 在每个负荷场景下，运行最优潮流（OPF）以获取系统真实状态。
% 3. 模拟SCADA和PMU测量数据，并添加高斯噪声。
% 4. 对带噪声的测量进行状态估计，并进行坏数据检测（攻击前）。
% 5. 调用 `runAttackGeneration` 生成旨在使特定线路过载的隐蔽攻击。
% 6. 将攻击向量注入测量值，并再次运行状态估计和坏数据检测（攻击后）。
% 7. 验证攻击是否成功导致线路过载，以及是否被检测到。
% 8. 记录所有场景的结果并进行统计分析。
%
% 测量类型:
%   - SCADA: 节点电压幅值, 线路有功/无功潮流, 节点有功/无功注入。
%   - PMU:   节点电压相量, 支路电流相量。
%
% PMU安装位置 (case57): [4 10 15 20 24 29 32 37 41 48 54]'
% =================================================================================================

%% =================================================================================================
%  1. 初始化与配置 (Initialization & Configuration)
% =================================================================================================

clc;
clear;
rng('default'); % 设置随机种子以保证结果可复现
define_constants; % 加载MATPOWER常量 (如 PD, QD, VM, VA 等)

% --- 加载配置文件 ---
config = load_config();
VERBOSE_MODE = config.Simulation.VerboseMode; % 保持VERBOSE_MODE以便于旧代码调试

% --- 电网模型与数据加载 ---
mpc_base = loadcase(config.System.CaseName);
load_multipliers = calculate_rate(config.System.LoadDataFile); % 从文件加载负荷变化率

% --- 仿真与结果存储设置 ---

log_template = struct(...
    'scenario_index', 0, ...
    'attack_generation_successful', false, ...
    'attacked_lines_indices', [], ...
    'pre_attack_residual_norm', NaN, ...
    'post_attack_residual_norm', NaN, ...
    'is_attack_detected', false, ...
    'is_pre_attack_bad_data_detected', false, ...
    'overload_check_results', {{}} ...
);
simulation_log = repmat(log_template, config.Simulation.NumScenarios, 1);

%% =================================================================================================
%  2. 预计算与系统信息展示 (Pre-calculation & System Info Display)
% =================================================================================================
fprintf('正在初始化并计算系统参数...\n');


%% =================================================================================================
%  3. 主仿真循环 (Main Simulation Loop)
% =================================================================================================
for scenario_index = 1:config.Simulation.NumScenarios

% --- 3.1. 测量值生成 (OPF 计算在函数内完成) ---
[true_measurements, noisy_measurements, opf_results, system_params, pmu_config] = generateMeasurements(scenario_index, config, load_multipliers);

if ~opf_results.success
fprintf('警告: 场景 %d 的最优潮流 (OPF) 未收敛! 跳过此场景。\n', scenario_index);
simulation_log(scenario_index).attack_generation_successful = false;
simulation_log(scenario_index).pre_attack_residual_norm = NaN;
simulation_log(scenario_index).post_attack_residual_norm = NaN;
continue;
end


% 创建调整后的 mpc 结构体用于状态估计和攻击生成
mpc = mpc_base;
mpc.bus(:, PD) = mpc_base.bus(:, PD) * config.Simulation.LoadScaleFactor * load_multipliers(scenario_index);
mpc.bus(:, QD) = mpc_base.bus(:, QD) * config.Simulation.LoadScaleFactor * load_multipliers(scenario_index);

% --- 3.2. 攻击前状态估计与坏数据检测 ---
% 准备用于状态估计和攻击的测量数据 (不含末端潮流)
% 对于状态估计，我们使用系统拥有的所有测量
measurements_for_se = noisy_measurements;

[est_state_pre_attack, est_meas_pre_attack, ~, ~, pre_attack_residual, is_pre_attack_baddata] = runStateEstimation(mpc, measurements_for_se, pmu_config, config);
simulation_log(scenario_index).is_pre_attack_bad_data_detected = is_pre_attack_baddata;
simulation_log(scenario_index).scenario_index = scenario_index;

if ~isempty(pre_attack_residual)
simulation_log(scenario_index).pre_attack_residual_norm = norm(pre_attack_residual);
else
simulation_log(scenario_index).pre_attack_residual_norm = NaN;
fprintf('警告: 场景 %d 的攻击前状态估计未收敛! 跳过此场景。\n', scenario_index);
continue;
end

% --- 3.3. 攻击生成 ---
% 为了模拟拥有完美信息的攻击者，我们使用无噪声的真实测量值来生成攻击
% 为攻击生成准备输入:
% 1. full_true_measurements: 完整的真实测量，作为修改的蓝本
% 2. attacker_known_measurements: 攻击者已知的测量（不含pt, qt）
full_true_measurements = true_measurements;
attacker_known_measurements = true_measurements;
for k = 1:length(config.Attack.UnknownFields)
    field_name = config.Attack.UnknownFields{k};
    % 从SCADA测量中移除未知字段
    if isfield(attacker_known_measurements.scada, field_name)
        attacker_known_measurements.scada = rmfield(attacker_known_measurements.scada, field_name);
    end
    % 从PMU测量中移除未知字段
    if isfield(attacker_known_measurements, 'pmu') && isfield(attacker_known_measurements.pmu, field_name)
        attacker_known_measurements.pmu = rmfield(attacker_known_measurements.pmu, field_name);
    end
end

% 调用新的攻击生成函数
attack_results = runAttackGeneration(mpc, attacker_known_measurements, full_true_measurements, pmu_config, config);
simulation_log(scenario_index).attacked_lines_indices = attack_results.attacked_lines_indices;

% --- 3.4. 攻击后状态估计、分析与结果展示 ---
fprintf('==================================================\n');
fprintf(' 场景 %d / %d\n', scenario_index, config.Simulation.NumScenarios);
fprintf('==================================================\n');

fprintf('[+] 潮流计算 (OPF)               : 成功\n');
if isnan(simulation_log(scenario_index).pre_attack_residual_norm)
fprintf('[!] 状态估计 (攻击前)            : 未收敛\n');
else
fprintf('[+] 状态估计 (攻击前)            : 收敛, 残差范数 = %.6f\n', simulation_log(scenario_index).pre_attack_residual_norm);
if is_pre_attack_baddata, fprintf('[!] 坏数据检测 (攻击前)          : <font color="orange">已检测到异常</font>\n');
else, fprintf('[+] 坏数据检测 (攻击前)          : 未检测到异常\n'); end
end

if attack_results.success
simulation_log(scenario_index).attack_generation_successful = true;
fprintf('[+] 攻击向量生成                 : 成功 (目标线路: %s)\n', mat2str(attack_results.attacked_lines_indices));
if isfield(attack_results, 'objective'), fprintf('    - 优化目标函数值 (L1 Norm) : %.6f\n', attack_results.objective); end

[est_state_post_attack, est_meas_post_attack, ~, ~, post_attack_residual, is_attack_detected] = runStateEstimation(mpc, attack_results.attacked_measurements, pmu_config, config);
simulation_log(scenario_index).is_attack_detected = is_attack_detected;

if ~isempty(post_attack_residual)
post_attack_residual_norm = norm(post_attack_residual);
simulation_log(scenario_index).post_attack_residual_norm = post_attack_residual_norm;

fprintf('[+] 状态估计 (攻击后)            : 收敛, 残差范数 = %.6f\n', post_attack_residual_norm);
if is_attack_detected, fprintf('[!] 坏数据检测                   : <font color="red">检测到异常 (攻击被发现!)</font>\n');
else, fprintf('[+] 坏数据检测                   : <font color="green">未检测到异常 (攻击隐蔽)</font>\n'); end
else
simulation_log(scenario_index).post_attack_residual_norm = NaN;
fprintf('[!] 状态估计 (攻击后)            : 未收敛\n');
end


else
fprintf('[-] 攻击向量生成                 : 失败\n');
simulation_log(scenario_index).attack_generation_successful = false;
simulation_log(scenario_index).post_attack_residual_norm = NaN;
end

fprintf('\n');

end



