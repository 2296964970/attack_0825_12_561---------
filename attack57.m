%% ================================================================================================
% 攻击仿真主脚本（重构版：全量测量模型 + 选择切片 + 新SE/BDD + 新攻击接口）
%% ================================================================================================

clc; clear;
rng('default');                 % 固定随机种子，保证可复现
define_constants;               % MATPOWER 常量 (PD, QD, VM, VA 等)

% 加载配置
config = load_config();

% 电网基础模型与负荷倍率
mpc_base = loadcase(config.System.CaseName);
load_multipliers = calculate_rate(config.System.LoadDataFile);

% 构建 PMU 布局并编译“全量测量模型”（仅依赖网络拓扑，可循环复用）
pmu_bus_locations = config.Grid.PmuBusLocations(:);
from_bus_indices = mpc_base.branch(:, F_BUS);
to_bus_indices   = mpc_base.branch(:, T_BUS);
pmu_from_branch_indices = find(ismember(from_bus_indices, pmu_bus_locations));
pmu_to_branch_indices   = find(ismember(to_bus_indices,   pmu_bus_locations));
layout = struct('PmuBusLocations', pmu_bus_locations, ...
                'PmuFromBranchIdx', pmu_from_branch_indices, ...
                'PmuToBranchIdx',   pmu_to_branch_indices);
opts_full = struct('slack_bus_id', config.Grid.SlackBusId);
[fullModel, registry] = buildFullMeasurementModel(mpc_base, layout, config.Noise, opts_full);

% 量测选择：默认全覆盖（可扩展为 DSL 的 include/exclude）
[sel, S, ~] = makeSelection(registry, struct());
selectedModel = sliceModel(fullModel, sel);

% 仿真日志模板
log_template = struct(...
    'scenario_index', 0, ...
    'pre_attack_residual_norm', NaN, ...
    'is_pre_attack_bad_data_detected', false, ...
    'post_attack_residual_norm', NaN, ...
    'is_attack_detected', false, ...
    'attack_success', false, ...
    'attacked_lines_indices', [] ...
);
simulation_log = repmat(log_template, config.Simulation.NumScenarios, 1);

fprintf('初始化完成（全量模型已构建），开始进入场景循环...\n');

for scenario_index = 1:config.Simulation.NumScenarios
    % 按场景调整负荷并执行 OPF
    mpc = mpc_base;
    scale = config.Simulation.LoadScaleFactor * load_multipliers(scenario_index);
    mpc.bus(:, PD) = mpc_base.bus(:, PD) * scale;
    mpc.bus(:, QD) = mpc_base.bus(:, QD) * scale;

    mpopt = mpoption('verbose', 0, 'out.all', 0);
    try
        opf_results = runopf(mpc, mpopt);
    catch ME
        warning('场景 %d 的 OPF 执行异常: %s', scenario_index, ME.message);
        opf_results = struct('success', false);
    end

    if ~isfield(opf_results, 'success') || ~opf_results.success
        fprintf('警告: 场景 %d 的 OPF 未收敛，跳过。\n', scenario_index);
        simulation_log(scenario_index).scenario_index = scenario_index;
        continue;
    end

    % 从 OPF 结果构造真实状态 x_true（直角坐标，去除 slack 的 f）
    vm = opf_results.bus(:, VM); va = opf_results.bus(:, VA) * pi/180;
    e_true = vm .* cos(va); f_true = vm .* sin(va);
    f_idx = setdiff((1:size(mpc.bus,1))', config.Grid.SlackBusId);
    x_true = [e_true; f_true(f_idx)];

    % 基于全量模型生成含噪观测向量（全覆盖），再按选择切片
    [y_full, ~] = generateMeasurementsFromState(fullModel, x_true);
    y = S * y_full;

    % 运行基线 WLS + BDD
    [state_base, stats_base] = runStateEstimation(selectedModel, y, config);

    simulation_log(scenario_index).scenario_index = scenario_index;
    simulation_log(scenario_index).pre_attack_residual_norm = norm(stats_base.residual);
    simulation_log(scenario_index).is_pre_attack_bad_data_detected = stats_base.detectionFlag;

    % 生成攻击并构造攻击后测量向量
    attack = runAttackGeneration(selectedModel, y, mpc, opf_results, config);
    simulation_log(scenario_index).attacked_lines_indices = attack.attacked_lines_indices;
    simulation_log(scenario_index).attack_success = attack.success;

    % 攻击后 SE/BDD
    if attack.success
        [state_att, stats_att] = runStateEstimation(selectedModel, attack.y_att, config);
        simulation_log(scenario_index).post_attack_residual_norm = norm(stats_att.residual);
        simulation_log(scenario_index).is_attack_detected = stats_att.detectionFlag;
    end

    % 输出摘要
    fprintf('==================================================\n');
    fprintf(' 场景 %d / %d\n', scenario_index, config.Simulation.NumScenarios);
    fprintf('==================================================\n');
    fprintf('[+] 潮流计算 (OPF)               : 成功\n');
    fprintf('[+] 状态估计 (基线)              : %s, 残差范数 = %.6f\n', ...
        ternary(stats_base.converged,'收敛','未收敛'), simulation_log(scenario_index).pre_attack_residual_norm);
    fprintf('[%c] 坏数据检测 (基线)           : %s\n', ...
        ternary(stats_base.detectionFlag,'!','+'), ternary(stats_base.detectionFlag,'检测到异常','未检测到异常'));
    if attack.success
        fprintf('[+] 攻击向量生成                 : 成功 (目标线路: %s)\n', mat2str(attack.attacked_lines_indices));
        fprintf('[+] 状态估计 (攻击后)            : %s, 残差范数 = %.6f\n', ...
            ternary(stats_att.converged,'收敛','未收敛'), simulation_log(scenario_index).post_attack_residual_norm);
        fprintf('[%c] 坏数据检测 (攻击后)         : %s\n', ...
            ternary(stats_att.detectionFlag,'!','+'), ternary(stats_att.detectionFlag,'检测到异常','未检测到异常'));
    else
        fprintf('[-] 攻击向量生成                 : 失败\n');
    end
    fprintf('\n');
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
