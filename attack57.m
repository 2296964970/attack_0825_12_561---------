%% ================================================================================================
% 攻击仿真主脚本
% ================================================================================================
% 流程：
% 1) 读取负荷曲线并按场景缩放负荷；
% 2) 运行 OPF 获取真实状态；生成真实与带噪声的 SCADA/PMU 测量；
% 3) 攻击前执行 WLS 状态估计与坏数据检测；
% 4) 基于攻击者知识生成隐蔽攻击向量，并注入构造攻击后测量；
% 5) 攻击后再次执行状态估计与坏数据检测；
% 6) 打印结果并汇总日志。

clc; clear;
rng('default');                 % 固定随机种子，保证可复现
define_constants;               % MATPOWER 常量 (PD, QD, VM, VA 等)

% 加载配置
config = load_config();
VERBOSE_MODE = config.Simulation.VerboseMode;

% 电网基础模型与负荷倍率
mpc_base = loadcase(config.System.CaseName);
load_multipliers = calculate_rate(config.System.LoadDataFile);

% 检查并创建绘图输出目录
if ~isfolder(config.Simulation.PlotOutputDir)
    mkdir(config.Simulation.PlotOutputDir);
end

% 仿真日志模板
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

fprintf('初始化完成，开始进入场景循环...\n');

for scenario_index = 1:config.Simulation.NumScenarios

    % --- 3.1 生成测量 (内部执行 OPF) ---
    [true_measurements, noisy_measurements, opf_results, system_params, pmu_config] = ...
        generateMeasurements(scenario_index, config, load_multipliers);

    if ~isfield(opf_results, 'success') || ~opf_results.success
        fprintf('警告: 场景 %d 的 OPF 未收敛，跳过。\n', scenario_index);
        simulation_log(scenario_index).attack_generation_successful = false;
        simulation_log(scenario_index).pre_attack_residual_norm = NaN;
        simulation_log(scenario_index).post_attack_residual_norm = NaN;
        continue;
    end

    % 按场景调整 mpc (仅负荷)，用于 SE 与攻击生成
    mpc = mpc_base;
    scale = config.Simulation.LoadScaleFactor * load_multipliers(scenario_index);
    mpc.bus(:, PD) = mpc_base.bus(:, PD) * scale;
    mpc.bus(:, QD) = mpc_base.bus(:, QD) * scale;

    % --- 3.2 攻击前状态估计与坏数据检测 ---
    measurements_for_se = noisy_measurements; % 使用系统拥有的全部测量
    [pre_attack_state, ~, ~, ~, pre_attack_residual, is_pre_attack_baddata] = ...
        runStateEstimation(mpc, measurements_for_se, pmu_config, config);

    simulation_log(scenario_index).is_pre_attack_bad_data_detected = is_pre_attack_baddata;
    simulation_log(scenario_index).scenario_index = scenario_index;

    if ~isempty(pre_attack_residual)
        simulation_log(scenario_index).pre_attack_residual_norm = norm(pre_attack_residual);
    else
        simulation_log(scenario_index).pre_attack_residual_norm = NaN;
        fprintf('警告: 场景 %d 的攻击前状态估计未收敛，跳过。\n', scenario_index);
        continue;
    end

    % --- 3.3 生成攻击向量 ---
    % 将攻击者未知的测量字段从已知集中剔除
    full_true_measurements = noisy_measurements;
    attacker_known_measurements =true_measurements;
    
    % 检查是否有未知字段需要移除
    if ~isempty(config.Attack.UnknownFields)
        for k = 1:length(config.Attack.UnknownFields)
            field_name = config.Attack.UnknownFields{k};
            if isfield(attacker_known_measurements, 'scada') && ...
               isfield(attacker_known_measurements.scada, field_name)
                attacker_known_measurements.scada = rmfield(attacker_known_measurements.scada, field_name);
            end
            if isfield(attacker_known_measurements, 'pmu') && ...
               isfield(attacker_known_measurements.pmu, field_name)
                attacker_known_measurements.pmu = rmfield(attacker_known_measurements.pmu, field_name);
            end
        end
    end

    attack_results = runAttackGeneration(mpc, attacker_known_measurements, full_true_measurements, pmu_config, config);
    simulation_log(scenario_index).attacked_lines_indices = attack_results.attacked_lines_indices;

    % --- 3.4 攻击后状态估计与结果打印 ---
    fprintf('==================================================\n');
    fprintf(' 场景 %d / %d\n', scenario_index, config.Simulation.NumScenarios);
    fprintf('==================================================\n');

    fprintf('[+] 潮流计算 (OPF)               : 成功\n');
    if isnan(simulation_log(scenario_index).pre_attack_residual_norm)
        fprintf('[!] 状态估计 (攻击前)           : 未收敛\n');
    else
        fprintf('[+] 状态估计 (攻击前)           : 收敛, 残差范数 = %.6f\n', ...
            simulation_log(scenario_index).pre_attack_residual_norm);
        if is_pre_attack_baddata
            fprintf('[!] 坏数据检测 (攻击前)         : 已检测到异常\n');
        else
            fprintf('[+] 坏数据检测 (攻击前)         : 未检测到异常\n');
        end
    end

    if isfield(attack_results, 'success') && attack_results.success
        simulation_log(scenario_index).attack_generation_successful = true;
        fprintf('[+] 攻击向量生成                 : 成功 (目标线路: %s)\n', ...
            mat2str(attack_results.attacked_lines_indices));
        if isfield(attack_results, 'objective')
            fprintf('    - 优化目标值 (L1)           : %.6f\n', attack_results.objective);
        end

        [post_attack_state, ~, ~, ~, post_attack_residual, is_attack_detected] = ...
            runStateEstimation(mpc, attack_results.attacked_measurements, pmu_config, config);
        simulation_log(scenario_index).is_attack_detected = is_attack_detected;

        if ~isempty(post_attack_residual)
            post_attack_residual_norm = norm(post_attack_residual);
            simulation_log(scenario_index).post_attack_residual_norm = post_attack_residual_norm;

            fprintf('[+] 状态估计 (攻击后)           : 收敛, 残差范数 = %.6f\n', post_attack_residual_norm);
            if is_attack_detected
                fprintf('[!] 坏数据检测                   : 检测到异常 (攻击暴露)\n');
            else
                fprintf('[+] 坏数据检测                   : 未检测到异常 (攻击隐蔽)\n');
            end
        else
            simulation_log(scenario_index).post_attack_residual_norm = NaN;
            fprintf('[!] 状态估计 (攻击后)           : 未收敛\n');
        end
        
        % --- 3.5 生成并保存对比图 ---
        if isfield(attack_results, 'attacked_measurements') && ~isempty(attack_results.attacked_measurements)
            
            % 获取攻击前的测量向量
            [z_before, ~, ~] = vectorizeAndMapMeasurements( ...
                noisy_measurements, pmu_config, config.Noise, ...
                struct('num_buses', size(mpc.bus,1), 'num_branches', size(mpc.branch,1), 'selection', config.MeasurementSelection));

            % 获取攻击后的测量向量
            [z_after, map_after, ~] = vectorizeAndMapMeasurements( ...
                attack_results.attacked_measurements, pmu_config, config.Noise, ...
                struct('num_buses', size(mpc.bus,1), 'num_branches', size(mpc.branch,1), 'selection', config.MeasurementSelection));

            fig = figure('Visible', 'off', 'Position', [100, 100, 1200, 400]);
            plot(1:length(z_before), z_before, 'b-', 'LineWidth', 1.5, 'DisplayName', '攻击前测量');
            hold on;
            plot(1:length(z_after), z_after, 'r-', 'LineWidth', 1.5, 'DisplayName', '攻击后测量');
            hold off;
            
            legend('show');
            title(sprintf('场景 %d: 攻击前后测量值对比', scenario_index));
            xlabel('测量点索引');
            ylabel('测量值 (p.u.)');
            grid on;
            
            % 构建详细的 x 轴标签
            labels = {};
            running_idx = 0;
            for k=1:length(map_after)
                labels{end+1} = sprintf('%s_%s(1)', map_after{k}.type, map_after{k}.field);
                running_idx = running_idx + map_after{k}.count;
            end
            xticks(cumsum([1, cellfun(@(c) c.count, map_after)]));
            xticklabels(labels);
            xtickangle(45);

            filename = fullfile(config.Simulation.PlotOutputDir, sprintf('scenario_%03d_comparison.png', scenario_index));
            saveas(fig, filename);
            fprintf('[+] 测量对比图               : 已保存至 %s\n', filename);
            close(fig);

            % --- 3.5b 绘制电压幅值对比图 ---
            if isfield(pre_attack_state, 'V') && isfield(post_attack_state, 'V')
                fig_v = figure('Visible', 'off', 'Position', [100, 100, 1200, 400]);
                plot(pre_attack_state.V, 'b-', 'LineWidth', 1.5, 'DisplayName', '攻击前电压幅值');
                hold on;
                plot(post_attack_state.V, 'r-', 'LineWidth', 1.5, 'DisplayName', '攻击后电压幅值');
                hold off;
                legend('show');
                title(sprintf('场景 %d: 攻击前后电压幅值对比', scenario_index));
                xlabel('母线索引');
                ylabel('电压幅值 (p.u.)');
                grid on;
                filename_v = fullfile(config.Simulation.PlotOutputDir, sprintf('scenario_%03d_voltage.png', scenario_index));
                saveas(fig_v, filename_v);
                fprintf('[+] 电压幅值图               : 已保存至 %s\n', filename_v);
                close(fig_v);
            end
            
            % --- 3.5c 绘制电压相角对比图 ---
            if isfield(pre_attack_state, 'theta') && isfield(post_attack_state, 'theta')
                fig_a = figure('Visible', 'off', 'Position', [100, 100, 1200, 400]);
                plot(rad2deg(pre_attack_state.theta), 'b-', 'LineWidth', 1.5, 'DisplayName', '攻击前电压相角');
                hold on;
                plot(rad2deg(post_attack_state.theta), 'r-', 'LineWidth', 1.5, 'DisplayName', '攻击后电压相角');
                hold off;
                legend('show');
                title(sprintf('场景 %d: 攻击前后电压相角对比', scenario_index));
                xlabel('母线索引');
                ylabel('电压相角 (度)');
                grid on;
                filename_a = fullfile(config.Simulation.PlotOutputDir, sprintf('scenario_%03d_angle.png', scenario_index));
                saveas(fig_a, filename_a);
                fprintf('[+] 电压相角图               : 已保存至 %s\n', filename_a);
                close(fig_a);
            end
        end

    else
        fprintf('[-] 攻击向量生成                 : 失败\n');
        simulation_log(scenario_index).attack_generation_successful = false;
        simulation_log(scenario_index).post_attack_residual_norm = NaN;
    end

    fprintf('\n');
end

