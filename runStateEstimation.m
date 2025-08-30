function [final_estimated_state, final_estimated_measurements, final_measurement_error, final_iter_count, final_residual, detection_flag] = runStateEstimation(mpc, measured_measurements, pmu_config, config)
%runStateEstimation 执行混合SCADA/PMU状态估计
%
%   此函数使用加权最小二乘(WLS)算法来估计电力系统的状态。
%   它能够处理传统的SCADA测量和高精度的PMU同步相量测量。
%   状态变量在内部使用直角坐标（e, f）表示，以获得线性的PMU测量模型。
%
%   输入:
%     mpc                  - MATPOWER案例结构体 (包含 baseMVA, bus, branch)。
%     measured_measurements - 包含SCADA和PMU测量值的结构体。
%     pmu_config           - PMU配置信息，如安装位置。
%     config               - 配置结构体 (包含噪声参数、状态估计选项等)。
%
%   输出:
%     final_estimated_state      - 最终估计的状态 (V, theta, e, f)。
%     final_estimated_measurements - 根据最终状态计算出的所有测量值。
%     final_measurement_error    - 输入测量值与估计测量值之间的差值。
%     final_iter_count           - 最终的WLS迭代次数。
%     final_residual             - 最终的测量残差向量 r = z - h(x)。
%     detection_flag             - 坏数据检测标志 (true表示检测到异常)。

%% =======================================================================
%  第一部分: 初始化与参数设置
% ========================================================================

% --- 1.1 初始化输出变量和从config提取参数 ---
final_residual = [];
detection_flag = false; % 默认未检测到坏数据

% 从config结构体中提取参数
noise_params = config.Noise;
verbose = config.Simulation.VerboseMode;

% 构建options结构体
options = struct();
options.max_iter = config.StateEstimation.MaxIter;
options.tolerance = config.StateEstimation.Tolerance;
options.slack_bus_id = config.Grid.SlackBusId;
options.confidence_level = config.StateEstimation.ConfidenceLevel;
options.regularization_factor = config.StateEstimation.RegularizationFactor;

if verbose
    fprintf('\n--- 开始状态估计 ---\n');
end

% --- 1.2 提取电网参数并构建导纳矩阵 ---
[baseMVA, bus, branch] = deal(mpc.baseMVA, mpc.bus, mpc.branch);
num_buses = size(bus, 1);
define_constants; % 引入 F_BUS, T_BUS 等MATPOWER常量
branch_from_bus = branch(:, F_BUS);
branch_to_bus = branch(:, T_BUS);

% 构建导纳矩阵 Ybus, Yf, Yt
[Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);
G_bus = real(Y_bus); B_bus = imag(Y_bus);
G_from = real(Y_from); B_from = imag(Y_from);
G_to = real(Y_to);   B_to = imag(Y_to);

% --- 1.3 初始化状态向量 (直角坐标) ---
% 状态向量 x = [e_1, ..., e_n, f_1, ..., f_slack-1, f_slack+1, ..., f_n]'
% e 是电压的实部，f 是电压的虚部。为了确保雅可比矩阵非奇异，参考节点的虚部f被固定为0。
slack_bus_id = options.slack_bus_id;
num_state_vars = 2 * num_buses - 1;

% 使用平启动（flat start）或PMU测量值改进初始状态
initial_voltage_real = ones(num_buses, 1);
initial_voltage_imag = zeros(num_buses, 1);

if isfield(measured_measurements, 'pmu') && isfield(measured_measurements.pmu, 'vm') && ~isempty(measured_measurements.pmu.vm)
    % 如果有PMU电压测量，用它们来改进初始猜测值，以加速收敛
    for i = 1:length(pmu_config.locations)
        bus_idx = pmu_config.locations(i);
        vm_measured = measured_measurements.pmu.vm(i);
        va_measured = measured_measurements.pmu.va(i);
        initial_voltage_real(bus_idx) = vm_measured * cos(va_measured);
        initial_voltage_imag(bus_idx) = vm_measured * sin(va_measured);
    end
end

% 构建状态向量，排除参考节点的虚部
f_indices = setdiff(1:num_buses, slack_bus_id);
state_vector = [initial_voltage_real; initial_voltage_imag(f_indices)];


%% =======================================================================
%  第二部分: 构建测量模型 (测量向量 z 和 权重矩阵 W)
% ========================================================================
if verbose
    fprintf('--> 正在构建测量向量和权重矩阵...\n');
end

[measurement_vector, ~, weights] = vectorizeAndMapMeasurements(measured_measurements, pmu_config, noise_params);
weight_matrix = diag(weights);

if verbose
    fprintf('--> 测量模型构建完成。总测量数: %d\n', length(measurement_vector));
end


%% =======================================================================
%  第三部分: 加权最小二乘 (WLS) 迭代求解
% ========================================================================
if verbose
    fprintf('--> 开始WLS迭代求解...\n');
end

final_iter_count = -1; % 初始化为-1，表示未收敛

for iteration_count = 1:options.max_iter
    
    % --- 3.1 & 3.2: 调用外部函数计算 h(x) 和雅可比矩阵 H ---
    num_measurements = length(measurement_vector);
    [calculated_measurements, jacobian_matrix] = calculate_h_H(state_vector, mpc, measured_measurements, pmu_config, num_measurements, num_state_vars, options);
    
    % --- 3.3 求解正规方程 ---
    residual = measurement_vector - calculated_measurements;
    gain_matrix = jacobian_matrix' * weight_matrix * jacobian_matrix;
    right_hand_side = jacobian_matrix' * weight_matrix * residual;
    
    % 检查矩阵条件数，如果接近奇异则进行正则化以提高数值稳定性
    condition_number = rcond(gain_matrix);
    if condition_number < 1e-14
        if verbose, fprintf('  - 警告: 增益矩阵接近奇异 (条件数: %.2e)，应用正则化。\n', condition_number); end
        % 使用配置中的正则化因子
        gain_matrix = gain_matrix + options.regularization_factor * eye(size(gain_matrix));
    end
    
    state_update_vector = gain_matrix \ right_hand_side;
    
    % --- 3.4 更新状态并检查收敛 ---
    state_vector = state_vector + state_update_vector;
    max_state_update = max(abs(state_update_vector));
    
    if verbose
        fprintf('  - 迭代 %2d: 最大状态更新 = %.2e\n', iteration_count, max_state_update);
    end
    
    if max_state_update < options.tolerance
        if verbose, fprintf('--> WLS在 %d 次迭代后收敛。\n', iteration_count); end
        final_iter_count = iteration_count;
        break;
    end
    
    if iteration_count == options.max_iter
        if verbose, fprintf('--> 达到最大迭代次数 %d，WLS未收敛。\n', options.max_iter); end
    end
end


%% =======================================================================
%  第四部分: 结果后处理
% ========================================================================

if final_iter_count > 0 % --- 4.1 处理收敛的情况 ---
    % 提取最终状态 (直角坐标)
    final_voltage_real = state_vector(1:num_buses);
    final_voltage_imag = zeros(num_buses, 1);
    final_voltage_imag(f_indices) = state_vector(num_buses+1:end);
    
    % 转换为极坐标并构建输出结构体
    final_estimated_state = struct();
    final_estimated_state.e = final_voltage_real;
    final_estimated_state.f = final_voltage_imag;
    final_estimated_state.V = sqrt(final_voltage_real.^2 + final_voltage_imag.^2);
    final_estimated_state.theta = atan2(final_voltage_imag, final_voltage_real);
    
    % --- 计算完整的估计测量值集合 (用于分析和比较) ---
    final_estimated_measurements = struct();
    est_v_complex = final_voltage_real + 1j * final_voltage_imag;
    
    % SCADA 估计值
    estimated_S_injection = est_v_complex .* conj(Y_bus * est_v_complex);
    estimated_S_from = est_v_complex(branch_from_bus) .* conj(Y_from * est_v_complex);
    estimated_S_to = est_v_complex(branch_to_bus) .* conj(Y_to * est_v_complex);
    
    final_estimated_measurements.scada = struct();
    final_estimated_measurements.scada.v = abs(est_v_complex);
    final_estimated_measurements.scada.pi = real(estimated_S_injection);
    final_estimated_measurements.scada.qi = imag(estimated_S_injection);
    final_estimated_measurements.scada.pf = real(estimated_S_from);
    final_estimated_measurements.scada.qf = imag(estimated_S_from);
    final_estimated_measurements.scada.pt = real(estimated_S_to);
    final_estimated_measurements.scada.qt = imag(estimated_S_to);
    
    % PMU 估计值
    if isfield(pmu_config, 'locations') && ~isempty(pmu_config.locations)
        final_estimated_measurements.pmu = struct();
        estimated_I_from = Y_from * est_v_complex;
        estimated_I_to = Y_to * est_v_complex;
        
        final_estimated_measurements.pmu.va = angle(est_v_complex(pmu_config.locations));
        final_estimated_measurements.pmu.vm = abs(est_v_complex(pmu_config.locations));
        if isfield(pmu_config, 'pmu_from_branch_indices') && ~isempty(pmu_config.pmu_from_branch_indices)
            pmu_from_idx = pmu_config.pmu_from_branch_indices;
            final_estimated_measurements.pmu.iaf = angle(estimated_I_from(pmu_from_idx));
            final_estimated_measurements.pmu.imf = abs(estimated_I_from(pmu_from_idx));
        end
        if isfield(pmu_config, 'pmu_to_branch_indices') && ~isempty(pmu_config.pmu_to_branch_indices)
            pmu_to_idx = pmu_config.pmu_to_branch_indices;
            final_estimated_measurements.pmu.iat = angle(estimated_I_to(pmu_to_idx));
            final_estimated_measurements.pmu.imt = abs(estimated_I_to(pmu_to_idx));
        end
    end
    
    % --- 计算测量误差 ---
    final_measurement_error = struct();
    
    % 计算SCADA测量误差
    if isfield(measured_measurements, 'scada')
        final_measurement_error.scada = struct();
        scada_in = measured_measurements.scada;
        
        if isfield(scada_in, 'v')
            final_measurement_error.scada.v = scada_in.v - final_estimated_measurements.scada.v(1:length(scada_in.v));
        end
        if isfield(scada_in, 'pi')
            final_measurement_error.scada.pi = scada_in.pi - final_estimated_measurements.scada.pi(1:length(scada_in.pi));
        end
        if isfield(scada_in, 'qi')
            final_measurement_error.scada.qi = scada_in.qi - final_estimated_measurements.scada.qi(1:length(scada_in.qi));
        end
        if isfield(scada_in, 'pf')
            final_measurement_error.scada.pf = scada_in.pf - final_estimated_measurements.scada.pf(1:length(scada_in.pf));
        end
        if isfield(scada_in, 'qf')
            final_measurement_error.scada.qf = scada_in.qf - final_estimated_measurements.scada.qf(1:length(scada_in.qf));
        end
        if isfield(scada_in, 'pt')
            final_measurement_error.scada.pt = scada_in.pt - final_estimated_measurements.scada.pt(1:length(scada_in.pt));
        end
        if isfield(scada_in, 'qt')
            final_measurement_error.scada.qt = scada_in.qt - final_estimated_measurements.scada.qt(1:length(scada_in.qt));
        end
    end
    
    % 计算PMU测量误差
    if isfield(measured_measurements, 'pmu')
        final_measurement_error.pmu = struct();
        pmu_in = measured_measurements.pmu;
        
        if isfield(pmu_in, 'vm')
            final_measurement_error.pmu.vm = pmu_in.vm - final_estimated_measurements.pmu.vm;
        end
        if isfield(pmu_in, 'va')
            final_measurement_error.pmu.va = pmu_in.va - final_estimated_measurements.pmu.va;
        end
        if isfield(pmu_in, 'imf')
            final_measurement_error.pmu.imf = pmu_in.imf - final_estimated_measurements.pmu.imf;
        end
        if isfield(pmu_in, 'iaf')
            final_measurement_error.pmu.iaf = pmu_in.iaf - final_estimated_measurements.pmu.iaf;
        end
        if isfield(pmu_in, 'imt')
            final_measurement_error.pmu.imt = pmu_in.imt - final_estimated_measurements.pmu.imt;
        end
        if isfield(pmu_in, 'iat')
            final_measurement_error.pmu.iat = pmu_in.iat - final_estimated_measurements.pmu.iat;
        end
    end
    
    if verbose, fprintf('--> 状态估计完成。\n'); end
else
    % --- 4.2 处理未收敛的情况 ---
    if verbose, fprintf('--> 状态估计未收敛，返回NaN。\n'); end
    nan_vec = NaN(num_buses, 1);
    final_estimated_state = struct('V', nan_vec, 'theta', nan_vec, 'e', nan_vec, 'f', nan_vec);
    final_estimated_measurements = struct();
    final_measurement_error = struct();
    final_residual = [];
end


%% =======================================================================
%  第五部分: 坏数据检测 (BDD)
% ========================================================================
if final_iter_count > 0 % 仅在收敛时执行
    if verbose, fprintf('--> 正在执行坏数据检测...\n'); end
    
    % --- 5.1 使用最终状态精确计算 h(x_final) 和残差 r ---
    % 此处重新计算h(x)以获得与最终状态完全对应的残差，而不是使用最后一次迭代的近似值
    % 通过复用 calculate_h_H 函数来计算，以消除代码重复。

    % 1. 构建与 calculate_h_H 所需格式一致的最终状态向量
    e_final = final_estimated_state.e;
    f_final_full = final_estimated_state.f;
    f_indices = setdiff(1:num_buses, options.slack_bus_id);
    state_vector_final = [e_final; f_final_full(f_indices)];

    % 2. 调用函数计算 h(x_final)
    %    我们只需要 h(x)，因此使用 ~ 忽略第二个输出（雅可比矩阵 H）
    num_measurements = length(measurement_vector);
    num_state_vars = 2 * num_buses - 1;
    [h_final, ~] = calculate_h_H(state_vector_final, mpc, measured_measurements, pmu_config, num_measurements, num_state_vars, options);

    final_residual = measurement_vector - h_final;

    % --- 5.2 执行最大归一化残差 (LNR) 检验 ---
    try
        % 残差协方差矩阵 R_cov = inv(W) - H * inv(G) * H'
        residual_covariance = inv(weight_matrix) - jacobian_matrix * (gain_matrix \ jacobian_matrix');
        
        % 提取对角线元素用于归一化
        diag_residual_cov = abs(diag(residual_covariance));
        diag_residual_cov(diag_residual_cov < 1e-9) = 1e-9; % 避免除以零

        % 计算归一化残差 (服从标准正态分布)
        normalized_residual = abs(final_residual) ./ sqrt(diag_residual_cov);
        [max_normalized_residual, ~] = max(normalized_residual);
        
        % 设定检验阈值
        % 由于我们对多个测量值进行了检验（多重比较），需要对显著性水平alpha进行校正
        % 此处使用保守的Bonferroni校正: alpha_corrected = alpha / m
        num_total_measurements = length(measurement_vector);
        alpha = 1 - options.confidence_level;
        alpha_corrected = alpha / num_total_measurements; 
        
        % 归一化残差服从 N(0,1)，因此从标准正态分布中找到临界值
        critical_value = norminv(1 - alpha_corrected / 2, 0, 1);
        
        if verbose
            fprintf('--> 坏数据检测: 最大归一化残差 = %.4f, 阈值 (置信度 %.2f%%) = %.4f\n', ...
                max_normalized_residual, options.confidence_level*100, critical_value);
        end

        % 判断是否检测到坏数据
        if max_normalized_residual > critical_value
            detection_flag = true;
            if verbose, fprintf('--> <font color="red">检测到坏数据!</font>\n'); end
        else
            if verbose, fprintf('--> 未检测到坏数据。\n'); end
        end
    catch ME
        if verbose
            fprintf('--> 警告: 坏数据检测计算失败: %s\n', ME.message);
        end
    end
end

if verbose, fprintf('--- 状态估计结束 ---\n\n'); end

end
