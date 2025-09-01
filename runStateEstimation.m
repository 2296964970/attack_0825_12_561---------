function [final_estimated_state, final_estimated_measurements, final_measurement_error, final_iter_count, final_residual, detection_flag] = runStateEstimation(mpc, measured_measurements, pmu_config, config)
% runStateEstimation 混合 SCADA/PMU 的加权最小二乘 (WLS) 状态估计 + LNR-BDD。
%
% 输入
% - mpc                   : MATPOWER 案例 (baseMVA/bus/branch)
% - measured_measurements : 结构化测量 (scada/pmu)
% - pmu_config            : PMU 配置与索引
% - config                : 全局配置（Noise/StateEstimation 等）
%
% 输出
% - final_estimated_state        : 估计状态（e,f,V,theta）
% - final_estimated_measurements : 估计测量（scada/pmu）
% - final_measurement_error      : 误差（输入测量 - 估计测量）
% - final_iter_count             : 收敛迭代次数（未收敛为 -1）
% - final_residual               : 最终残差 r = z - h(x)
% - detection_flag               : 最大归一化残差 BDD 检测标志

%% === 1) 初始化与参数 ===
final_residual = [];
detection_flag = false;

noise_params = config.Noise;
verbose = config.Simulation.VerboseMode;

options = struct();
options.max_iter = config.StateEstimation.MaxIter;
options.tolerance = config.StateEstimation.Tolerance;
options.slack_bus_id = config.Grid.SlackBusId;
options.confidence_level = config.StateEstimation.ConfidenceLevel;
options.regularization_factor = config.StateEstimation.RegularizationFactor;

if verbose, fprintf('\n--- 开始状态估计 ---\n'); end

[baseMVA, bus, branch] = deal(mpc.baseMVA, mpc.bus, mpc.branch);
num_buses = size(bus, 1);
define_constants;
branch_from_bus = branch(:, F_BUS);
branch_to_bus   = branch(:, T_BUS);

[Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);
G_bus = real(Y_bus); B_bus = imag(Y_bus);
G_from = real(Y_from); B_from = imag(Y_from);
G_to = real(Y_to);   B_to = imag(Y_to);

% 状态：x = [e; f_without_slack]
slack_bus_id = options.slack_bus_id;
num_state_vars = 2 * num_buses - 1;

% 平启动或使用 PMU 电压改进初值
initial_voltage_real = ones(num_buses, 1);
initial_voltage_imag = zeros(num_buses, 1);
if isfield(measured_measurements, 'pmu') && isfield(measured_measurements.pmu, 'vm') && ~isempty(measured_measurements.pmu.vm)
    for i = 1:length(pmu_config.locations)
        bus_idx = pmu_config.locations(i);
        vm_meas = measured_measurements.pmu.vm(i);
        va_meas = measured_measurements.pmu.va(i);
        initial_voltage_real(bus_idx) = vm_meas * cos(va_meas);
        initial_voltage_imag(bus_idx) = vm_meas * sin(va_meas);
    end
end
f_indices = setdiff(1:num_buses, slack_bus_id);
state_vector = [initial_voltage_real; initial_voltage_imag(f_indices)];

%% === 2) 构建测量模型 z/H/W ===
if verbose, fprintf('--> 正在构建测量向量与权重...\n'); end
[measurement_vector, measurement_map, weights] = vectorizeAndMapMeasurements( ...
    measured_measurements, pmu_config, noise_params, ...
    struct('num_buses', num_buses, 'num_branches', size(branch,1), 'selection', config.MeasurementSelection));
weight_matrix = diag(weights);
if verbose, fprintf('--> 测量模型完成，总测量数: %d\n', length(measurement_vector)); end

%% === 3) WLS 迭代 ===
if verbose, fprintf('--> 开始 WLS 迭代求解...\n'); end
final_iter_count = -1;

for iteration_count = 1:options.max_iter
    % 3.1/3.2 计算 h(x)/H
    [calculated_measurements, jacobian_matrix] = calculate_h_H(state_vector, mpc, measurement_map, pmu_config, num_state_vars, options);

    % 3.3 正规方程（稳健求解）
    residual = measurement_vector - calculated_measurements;
    gain_matrix = jacobian_matrix' * weight_matrix * jacobian_matrix;
    right_hand_side = jacobian_matrix' * weight_matrix * residual;

    % 等价形式：A = sqrt(W)*H, b = sqrt(W)*r
    sqrt_w = sqrt(diag(weight_matrix));
    A = bsxfun(@times, sqrt_w, jacobian_matrix);
    bvec = sqrt_w .* residual;

    try
        if exist('lsqminnorm','file') == 2
            state_update_vector = lsqminnorm(A, bvec);
        else
            state_update_vector = A \ bvec;
        end
    catch
        % 回退：阻尼正规方程
        rc = rcond(gain_matrix);
        if rc < 1e-10
            if verbose, fprintf('  - 警告: 增益矩阵接近奇异 (cond≈%.2e)，应用正则化。\n', rc); end
            dmax = max(1.0, max(abs(diag(gain_matrix))));
            tau = max([options.regularization_factor, 1e-6, 1e-3 * dmax]);
            if rc < 1e-14, tau = max(tau, 1e-2 * dmax); end
            gain_matrix = gain_matrix + tau * eye(size(gain_matrix));
        end
        state_update_vector = gain_matrix \ right_hand_side;
    end

    % 3.4 更新与收敛判据
    state_vector = state_vector + state_update_vector;
    max_state_update = max(abs(state_update_vector));
    if verbose, fprintf('  - 迭代 %2d: max|Δx| = %.2e\n', iteration_count, max_state_update); end
    if max_state_update < options.tolerance
        if verbose, fprintf('--> WLS 于 %d 次迭代后收敛。\n', iteration_count); end
        final_iter_count = iteration_count;
        break;
    end
    if iteration_count == options.max_iter && verbose
        fprintf('--> 达到最大迭代次数 %d，WLS 未收敛。\n', options.max_iter);
    end
end

%% === 4) 结果回填 ===
if final_iter_count > 0
    % 直角坐标 -> 极坐标
    final_voltage_real = state_vector(1:num_buses);
    final_voltage_imag = zeros(num_buses, 1);
    final_voltage_imag(f_indices) = state_vector(num_buses+1:end);

    final_estimated_state = struct();
    final_estimated_state.e = final_voltage_real;
    final_estimated_state.f = final_voltage_imag;
    final_estimated_state.V = hypot(final_voltage_real, final_voltage_imag);
    final_estimated_state.theta = atan2(final_voltage_imag, final_voltage_real);

    % 构造估计测量值（用于分析）
    final_estimated_measurements = struct();
    est_v_complex = final_voltage_real + 1j * final_voltage_imag;
    estimated_S_injection = est_v_complex .* conj(Y_bus * est_v_complex);
    estimated_S_from = est_v_complex(branch_from_bus) .* conj(Y_from * est_v_complex);
    estimated_S_to   = est_v_complex(branch_to_bus)   .* conj(Y_to   * est_v_complex);

    final_estimated_measurements.scada = struct();
    final_estimated_measurements.scada.v  = abs(est_v_complex);
    final_estimated_measurements.scada.pi = real(estimated_S_injection);
    final_estimated_measurements.scada.qi = imag(estimated_S_injection);
    final_estimated_measurements.scada.pf = real(estimated_S_from);
    final_estimated_measurements.scada.qf = imag(estimated_S_from);
    final_estimated_measurements.scada.pt = real(estimated_S_to);
    final_estimated_measurements.scada.qt = imag(estimated_S_to);

    if isfield(pmu_config, 'locations') && ~isempty(pmu_config.locations)
        final_estimated_measurements.pmu = struct();
        estimated_I_from = Y_from * est_v_complex;
        estimated_I_to   = Y_to   * est_v_complex;
        final_estimated_measurements.pmu.va = angle(est_v_complex(pmu_config.locations));
        final_estimated_measurements.pmu.vm = abs( est_v_complex(pmu_config.locations));
        if isfield(pmu_config, 'pmu_from_branch_indices') && ~isempty(pmu_config.pmu_from_branch_indices)
            idxf = pmu_config.pmu_from_branch_indices;
            final_estimated_measurements.pmu.iaf = angle(estimated_I_from(idxf));
            final_estimated_measurements.pmu.imf = abs(  estimated_I_from(idxf));
        end
        if isfield(pmu_config, 'pmu_to_branch_indices') && ~isempty(pmu_config.pmu_to_branch_indices)
            idxt = pmu_config.pmu_to_branch_indices;
            final_estimated_measurements.pmu.iat = angle(estimated_I_to(idxt));
            final_estimated_measurements.pmu.imt = abs(  estimated_I_to(idxt));
        end
    end

    % 计算测量误差（输入 - 估计）
    final_measurement_error = struct();
    if isfield(measured_measurements, 'scada')
        final_measurement_error.scada = struct();
        sc = measured_measurements.scada;
        if isfield(sc, 'v'),  final_measurement_error.scada.v  = sc.v  - final_estimated_measurements.scada.v(1:length(sc.v)); end
        if isfield(sc, 'pi'), final_measurement_error.scada.pi = sc.pi - final_estimated_measurements.scada.pi(1:length(sc.pi)); end
        if isfield(sc, 'qi'), final_measurement_error.scada.qi = sc.qi - final_estimated_measurements.scada.qi(1:length(sc.qi)); end
        if isfield(sc, 'pf'), final_measurement_error.scada.pf = sc.pf - final_estimated_measurements.scada.pf(1:length(sc.pf)); end
        if isfield(sc, 'qf'), final_measurement_error.scada.qf = sc.qf - final_estimated_measurements.scada.qf(1:length(sc.qf)); end
        if isfield(sc, 'pt'), final_measurement_error.scada.pt = sc.pt - final_estimated_measurements.scada.pt(1:length(sc.pt)); end
        if isfield(sc, 'qt'), final_measurement_error.scada.qt = sc.qt - final_estimated_measurements.scada.qt(1:length(sc.qt)); end
    end
    if isfield(measured_measurements, 'pmu')
        final_measurement_error.pmu = struct();
        pmu_in = measured_measurements.pmu;
        if isfield(pmu_in, 'vm'),  final_measurement_error.pmu.vm  = pmu_in.vm  - final_estimated_measurements.pmu.vm; end
        if isfield(pmu_in, 'va'),  final_measurement_error.pmu.va  = pmu_in.va  - final_estimated_measurements.pmu.va; end
        if isfield(pmu_in, 'imf'), final_measurement_error.pmu.imf = pmu_in.imf - final_estimated_measurements.pmu.imf; end
        if isfield(pmu_in, 'iaf'), final_measurement_error.pmu.iaf = pmu_in.iaf - final_estimated_measurements.pmu.iaf; end
        if isfield(pmu_in, 'imt'), final_measurement_error.pmu.imt = pmu_in.imt - final_estimated_measurements.pmu.imt; end
        if isfield(pmu_in, 'iat'), final_measurement_error.pmu.iat = pmu_in.iat - final_estimated_measurements.pmu.iat; end
    end
    if verbose, fprintf('--> 状态估计完成。\n'); end
else
    if verbose, fprintf('--> 状态估计未收敛，返回 NaN。\n'); end
    nan_vec = NaN(num_buses, 1);
    final_estimated_state = struct('V', nan_vec, 'theta', nan_vec, 'e', nan_vec, 'f', nan_vec);
    final_estimated_measurements = struct();
    final_measurement_error = struct();
    final_residual = [];
end

%% === 5) 坏数据检测 (LNR) ===
if final_iter_count > 0
    if verbose, fprintf('--> 正在执行坏数据检测...\n'); end

    % 使用最终状态重算 h(x_final)
    e_final = final_estimated_state.e;
    f_final_full = final_estimated_state.f;
    f_indices = setdiff(1:num_buses, options.slack_bus_id);
    state_vector_final = [e_final; f_final_full(f_indices)];
    num_state_vars = 2 * num_buses - 1;
    [h_final, ~] = calculate_h_H(state_vector_final, mpc, measurement_map, pmu_config, num_state_vars, options);
    final_residual = measurement_vector - h_final;

    % 归一化残差的方差对角线
    G = jacobian_matrix' * weight_matrix * jacobian_matrix;
    rc = rcond(G);
    if rc < 1e-10
        dmax = max(1.0, max(abs(diag(G))));
        tau = max([options.regularization_factor, 1e-6, 1e-3 * dmax]);
        if rc < 1e-14, tau = max(tau, 1e-2 * dmax); end
        G = G + tau * eye(size(G));
    end
    Ginv = pinv(G, 1e-10);
    wdiag = diag(weight_matrix);
    Rdiag = 1 ./ max(wdiag, 1e-18);              % 避免 inv(W)
    HG = jacobian_matrix * Ginv;
    diag_HGHT = sum(HG .* jacobian_matrix, 2);   % diag(H*Ginv*H')
    diag_residual_cov = abs(Rdiag - diag_HGHT);
    diag_residual_cov(diag_residual_cov < 1e-9) = 1e-9;

    normalized_residual = abs(final_residual) ./ sqrt(diag_residual_cov);
    [max_normalized_residual, ~] = max(normalized_residual);

    % Bonferroni 阈值
    m = length(measurement_vector);
    alpha = 1 - options.confidence_level;
    alpha_corr = alpha / m;
    critical_value = norminv(1 - alpha_corr / 2, 0, 1);

    if verbose
        fprintf('--> 坏数据检测 LNR = %.4f, 阈值(%.0f%%) = %.4f\n', ...
            max_normalized_residual, options.confidence_level*100, critical_value);
    end
    detection_flag = (max_normalized_residual > critical_value);
    if verbose
        if detection_flag, fprintf('--> 检测到坏数据。\n'); else, fprintf('--> 未检测到坏数据。\n'); end
    end
end

end

