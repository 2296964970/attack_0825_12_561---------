function [true_measurements, noisy_measurements, opf_results, system_params, pmu_config] = generateMeasurements(scenario_index, config, load_multipliers)
%generateMeasurements 从基础电网模型开始，完整地处理从OPF计算到测量数据生成的整个流程
%
%   输入:
%     scenario_index   - 当前仿真场景索引（1到NumScenarios）
%     config          - 完整的系统配置结构体（来自load_config）
%     load_multipliers - 96个场景的负荷变化率数组（来自calculate_rate）
%
%   输出:
%     true_measurements  - 无噪声的真实测量值
%     noisy_measurements - 添加了高斯噪声的模拟测量值
%     opf_results       - 最优潮流计算结果
%     system_params     - 系统参数结构体 (Y-bus等)
%     pmu_config        - PMU相关的索引和配置

define_constants; % 加载 VM, VA, PF, QF 等MATPOWER常量

% =================================================================
%  1. 执行 OPF 计算 (从基础电网模型开始)
% =================================================================

% --- 加载基础电网模型 ---
mpc_base = loadcase(config.System.CaseName);

% --- 根据场景索引调整负荷 ---
mpc = mpc_base; % 复制一份基础模型，避免修改原始数据
mpc.bus(:, PD) = mpc_base.bus(:, PD) * config.Simulation.LoadScaleFactor * load_multipliers(scenario_index);
mpc.bus(:, QD) = mpc_base.bus(:, QD) * config.Simulation.LoadScaleFactor * load_multipliers(scenario_index);

% --- 设置最优潮流求解选项 ---
mpopt = mpoption('verbose', 0, 'out.all', 0);

% --- 执行最优潮流计算 ---
opf_results = runopf(mpc, mpopt);

if ~opf_results.success
    error('场景 %d 的最优潮流 (OPF) 未收敛!', scenario_index);
end

% =================================================================
%  2. 计算系统参数和PMU配置
% =================================================================

% --- 提取基础电网参数 ---
[baseMVA, bus, ~, branch] = deal(mpc_base.baseMVA, mpc_base.bus, mpc_base.gen, mpc_base.branch);

% --- 构建导纳矩阵 ---
[Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);

% --- 构建系统参数结构体（用于兼容性） ---
system_params = struct('baseMVA', baseMVA, 'Y_bus', Y_bus, 'Y_from', Y_from, 'Y_to', Y_to);

% --- 计算PMU相关索引 ---
from_bus_indices = branch(:, F_BUS);
to_bus_indices = branch(:, T_BUS);
pmu_bus_locations = config.Grid.PmuBusLocations;

pmu_va_bus_indices = pmu_bus_locations;
pmu_vm_bus_indices = pmu_bus_locations;
pmu_from_branch_indices = find(ismember(from_bus_indices, pmu_bus_locations)); % from bus is pmu bus
pmu_to_branch_indices = find(ismember(to_bus_indices, pmu_bus_locations)); % to bus is pmu bus
pmu_ia_f_branch_indices = pmu_from_branch_indices;
pmu_im_f_branch_indices = pmu_from_branch_indices;
pmu_ia_t_branch_indices = pmu_to_branch_indices;
pmu_im_t_branch_indices = pmu_to_branch_indices;

% --- 构建PMU配置结构体 ---
pmu_config = struct('locations', pmu_bus_locations, 'pmu_from_branch_indices', pmu_from_branch_indices, ...
    'pmu_to_branch_indices', pmu_to_branch_indices, 'from_bus_indices', from_bus_indices, ...
    'to_bus_indices', to_bus_indices);

% =================================================================
%  3. 生成真实测量数据 (无噪声)
% =================================================================

% --- 提取真实状态变量 ---
true_v_mag = opf_results.bus(:, VM);
true_v_ang_rad = opf_results.bus(:, VA) * pi / 180;
true_v_complex = true_v_mag .* exp(1j * true_v_ang_rad);

% --- 计算真实测量值 ---
true_measurements = struct();

% SCADA 测量
true_measurements.scada.v = true_v_mag;
true_measurements.scada.pf = opf_results.branch(:, PF) / baseMVA;
true_measurements.scada.qf = opf_results.branch(:, QF) / baseMVA;
true_measurements.scada.pt = opf_results.branch(:, PT) / baseMVA;
true_measurements.scada.qt = opf_results.branch(:, QT) / baseMVA;
true_s_injection = true_v_complex .* conj(Y_bus * true_v_complex);
true_measurements.scada.pi = real(true_s_injection);
true_measurements.scada.qi = imag(true_s_injection);

% PMU 测量
true_i_from = Y_from * true_v_complex;
true_i_to = Y_to * true_v_complex;
true_measurements.pmu.va = true_v_ang_rad(pmu_va_bus_indices);
true_measurements.pmu.vm = true_v_mag(pmu_vm_bus_indices);
true_measurements.pmu.iaf = angle(true_i_from(pmu_ia_f_branch_indices));
true_measurements.pmu.imf = abs(true_i_from(pmu_im_f_branch_indices));
true_measurements.pmu.iat = angle(true_i_to(pmu_ia_t_branch_indices));
true_measurements.pmu.imt = abs(true_i_to(pmu_im_t_branch_indices));

% =================================================================
%  4. 添加噪声以生成模拟测量值
% =================================================================
noisy_measurements = struct();

% SCADA 噪声
noisy_measurements.scada.v = true_measurements.scada.v + config.Noise.scada.v * randn(size(true_measurements.scada.v));
noisy_measurements.scada.pf = true_measurements.scada.pf + config.Noise.scada.p * randn(size(true_measurements.scada.pf));
noisy_measurements.scada.qf = true_measurements.scada.qf + config.Noise.scada.q * randn(size(true_measurements.scada.qf));
noisy_measurements.scada.pt = true_measurements.scada.pt + config.Noise.scada.p * randn(size(true_measurements.scada.pt));
noisy_measurements.scada.qt = true_measurements.scada.qt + config.Noise.scada.q * randn(size(true_measurements.scada.qt));
noisy_measurements.scada.pi = true_measurements.scada.pi + config.Noise.scada.p * randn(size(true_measurements.scada.pi));
noisy_measurements.scada.qi = true_measurements.scada.qi + config.Noise.scada.q * randn(size(true_measurements.scada.qi));

% PMU 噪声
noisy_measurements.pmu.va = true_measurements.pmu.va + config.Noise.pmu.va * randn(size(true_measurements.pmu.va));
noisy_measurements.pmu.vm = true_measurements.pmu.vm + config.Noise.pmu.vm * randn(size(true_measurements.pmu.vm));
noisy_measurements.pmu.iaf = true_measurements.pmu.iaf + config.Noise.pmu.ia * randn(size(true_measurements.pmu.iaf));
noisy_measurements.pmu.imf = true_measurements.pmu.imf + config.Noise.pmu.im * randn(size(true_measurements.pmu.imf));
noisy_measurements.pmu.iat = true_measurements.pmu.iat + config.Noise.pmu.ia * randn(size(true_measurements.pmu.iat));
noisy_measurements.pmu.imt = true_measurements.pmu.imt + config.Noise.pmu.im * randn(size(true_measurements.pmu.imt));

end