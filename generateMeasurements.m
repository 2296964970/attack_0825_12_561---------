function [true_measurements, noisy_measurements, opf_results, system_params, pmu_config] = generateMeasurements(scenario_index, config, load_multipliers)
% generateMeasurements 从 OPF 到测量生成（SCADA/PMU + 噪声）的完整流程。
%
% 输入
% - scenario_index: 当前仿真场景索引（1..NumScenarios）
% - config        : 配置结构体（load_config 返回）
% - load_multipliers: 负荷倍率数组（calculate_rate 返回，长度等于场景数）
%
% 输出
% - true_measurements : 无噪声真实测量（SCADA/PMU）
% - noisy_measurements: 加噪后的模拟测量（SCADA/PMU）
% - opf_results       : OPF 求解结果结构体
% - system_params     : 系统参数（baseMVA, Ybus/Yf/Yt）
% - pmu_config        : PMU 索引与布局配置

define_constants; % VM, VA, PF, QF, PT, QT 等常量

% =================================================================
% 1) 执行 OPF 计算
% =================================================================

% 加载基础电网模型并按场景调整负荷
mpc_base = loadcase(config.System.CaseName);
mpc = mpc_base;
scale = config.Simulation.LoadScaleFactor * load_multipliers(scenario_index);
mpc.bus(:, PD) = mpc_base.bus(:, PD) * scale;
mpc.bus(:, QD) = mpc_base.bus(:, QD) * scale;

% 设定安静输出
mpopt = mpoption('verbose', 0, 'out.all', 0);

try
    opf_results = runopf(mpc, mpopt);
catch ME
    warning('场景 %d 的 OPF 执行异常: %s', scenario_index, ME.message);
    opf_results = struct('success', false);
end

% OPF 未收敛：返回占位输出
if ~isfield(opf_results, 'success') || ~opf_results.success
    true_measurements  = struct();
    noisy_measurements = struct();
    system_params      = struct();
    pmu_config         = struct();
    return;
end

% =================================================================
% 2) 计算系统参数与 PMU 配置
% =================================================================

[baseMVA, bus, ~, branch] = deal(mpc_base.baseMVA, mpc_base.bus, mpc_base.gen, mpc_base.branch);
[Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);
system_params = struct('baseMVA', baseMVA, 'Y_bus', Y_bus, 'Y_from', Y_from, 'Y_to', Y_to);

from_bus_indices = branch(:, F_BUS);
to_bus_indices   = branch(:, T_BUS);
pmu_bus_locations = config.Grid.PmuBusLocations;

pmu_from_branch_indices = find(ismember(from_bus_indices, pmu_bus_locations));
pmu_to_branch_indices   = find(ismember(to_bus_indices,   pmu_bus_locations));

pmu_config = struct('locations', pmu_bus_locations, ...
    'pmu_from_branch_indices', pmu_from_branch_indices, ...
    'pmu_to_branch_indices', pmu_to_branch_indices, ...
    'from_bus_indices', from_bus_indices, ...
    'to_bus_indices', to_bus_indices);

% =================================================================
% 3) 生成真实测量（无噪声）
% =================================================================

true_v_mag = opf_results.bus(:, VM);
true_v_ang_rad = opf_results.bus(:, VA) * pi / 180;
true_v_complex = true_v_mag .* exp(1j * true_v_ang_rad);

true_measurements = struct();

% SCADA
true_measurements.scada.v  = true_v_mag;
true_measurements.scada.pf = opf_results.branch(:, PF) / baseMVA;
true_measurements.scada.qf = opf_results.branch(:, QF) / baseMVA;
true_measurements.scada.pt = opf_results.branch(:, PT) / baseMVA;
true_measurements.scada.qt = opf_results.branch(:, QT) / baseMVA;
true_s_injection = true_v_complex .* conj(Y_bus * true_v_complex);
true_measurements.scada.pi = real(true_s_injection);
true_measurements.scada.qi = imag(true_s_injection);

% PMU（以布局筛选母线/支路，角度弧度制）
true_i_from = Y_from * true_v_complex;
true_i_to   = Y_to   * true_v_complex;
true_measurements.pmu.va  = true_v_ang_rad(pmu_bus_locations);
true_measurements.pmu.vm  = true_v_mag(pmu_bus_locations);
true_measurements.pmu.iaf = angle(true_i_from(pmu_from_branch_indices));
true_measurements.pmu.imf = abs(true_i_from(pmu_from_branch_indices));
true_measurements.pmu.iat = angle(true_i_to(pmu_to_branch_indices));
true_measurements.pmu.imt = abs(true_i_to(pmu_to_branch_indices));

% =================================================================
% 4) 添加噪声得到模拟测量
% =================================================================

noisy_measurements = struct();

% SCADA 噪声
noisy_measurements.scada.v  = true_measurements.scada.v  + config.Noise.scada.v * randn(size(true_measurements.scada.v));
noisy_measurements.scada.pf = true_measurements.scada.pf + config.Noise.scada.p * randn(size(true_measurements.scada.pf));
noisy_measurements.scada.qf = true_measurements.scada.qf + config.Noise.scada.q * randn(size(true_measurements.scada.qf));
noisy_measurements.scada.pt = true_measurements.scada.pt + config.Noise.scada.p * randn(size(true_measurements.scada.pt));
noisy_measurements.scada.qt = true_measurements.scada.qt + config.Noise.scada.q * randn(size(true_measurements.scada.qt));
noisy_measurements.scada.pi = true_measurements.scada.pi + config.Noise.scada.p * randn(size(true_measurements.scada.pi));
noisy_measurements.scada.qi = true_measurements.scada.qi + config.Noise.scada.q * randn(size(true_measurements.scada.qi));

% PMU 噪声
noisy_measurements.pmu.va  = true_measurements.pmu.va  + config.Noise.pmu.va * randn(size(true_measurements.pmu.va));
noisy_measurements.pmu.vm  = true_measurements.pmu.vm  + config.Noise.pmu.vm * randn(size(true_measurements.pmu.vm));
noisy_measurements.pmu.iaf = true_measurements.pmu.iaf + config.Noise.pmu.ia * randn(size(true_measurements.pmu.iaf));
noisy_measurements.pmu.imf = true_measurements.pmu.imf + config.Noise.pmu.im * randn(size(true_measurements.pmu.imf));
noisy_measurements.pmu.iat = true_measurements.pmu.iat + config.Noise.pmu.ia * randn(size(true_measurements.pmu.iat));
noisy_measurements.pmu.imt = true_measurements.pmu.imt + config.Noise.pmu.im * randn(size(true_measurements.pmu.imt));

end

