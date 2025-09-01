function [Constraints, Objective, optimization_vars] = buildAttackModel(mpc, attacker_measurements, pmu_config, attack_params, attacker_measurement_map, z_vector)
% buildAttackModel: 构建用于虚假数据注入攻击的YALMIP优化模型
%
% 输入:
%   mpc                      - MATPOWER案例结构体
%   attacker_measurements    - 攻击者已知的测量值
%   pmu_config               - PMU配置信息
%   attack_params            - 攻击参数 (目标线路, 过载率等)
%   attacker_measurement_map - 攻击者已知测量的映射表
%   z_vector                 - 向量化的测量值
%
% 输出:
%   Constraints           - YALMIP约束条件
%   Objective             - YALMIP目标函数
%   optimization_vars     - 包含所有sdpvar变量的结构体

%% --- 1. 初始化与参数提取 ---
define_constants;
[baseMVA, bus, gen, branch] = deal(mpc.baseMVA, mpc.bus, mpc.gen, mpc.branch);
num_buses = size(bus, 1);
num_branches = size(branch, 1);
[ref_bus_id, ~, ~] = bustypes(bus, gen);
[Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);
branch_from_bus = branch(:, F_BUS);
branch_to_bus = branch(:, T_BUS);

pmu_locations = pmu_config.locations;
pmu_from_branch_indices = pmu_config.pmu_from_branch_indices;
pmu_to_branch_indices = pmu_config.pmu_to_branch_indices;
num_measurements = length(z_vector);

%% --- 2. 定义优化变量 (sdpvar) ---
V_compromised = sdpvar(num_buses, 1);       % 攻击后的电压幅值
theta_compromised = sdpvar(num_buses, 1);   % 攻击后的电压相角 (弧度)
P_inj_compromised = sdpvar(num_buses, 1);   % 节点注入有功
Q_inj_compromised = sdpvar(num_buses, 1);   % 节点注入无功
P_from_compromised = sdpvar(num_branches, 1); % 线路from端有功潮流
Q_from_compromised = sdpvar(num_branches, 1); % 线路from端无功潮流
P_to_compromised = sdpvar(num_branches, 1);   % 线路to端有功潮流
Q_to_compromised = sdpvar(num_branches, 1);   % 线路to端无功潮流
attack_vector_a = sdpvar(num_measurements, 1);

optimization_vars = struct(...
    'V_compromised', V_compromised, ...
    'theta_compromised', theta_compromised, ...
    'P_inj_compromised', P_inj_compromised, ...
    'Q_inj_compromised', Q_inj_compromised, ...
    'P_from_compromised', P_from_compromised, ...
    'Q_from_compromised', Q_from_compromised, ...
    'P_to_compromised', P_to_compromised, ...
    'Q_to_compromised', Q_to_compromised, ...
    'attack_vector_a', attack_vector_a ...
);

%% --- 3. 定义目标函数 ---
true_v_mag = attacker_measurements.scada.v;
lambda = attack_params.VoltageDeviationPenalty;
voltage_deviation = norm(V_compromised - true_v_mag, 2);
Objective = norm(attack_vector_a, 1) + lambda * voltage_deviation;

%% --- 4. 构建约束条件 ---
Constraints = [];

% 4.1 物理规律约束 (潮流方程)
V_c_complex = V_compromised .* exp(1j * theta_compromised);
S_inj_c = V_c_complex .* conj(Y_bus * V_c_complex);
Constraints = [Constraints, (P_inj_compromised == real(S_inj_c))];
Constraints = [Constraints, (Q_inj_compromised == imag(S_inj_c))];

S_from_c = V_c_complex(branch_from_bus) .* conj(Y_from * V_c_complex);
Constraints = [Constraints, (P_from_compromised == real(S_from_c))];
Constraints = [Constraints, (Q_from_compromised == imag(S_from_c))];

S_to_c = V_c_complex(branch_to_bus) .* conj(Y_to * V_c_complex);
Constraints = [Constraints, (P_to_compromised == real(S_to_c))];
Constraints = [Constraints, (Q_to_compromised == imag(S_to_c))];

% 4.2 隐蔽性约束 (h(x_c) = z + a)
current_row = 1;
for i = 1:length(attacker_measurement_map)
    map_item = attacker_measurement_map{i};
    count = map_item.count;
    indices = current_row : (current_row + count - 1);
    idx = [];
    if isfield(map_item, 'indices') && ~isempty(map_item.indices)
        idx = map_item.indices(:);
    end

    h_c = []; % h(x_c)
    switch map_item.field
        case 'v',    h_c = V_compromised(idx);
        case 'pi',   h_c = P_inj_compromised(idx);
        case 'qi',   h_c = Q_inj_compromised(idx);
        case 'pf',   h_c = P_from_compromised(idx);
        case 'qf',   h_c = Q_from_compromised(idx);
        case 'pt',   h_c = P_to_compromised(idx);
        case 'qt',   h_c = Q_to_compromised(idx);
        case 'v_real',  h_c = V_compromised(idx) .* cos(theta_compromised(idx));
        case 'v_imag',  h_c = V_compromised(idx) .* sin(theta_compromised(idx));
        case 'if_real', h_full = real(Y_from * V_c_complex); h_c = h_full(idx);
        case 'if_imag', h_full = imag(Y_from * V_c_complex); h_c = h_full(idx);
        case 'it_real', h_full = real(Y_to   * V_c_complex); h_c = h_full(idx);
        case 'it_imag', h_full = imag(Y_to   * V_c_complex); h_c = h_full(idx);
    end
    
    Constraints = [Constraints, (h_c - z_vector(indices) == attack_vector_a(indices))];
    current_row = current_row + count;
end

% 4.3 运行约束
Constraints = [Constraints, (attack_params.VoltageMin <= V_compromised <= attack_params.VoltageMax)];
Constraints = [Constraints, (theta_compromised(ref_bus_id) == bus(ref_bus_id, VA) * pi / 180)];
% Note: The original 'V_compromised(ref_bus_id) == system_measurements.scada.v(ref_bus_id)' constraint
% requires 'system_measurements', which is not passed to this function.
% It should be 'attacker_measurements' instead if the attacker knows the slack bus voltage.
% Assuming the slack bus voltage is known and available in attacker_measurements.
Constraints = [Constraints, (V_compromised(ref_bus_id) == attacker_measurements.scada.v(ref_bus_id))];

% 4.4 发电机约束
gen_bus_indices = gen(:, GEN_BUS);
Constraints = [Constraints, (gen(:, PMIN) / baseMVA <= P_inj_compromised(gen_bus_indices) <= gen(:, PMAX) / baseMVA)];

% 4.5 自定义系统约束 (动态加载)
if isfield(attack_params, 'ConstraintFile') && ~isempty(attack_params.ConstraintFile)
    constraint_func_name = attack_params.ConstraintFile;
    if exist(constraint_func_name, 'file') == 2 % 确保是 .m 文件
        constraint_func = str2func(constraint_func_name);
        Constraints = [Constraints, constraint_func(P_inj_compromised, mpc)];
        fprintf('    - 已加载自定义约束: %s\n', constraint_func_name);
    else
        % 如果配置文件指定了约束文件但未找到，可以选择性地发出警告
        % fprintf('警告: 在配置中指定的约束文件 "%s" 未找到。\n', constraint_func_name);
    end
end

% 4.6 攻击目标约束
target_line_indices = attack_params.attacked_lines_indices;
overload_factor = attack_params.OverloadFactor;
line_capacity_matrix = attack_params.line_capacity_data.Ptrmax57;

for i = 1:length(target_line_indices)
    idx = target_line_indices(i);
    from_bus = branch_from_bus(idx);
    to_bus = branch_to_bus(idx);
    capacity = line_capacity_matrix(from_bus, to_bus);
    if capacity > 0
        Constraints = [Constraints, (P_from_compromised(idx)^2 + Q_from_compromised(idx)^2 >= (overload_factor * capacity)^2)];
    end
end

end
