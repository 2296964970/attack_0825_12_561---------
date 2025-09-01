function [Constraints, Objective, optimization_vars] = buildAttackModel(mpc, attacker_measurements, pmu_config, attack_params, attacker_measurement_map, z_vector)
% buildAttackModel 构建用于隐蔽 FDIA 的 YALMIP 优化模型。
%
% 输入
% - mpc                      : MATPOWER 案例结构体
% - attacker_measurements    : 攻击者已知的测量（用于一致性约束）
% - pmu_config               : PMU 布局与索引
% - attack_params            : 攻击参数（目标线路/过载率/电压惩罚等）
% - attacker_measurement_map : 已知测量的映射（type/field/count/indices）
% - z_vector                 : 已知测量的向量化值 z
%
% 输出
% - Constraints              : 约束集合
% - Objective                : 目标函数
% - optimization_vars        : sdpvar 变量集合

define_constants;
[baseMVA, bus, gen, branch] = deal(mpc.baseMVA, mpc.bus, mpc.gen, mpc.branch);
num_buses = size(bus, 1);
num_branches = size(branch, 1);
[ref_bus_id, ~, ~] = bustypes(bus, gen);
[Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);
branch_from_bus = branch(:, F_BUS);
branch_to_bus   = branch(:, T_BUS);

pmu_locations = pmu_config.locations;
pmu_from_branch_indices = pmu_config.pmu_from_branch_indices;
pmu_to_branch_indices   = pmu_config.pmu_to_branch_indices;
num_measurements = length(z_vector);

% --- sdpvar 变量 ---
V_compromised    = sdpvar(num_buses, 1);
theta_compromised= sdpvar(num_buses, 1);
P_inj_compromised= sdpvar(num_buses, 1);
Q_inj_compromised= sdpvar(num_buses, 1);
P_from_compromised = sdpvar(num_branches, 1);
Q_from_compromised = sdpvar(num_branches, 1);
P_to_compromised   = sdpvar(num_branches, 1);
Q_to_compromised   = sdpvar(num_branches, 1);
attack_vector_a    = sdpvar(num_measurements, 1);

optimization_vars = struct( ...
    'V_compromised', V_compromised, ...
    'theta_compromised', theta_compromised, ...
    'P_inj_compromised', P_inj_compromised, ...
    'Q_inj_compromised', Q_inj_compromised, ...
    'P_from_compromised', P_from_compromised, ...
    'Q_from_compromised', Q_from_compromised, ...
    'P_to_compromised', P_to_compromised, ...
    'Q_to_compromised', Q_to_compromised, ...
    'attack_vector_a', attack_vector_a);

% --- 目标函数 ---
true_v_mag = attacker_measurements.scada.v;
lambda = attack_params.VoltageDeviationPenalty;
Objective = norm(attack_vector_a, 1) + lambda * norm(V_compromised - true_v_mag, 2);

% --- 约束 ---
Constraints = [];

% 1) 潮流一致性
Vc = V_compromised .* exp(1j * theta_compromised);
S_inj_c  = Vc .* conj(Y_bus * Vc);
S_from_c = Vc(branch_from_bus) .* conj(Y_from * Vc);
S_to_c   = Vc(branch_to_bus)   .* conj(Y_to   * Vc);
Constraints = [Constraints, P_inj_compromised == real(S_inj_c)];
Constraints = [Constraints, Q_inj_compromised == imag(S_inj_c)];
Constraints = [Constraints, P_from_compromised == real(S_from_c)];
Constraints = [Constraints, Q_from_compromised == imag(S_from_c)];
Constraints = [Constraints, P_to_compromised   == real(S_to_c)];
Constraints = [Constraints, Q_to_compromised   == imag(S_to_c)];

% 2) 隐蔽性: h(x_c) = z + a（按映射展开）
current_row = 1;
for i = 1:length(attacker_measurement_map)
    map_item = attacker_measurement_map{i};
    count = map_item.count;
    indices = current_row : (current_row + count - 1);
    idx = [];
    if isfield(map_item, 'indices') && ~isempty(map_item.indices), idx = map_item.indices(:); end

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
        case 'if_real', h_full = real(Y_from * Vc); h_c = h_full(idx);
        case 'if_imag', h_full = imag(Y_from * Vc); h_c = h_full(idx);
        case 'it_real', h_full = real(Y_to   * Vc); h_c = h_full(idx);
        case 'it_imag', h_full = imag(Y_to   * Vc); h_c = h_full(idx);
        otherwise, h_c = []; % 不应出现
    end
    Constraints = [Constraints, h_c - z_vector(indices) == attack_vector_a(indices)];
    current_row = current_row + count;
end

% 3) 运行约束 / 参考点
Constraints = [Constraints, attack_params.VoltageMin <= V_compromised <= attack_params.VoltageMax];
Constraints = [Constraints, theta_compromised(ref_bus_id) == bus(ref_bus_id, VA) * pi / 180];
Constraints = [Constraints, V_compromised(ref_bus_id) == attacker_measurements.scada.v(ref_bus_id)];

% 4) 发电机有功边界（按注入表达）
gen_bus_indices = gen(:, GEN_BUS);
Constraints = [Constraints, gen(:, PMIN) / baseMVA <= P_inj_compromised(gen_bus_indices) <= gen(:, PMAX) / baseMVA];

% 5) 自定义系统约束（按配置加载）
if isfield(attack_params, 'ConstraintFile') && ~isempty(attack_params.ConstraintFile)
    cf = attack_params.ConstraintFile;
    if exist(cf, 'file') == 2
        Constraints = [Constraints, feval(cf, P_inj_compromised, mpc)];
        fprintf('    - 已加载自定义约束: %s\n', cf);
    end
end

% 6) 攻击目标（过载约束）
target_line_indices = attack_params.attacked_lines_indices;
overload_factor = attack_params.OverloadFactor;
line_capacity_matrix = attack_params.line_capacity_data.Ptrmax57; % p.u.
for i = 1:length(target_line_indices)
    idx = target_line_indices(i);
    from_bus = branch_from_bus(idx);
    to_bus   = branch_to_bus(idx);
    capacity = line_capacity_matrix(from_bus, to_bus);
    if capacity > 0
        Constraints = [Constraints, P_from_compromised(idx)^2 + Q_from_compromised(idx)^2 >= (overload_factor * capacity)^2];
    end
end

end

