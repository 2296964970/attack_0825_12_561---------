function [Constraints, Objective, optimization_vars] = buildAttackModel(mpc, selectedModel, known_mask, y_known, V_ref, attack_params)
% buildAttackModel（重构版）: 基于 registry/known_mask 构建隐蔽 FDIA 的 YALMIP 模型
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
    registry = selectedModel.registry;
    idx_known = find(known_mask);
    num_known = numel(idx_known);

    % --- sdpvar 变量 ---
    V_compromised      = sdpvar(num_buses, 1);
    theta_compromised  = sdpvar(num_buses, 1);
    P_inj_compromised  = sdpvar(num_buses, 1);
    Q_inj_compromised  = sdpvar(num_buses, 1);
    P_from_compromised = sdpvar(num_branches, 1);
    Q_from_compromised = sdpvar(num_branches, 1);
    P_to_compromised   = sdpvar(num_branches, 1);
    Q_to_compromised   = sdpvar(num_branches, 1);
    attack_vector_a    = sdpvar(num_known, 1);

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
    lambda = attack_params.VoltageDeviationPenalty;
    Objective = norm(attack_vector_a, 1) + lambda * norm(V_compromised - V_ref, 2);

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

    % 2) 隐蔽性: 在“已知行”上满足 h(x_c) = y_known + a
    I_from = Y_from * Vc; I_to = Y_to * Vc;
    for k = 1:num_known
        r = registry(idx_known(k));
        switch r.domain
            case 'SCADA'
                switch r.type
                    case 'v',  h = V_compromised(r.index);
                    case 'pi', h = P_inj_compromised(r.index);
                    case 'qi', h = Q_inj_compromised(r.index);
                    case 'pf', h = P_from_compromised(r.index);
                    case 'qf', h = Q_from_compromised(r.index);
                    case 'pt', h = P_to_compromised(r.index);
                    case 'qt', h = Q_to_compromised(r.index);
                end
            case 'PMU'
                switch r.type
                    case 'v_real',  h = V_compromised(r.index) .* cos(theta_compromised(r.index));
                    case 'v_imag',  h = V_compromised(r.index) .* sin(theta_compromised(r.index));
                    case 'if_real', h = real(I_from(r.index));
                    case 'if_imag', h = imag(I_from(r.index));
                    case 'it_real', h = real(I_to(r.index));
                    case 'it_imag', h = imag(I_to(r.index));
                end
        end
        Constraints = [Constraints, h - y_known(k) == attack_vector_a(k)];
    end

    % 3) 运行约束 / 参考点
    Constraints = [Constraints, attack_params.VoltageMin <= V_compromised <= attack_params.VoltageMax];
    Constraints = [Constraints, theta_compromised(ref_bus_id) == bus(ref_bus_id, VA) * pi / 180];
    Constraints = [Constraints, V_compromised(ref_bus_id) == V_ref(ref_bus_id)];

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

