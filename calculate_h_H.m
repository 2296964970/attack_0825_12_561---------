function [calculated_measurements, jacobian_matrix] = calculate_h_H(state_vector, mpc, measurement_map, pmu_config, num_state_vars, options)
% calculate_h_H 计算测量函数 h(x) 及其雅可比 H = dh/dx（直角坐标）。
%
% 输入
% - state_vector   : 当前状态向量 x = [e; f_wo_slack]
% - mpc            : MATPOWER 案例结构体
% - measurement_map: 向量化测量的映射描述（type/field/count/indices）
% - pmu_config     : PMU 配置信息
% - num_state_vars : 状态维度（2*n_bus - 1）
% - options        : 包含 slack_bus_id 的结构
%
% 输出
% - calculated_measurements: h(x)
% - jacobian_matrix        : H(x)

define_constants;
[baseMVA, bus, branch] = deal(mpc.baseMVA, mpc.bus, mpc.branch);
num_buses = size(bus, 1);
[Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);
G_bus = real(Y_bus); B_bus = imag(Y_bus);
G_from = real(Y_from); B_from = imag(Y_from);
G_to = real(Y_to);   B_to = imag(Y_to);

slack_bus_id = options.slack_bus_id;
f_indices = setdiff(1:num_buses, slack_bus_id);

% 从 x 还原 e/f
e = state_vector(1:num_buses);
f = zeros(num_buses, 1);
f(f_indices) = state_vector(num_buses+1:end);

% 统计测量总数
num_measurements = 0;
for ii = 1:length(measurement_map)
    if isfield(measurement_map{ii}, 'count') && ~isempty(measurement_map{ii}.count)
        num_measurements = num_measurements + measurement_map{ii}.count;
    end
end

h_parts = {};
jacobian_matrix = zeros(num_measurements, num_state_vars);
row_idx = 1;

% 基础电流表达（节点注入与支路首/末端）
I_real_inj = G_bus * e - B_bus * f;
I_imag_inj = B_bus * e + G_bus * f;
If_real = G_from * e - B_from * f;
If_imag = B_from * e + G_from * f;
It_real = G_to   * e - B_to   * f;
It_imag = B_to   * e + G_to   * f;

branch_from_bus = branch(:, F_BUS);
branch_to_bus   = branch(:, T_BUS);

% 便捷视图
e_from = e(branch_from_bus); f_from = f(branch_from_bus);
e_to   = e(branch_to_bus);   f_to   = f(branch_to_bus);

for seg = 1:length(measurement_map)
    item = measurement_map{seg};
    cnt = item.count;
    if cnt == 0, continue; end
    field = item.field;
    typ   = item.type;
    idx = [];
    if isfield(item, 'indices') && ~isempty(item.indices), idx = item.indices(:); end

    switch typ
        case 'scada'
            switch field
                case 'v'
                    Vmag = hypot(e, f);
                    h_parts{end+1} = Vmag(idx);
                    for t = 1:cnt
                        i = idx(t);
                        Vi = max(Vmag(i), 1e-6);
                        jacobian_matrix(row_idx, i) = e(i) / Vi;
                        if i ~= slack_bus_id
                            f_col_idx = num_buses + find(f_indices == i, 1);
                            jacobian_matrix(row_idx, f_col_idx) = f(i) / Vi;
                        end
                        row_idx = row_idx + 1;
                    end
                case 'pi'
                    Pi = e .* I_real_inj + f .* I_imag_inj;
                    h_parts{end+1} = Pi(idx);
                    dPi_de = diag(I_real_inj) + diag(e) * G_bus + diag(f) * B_bus;
                    dPi_df_full = diag(I_imag_inj) - diag(e) * B_bus + diag(f) * G_bus;
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dPi_de(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dPi_df_full(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'qi'
                    Qi = f .* I_real_inj - e .* I_imag_inj;
                    h_parts{end+1} = Qi(idx);
                    dQi_de = diag(-I_imag_inj) + diag(f) * G_bus - diag(e) * B_bus;
                    dQi_df_full = diag(I_real_inj)  - diag(f) * B_bus - diag(e) * G_bus;
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dQi_de(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dQi_df_full(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'pf'
                    [dP_de, dP_df] = branch_P_block(idx, branch_from_bus, G_from, B_from, e, f, If_real, If_imag, num_buses);
                    h_parts{end+1} = real(e_from .* If_real + f_from .* If_imag);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dP_de;
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dP_df(:, f_indices);
                    row_idx = row_idx + cnt;
                case 'qf'
                    [dQ_de, dQ_df] = branch_Q_block(idx, branch_from_bus, G_from, B_from, e, f, If_real, If_imag, num_buses);
                    h_parts{end+1} = imag((e_from + 1j*f_from) .* (If_real - 1j*If_imag));
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dQ_de;
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dQ_df(:, f_indices);
                    row_idx = row_idx + cnt;
                case 'pt'
                    [dP_de, dP_df] = branch_P_block(idx, branch_to_bus, G_to, B_to, e, f, It_real, It_imag, num_buses);
                    h_parts{end+1} = real(e_to .* It_real + f_to .* It_imag);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dP_de;
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dP_df(:, f_indices);
                    row_idx = row_idx + cnt;
                case 'qt'
                    [dQ_de, dQ_df] = branch_Q_block(idx, branch_to_bus, G_to, B_to, e, f, It_real, It_imag, num_buses);
                    h_parts{end+1} = imag((e_to + 1j*f_to) .* (It_real - 1j*It_imag));
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dQ_de;
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dQ_df(:, f_indices);
                    row_idx = row_idx + cnt;
            end
        case 'pmu'
            switch field
                case 'v_real'
                    h_parts{end+1} = e(idx);
                    for t = 1:cnt
                        bus_k = idx(t);
                        jacobian_matrix(row_idx, bus_k) = 1;
                        row_idx = row_idx + 1;
                    end
                case 'v_imag'
                    h_parts{end+1} = f(idx);
                    for t = 1:cnt
                        bus_k = idx(t);
                        if bus_k ~= slack_bus_id
                            f_col_idx = num_buses + find(f_indices == bus_k, 1);
                            jacobian_matrix(row_idx, f_col_idx) = 1;
                        end
                        row_idx = row_idx + 1;
                    end
                case 'if_real'
                    h_parts{end+1} = If_real(idx);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = G_from(idx,:);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = -B_from(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'if_imag'
                    h_parts{end+1} = If_imag(idx);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = B_from(idx,:);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) =  G_from(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'it_real'
                    h_parts{end+1} = It_real(idx);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = G_to(idx,:);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = -B_to(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'it_imag'
                    h_parts{end+1} = It_imag(idx);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = B_to(idx,:);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) =  G_to(idx, f_indices);
                    row_idx = row_idx + cnt;
            end
    end
end

calculated_measurements = vertcat(h_parts{:});
end

function [dP_de, dP_df] = branch_P_block(kset, branch_bus_side, Gs, Bs, e_all, f_all, Ir, Ii, num_buses)
% 分块计算支路有功潮流对 e/f 的偏导（按首/末端矩阵选择）。
nb = numel(kset);
dP_de = zeros(nb, num_buses);
dP_df = zeros(nb, num_buses);
for t = 1:nb
    k = kset(t);
    i = branch_bus_side(k);
    e_i = e_all(i); f_i = f_all(i);
    row_de = zeros(1, num_buses);
    row_df = zeros(1, num_buses);
    row_de(i) = Ir(k); row_de = row_de + e_i * Gs(k,:) + f_i * Bs(k,:);
    row_df(i) = Ii(k); row_df = row_df - e_i * Bs(k,:) + f_i * Gs(k,:);
    dP_de(t,:) = row_de;
    dP_df(t,:) = row_df;
end
end

function [dQ_de, dQ_df] = branch_Q_block(kset, branch_bus_side, Gs, Bs, e_all, f_all, Ir, Ii, num_buses)
% 分块计算支路无功潮流对 e/f 的偏导（按首/末端矩阵选择）。
nb = numel(kset);
dQ_de = zeros(nb, num_buses);
dQ_df = zeros(nb, num_buses);
for t = 1:nb
    k = kset(t);
    i = branch_bus_side(k);
    e_i = e_all(i); f_i = f_all(i);
    row_de = zeros(1, num_buses);
    row_df = zeros(1, num_buses);
    row_de(i) = -Ii(k); row_de = row_de + f_i * Gs(k,:) - e_i * Bs(k,:);
    row_df(i) =  Ir(k); row_df = row_df - f_i * Bs(k,:) - e_i * Gs(k,:);
    dQ_de(t,:) = row_de;
    dQ_df(t,:) = row_df;
end
end

