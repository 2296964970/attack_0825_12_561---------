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

% --- Wirtinger Calculus Setup for Jacobians ---
V = e + 1j * f;
Ybus = G_bus + 1j * B_bus;

% Nodal Injection Jacobians (for pi, qi)
I_inj = Ybus * V;
dS_inj_dV = diag(conj(I_inj));
dS_inj_dV_conj = diag(V) * conj(Ybus);
dS_inj_de = dS_inj_dV + dS_inj_dV_conj;
dS_inj_df = 1j * (dS_inj_dV - dS_inj_dV_conj);
dPi_de_mat = real(dS_inj_de);
dPi_df_mat = real(dS_inj_df);
dQi_de_mat = imag(dS_inj_de);
dQi_df_mat = imag(dS_inj_df);

% Branch Flow Jacobians (for pf, qf, pt, qt)
num_branches = size(branch, 1);
Yf = G_from + 1j * B_from;
Yt = G_to   + 1j * B_to;

% "From" end Jacobians
If = Yf * V;
dSf_dV = sparse(1:num_branches, branch_from_bus, conj(If), num_branches, num_buses);
dSf_dV_conj = diag(V(branch_from_bus)) * conj(Yf);
dSf_de = dSf_dV + dSf_dV_conj;
dSf_df = 1j * (dSf_dV - dSf_dV_conj);
dPf_de_mat = real(dSf_de);
dPf_df_mat = real(dSf_df);
dQf_de_mat = imag(dSf_de);
dQf_df_mat = imag(dSf_df);

% "To" end Jacobians
It = Yt * V;
dSt_dV = sparse(1:num_branches, branch_to_bus, conj(It), num_branches, num_buses);
dSt_dV_conj = diag(V(branch_to_bus)) * conj(Yt);
dSt_de = dSt_dV + dSt_dV_conj;
dSt_df = 1j * (dSt_dV - dSt_dV_conj);
dPt_de_mat = real(dSt_de);
dPt_df_mat = real(dSt_df);
dQt_de_mat = imag(dSt_de);
dQt_df_mat = imag(dSt_df);
% --- End Wirtinger Setup ---

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
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dPi_de_mat(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dPi_df_mat(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'qi'
                    Qi = f .* I_real_inj - e .* I_imag_inj;
                    h_parts{end+1} = Qi(idx);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dQi_de_mat(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dQi_df_mat(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'pf'
                    h_parts{end+1} = e_from(idx) .* If_real(idx) + f_from(idx) .* If_imag(idx);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dPf_de_mat(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dPf_df_mat(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'qf'
                    h_parts{end+1} = f_from(idx) .* If_real(idx) - e_from(idx) .* If_imag(idx);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dQf_de_mat(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dQf_df_mat(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'pt'
                    h_parts{end+1} = e_to(idx) .* It_real(idx) + f_to(idx) .* It_imag(idx);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dPt_de_mat(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dPt_df_mat(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'qt'
                    h_parts{end+1} = f_to(idx) .* It_real(idx) - e_to(idx) .* It_imag(idx);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dQt_de_mat(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dQt_df_mat(idx, f_indices);
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



