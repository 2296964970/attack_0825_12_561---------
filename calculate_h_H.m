  function [calculated_measurements, jacobian_matrix] = calculate_h_H(state_vector, mpc, measured_measurements, pmu_config, num_measurements, num_state_vars, options)
%CALCULATE_H_H 计算WLS状态估计中的测量函数h(x)和雅可比矩阵H
%
%   此函数封装了为混合SCADA/PMU状态估计计算h(x)和H的复杂逻辑。
%
%   输入:
%     state_vector           - 当前状态向量 [e; f] (直角坐标)
%     mpc                    - MATPOWER案例，用于获取电网参数
%     measured_measurements  - 包含当前有效测量类型的结构体
%     pmu_config             - PMU配置，如安装位置和支路索引
%     num_measurements       - 测量向量z的总维度
%     num_state_vars         - 状态向量x的总维度 (2*n-1)
%     options                - 包含slack_bus_id等设置的结构体
%
%   输出:
%     calculated_measurements - 计算得到的测量向量 h(x)
%     jacobian_matrix         - 雅可比矩阵 H = dh/dx

define_constants;

% --- 1. 从MPC和状态向量中提取参数 ---
[baseMVA, bus, branch] = deal(mpc.baseMVA, mpc.bus, mpc.branch);
num_buses = size(bus, 1);
[Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);
G_bus = real(Y_bus); B_bus = imag(Y_bus);
G_from = real(Y_from); B_from = imag(Y_from);
G_to = real(Y_to);   B_to = imag(Y_to);

slack_bus_id = options.slack_bus_id;
f_indices = setdiff(1:num_buses, slack_bus_id);

current_voltage_real = state_vector(1:num_buses);
current_voltage_imag = zeros(num_buses, 1);
current_voltage_imag(f_indices) = state_vector(num_buses+1:end);

% --- 2. 初始化输出 ---
h_parts = {};
jacobian_matrix = zeros(num_measurements, num_state_vars);
row_idx = 1;

% --- 3. 公共计算: 注入电流和支路电流 ---
I_real_inj = G_bus * current_voltage_real - B_bus * current_voltage_imag;
I_imag_inj = B_bus * current_voltage_real + G_bus * current_voltage_imag;
If_real = G_from * current_voltage_real - B_from * current_voltage_imag;
If_imag = B_from * current_voltage_real + G_from * current_voltage_imag;
It_real = G_to * current_voltage_real - B_to * current_voltage_imag;
It_imag = B_to * current_voltage_real + G_to * current_voltage_imag;

scada_meas = measured_measurements.scada; % 简化访问
branch_from_bus = branch(:, F_BUS);
branch_to_bus = branch(:, T_BUS);


% --- 4. H 矩阵: SCADA 测量部分 ---
% 1. 电压幅值 V
if isfield(scada_meas, 'v') && ~isempty(scada_meas.v)
    num_v_meas = length(scada_meas.v);
    V_calculated = sqrt(current_voltage_real.^2 + current_voltage_imag.^2);
    h_parts{end+1} = V_calculated;
    for i = 1:num_v_meas
        if V_calculated(i) < 1e-6, V_calculated(i) = 1e-6; end % 避免除以零
        jacobian_matrix(row_idx, i) = current_voltage_real(i) / V_calculated(i);
        if i ~= slack_bus_id
            f_col_idx = num_buses + find(f_indices == i, 1);
            jacobian_matrix(row_idx, f_col_idx) = current_voltage_imag(i) / V_calculated(i);
        end
        row_idx = row_idx + 1;
    end
end

% 2. 注入有功功率 Pi
if isfield(scada_meas, 'pi') && ~isempty(scada_meas.pi)
    num_pi_meas = length(scada_meas.pi);
    Pi_calculated = current_voltage_real .* I_real_inj + current_voltage_imag .* I_imag_inj;
    h_parts{end+1} = Pi_calculated;
    dPi_de = diag(I_real_inj) + diag(current_voltage_real) * G_bus + diag(current_voltage_imag) * B_bus;
    dPi_df_full = diag(I_imag_inj) - diag(current_voltage_real) * B_bus + diag(current_voltage_imag) * G_bus;
    jacobian_matrix(row_idx:row_idx+num_pi_meas-1, 1:num_buses) = dPi_de;
    jacobian_matrix(row_idx:row_idx+num_pi_meas-1, num_buses+1:end) = dPi_df_full(:, f_indices);
    row_idx = row_idx + num_pi_meas;
end

% 3. 注入无功功率 Qi
if isfield(scada_meas, 'qi') && ~isempty(scada_meas.qi)
    num_qi_meas = length(scada_meas.qi);
    Qi_calculated = current_voltage_imag .* I_real_inj - current_voltage_real .* I_imag_inj;
    h_parts{end+1} = Qi_calculated;
    dQi_de = diag(-I_imag_inj) + diag(current_voltage_imag) * G_bus - diag(current_voltage_real) * B_bus;
    dQi_df_full = diag(I_real_inj) - diag(current_voltage_imag) * B_bus - diag(current_voltage_real) * G_bus;
    jacobian_matrix(row_idx:row_idx+num_qi_meas-1, 1:num_buses) = dQi_de;
    jacobian_matrix(row_idx:row_idx+num_qi_meas-1, num_buses+1:end) = dQi_df_full(:, f_indices);
    row_idx = row_idx + num_qi_meas;
end

e_from = current_voltage_real(branch_from_bus); f_from = current_voltage_imag(branch_from_bus);
e_to = current_voltage_real(branch_to_bus);   f_to = current_voltage_imag(branch_to_bus);

% 4. From-end 有功功率 Pf
if isfield(scada_meas, 'pf') && ~isempty(scada_meas.pf)
    num_pf_meas = length(scada_meas.pf);
    Pf_calculated = e_from .* If_real + f_from .* If_imag;
    h_parts{end+1} = Pf_calculated;
    for k = 1:num_pf_meas
        i = branch_from_bus(k); % from bus index
        e_i = current_voltage_real(i);
        f_i = current_voltage_imag(i);
        
        % Derivative w.r.t. e (dPf_k/de)
        dPf_de = zeros(1, num_buses);
        dPf_de(i) = If_real(k); % from product rule: d(e_i)/de_i * If_real
        dPf_de = dPf_de + e_i * G_from(k,:) + f_i * B_from(k,:);
        
        % Derivative w.r.t. f (dPf_k/df)
        dPf_df = zeros(1, num_buses);
        dPf_df(i) = If_imag(k); % from product rule: d(f_i)/df_i * If_imag
        dPf_df = dPf_df - e_i * B_from(k,:) + f_i * G_from(k,:);
        
        jacobian_matrix(row_idx, 1:num_buses) = dPf_de;
        jacobian_matrix(row_idx, num_buses+1:end) = dPf_df(f_indices);
        row_idx = row_idx + 1;
    end
end

% 5. From-end 无功功率 Qf
if isfield(scada_meas, 'qf') && ~isempty(scada_meas.qf)
    num_qf_meas = length(scada_meas.qf);
    Qf_calculated = f_from .* If_real - e_from .* If_imag;
    h_parts{end+1} = Qf_calculated;
    for k = 1:num_qf_meas
        i = branch_from_bus(k);
        e_i = current_voltage_real(i);
        f_i = current_voltage_imag(i);

        % Derivative w.r.t. e (dQf_k/de)
        dQf_de = zeros(1, num_buses);
        dQf_de(i) = -If_imag(k);
        dQf_de = dQf_de + f_i * G_from(k,:) - e_i * B_from(k,:);

        % Derivative w.r.t. f (dQf_k/df)
        dQf_df = zeros(1, num_buses);
        dQf_df(i) = If_real(k);
        dQf_df = dQf_df - f_i * B_from(k,:) - e_i * G_from(k,:);

        jacobian_matrix(row_idx, 1:num_buses) = dQf_de;
        jacobian_matrix(row_idx, num_buses+1:end) = dQf_df(f_indices);
        row_idx = row_idx + 1;
    end
end

% 6. To-end 有功功率 Pt
if isfield(scada_meas, 'pt') && ~isempty(scada_meas.pt)
    num_pt_meas = length(scada_meas.pt);
    Pt_calculated = e_to .* It_real + f_to .* It_imag;
    h_parts{end+1} = Pt_calculated;
    for k = 1:num_pt_meas
        i = branch_to_bus(k); % to bus index
        e_i = current_voltage_real(i);
        f_i = current_voltage_imag(i);
        
        % Derivative w.r.t. e (dPt_k/de)
        dPt_de = zeros(1, num_buses);
        dPt_de(i) = It_real(k);
        dPt_de = dPt_de + e_i * G_to(k,:) + f_i * B_to(k,:);
        
        % Derivative w.r.t. f (dPt_k/df)
        dPt_df = zeros(1, num_buses);
        dPt_df(i) = It_imag(k);
        dPt_df = dPt_df - e_i * B_to(k,:) + f_i * G_to(k,:);
        
        jacobian_matrix(row_idx, 1:num_buses) = dPt_de;
        jacobian_matrix(row_idx, num_buses+1:end) = dPt_df(f_indices);
        row_idx = row_idx + 1;
    end
end

% 7. To-end 无功功率 Qt
if isfield(scada_meas, 'qt') && ~isempty(scada_meas.qt)
    num_qt_meas = length(scada_meas.qt);
    Qt_calculated = f_to .* It_real - e_to .* It_imag;
    h_parts{end+1} = Qt_calculated;
    for k = 1:num_qt_meas
        i = branch_to_bus(k);
        e_i = current_voltage_real(i);
        f_i = current_voltage_imag(i);

        % Derivative w.r.t. e (dQt_k/de)
        dQt_de = zeros(1, num_buses);
        dQt_de(i) = -It_imag(k);
        dQt_de = dQt_de + f_i * G_to(k,:) - e_i * B_to(k,:);

        % Derivative w.r.t. f (dQt_k/df) - *** THIS IS THE CORRECTED PART ***
        dQt_df = zeros(1, num_buses);
        dQt_df(i) = It_real(k);
        dQt_df = dQt_df - f_i * B_to(k,:) - e_i * G_to(k,:);

        jacobian_matrix(row_idx, 1:num_buses) = dQt_de;
        jacobian_matrix(row_idx, num_buses+1:end) = dQt_df(f_indices);
        row_idx = row_idx + 1;
    end
end

% --- 5. H 矩阵: PMU 伪测量部分 (线性) ---
if isfield(measured_measurements, 'pmu') && ~isempty(fieldnames(measured_measurements.pmu))
    pmu = measured_measurements.pmu;
    pmu_locs = pmu_config.locations;

    % 1. 电压实部伪测量 e (vm存在时)
    if isfield(pmu, 'vm') && ~isempty(pmu.vm)
        h_parts{end+1} = current_voltage_real(pmu_locs);
        for i = 1:length(pmu_locs), bus_k=pmu_locs(i); jacobian_matrix(row_idx,bus_k)=1; row_idx=row_idx+1; end
    end
    % 1b. 电压虚部伪测量 f (va存在时)
    if isfield(pmu, 'va') && ~isempty(pmu.va)
        h_parts{end+1} = current_voltage_imag(pmu_locs);
        for i = 1:length(pmu_locs), bus_k=pmu_locs(i); if bus_k~=slack_bus_id, f_col_idx=num_buses+find(f_indices==bus_k,1); jacobian_matrix(row_idx,f_col_idx)=1; end; row_idx=row_idx+1; end
    end
    % 2. From-end 电流实部伪测量 I_f_real (imf存在时)
    if isfield(pmu, 'imf') && ~isempty(pmu.imf)
        pmu_from_idx = pmu_config.pmu_from_branch_indices; num_imf = length(pmu_from_idx);
        h_parts{end+1} = If_real(pmu_from_idx);
        jacobian_matrix(row_idx:row_idx+num_imf-1, 1:num_buses)=G_from(pmu_from_idx,:); jacobian_matrix(row_idx:row_idx+num_imf-1, num_buses+1:end)=-B_from(pmu_from_idx,f_indices); row_idx=row_idx+num_imf;
    end
    % 2b. From-end 电流虚部伪测量 I_f_imag (iaf存在时)
    if isfield(pmu, 'iaf') && ~isempty(pmu.iaf)
        pmu_from_idx = pmu_config.pmu_from_branch_indices; num_iaf = length(pmu_from_idx);
        h_parts{end+1} = If_imag(pmu_from_idx);
        jacobian_matrix(row_idx:row_idx+num_iaf-1, 1:num_buses)=B_from(pmu_from_idx,:); jacobian_matrix(row_idx:row_idx+num_iaf-1, num_buses+1:end)=G_from(pmu_from_idx,f_indices); row_idx=row_idx+num_iaf;
    end
    % 3. To-end 电流实部伪测量 I_t_real (imt存在时)
    if isfield(pmu, 'imt') && ~isempty(pmu.imt)
        pmu_to_idx = pmu_config.pmu_to_branch_indices; num_imt = length(pmu_to_idx);
        h_parts{end+1} = It_real(pmu_to_idx);
        jacobian_matrix(row_idx:row_idx+num_imt-1, 1:num_buses)=G_to(pmu_to_idx,:); jacobian_matrix(row_idx:row_idx+num_imt-1, num_buses+1:end)=-B_to(pmu_to_idx,f_indices); row_idx=row_idx+num_imt;
    end
    % 3b. To-end 电流虚部伪测量 I_t_imag (iat存在时)
    if isfield(pmu, 'iat') && ~isempty(pmu.iat)
        pmu_to_idx = pmu_config.pmu_to_branch_indices; num_iat = length(pmu_to_idx);
        h_parts{end+1} = It_imag(pmu_to_idx);
        jacobian_matrix(row_idx:row_idx+num_iat-1, 1:num_buses)=B_to(pmu_to_idx,:); jacobian_matrix(row_idx:row_idx+num_iat-1, num_buses+1:end)=G_to(pmu_to_idx,f_indices); row_idx=row_idx+num_iat;
    end
end

calculated_measurements = vertcat(h_parts{:});
end