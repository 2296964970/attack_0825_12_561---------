  function [calculated_measurements, jacobian_matrix] = calculate_h_H(state_vector, mpc, measurement_map, pmu_config, num_state_vars, options)
%CALCULATE_H_H čŽĄçŽWLSçśćäź°čŽĄä¸­çćľéĺ˝ć°h(x)ĺéĺŻćŻçŠéľH
%
%   ć­¤ĺ˝ć°ĺ°čŁäşä¸şćˇˇĺSCADA/PMUçśćäź°čŽĄčŽĄçŽh(x)ĺHçĺ¤ćéťčžă
%
%   čžĺĽ:
%     state_vector           - ĺ˝ĺçśćĺé [e; f] (ç´č§ĺć )
%     mpc                    - MATPOWERćĄäžďźç¨äşčˇĺçľç˝ĺć°
%     measurement_map        - çąĺéĺéśćŽľçćçć ĺ°čĄ¨ďźćŽľ+indicesďź
%     pmu_config             - PMUéç˝ŽďźĺŚĺŽčŁä˝ç˝ŽĺćŻčˇŻç´˘ĺź
%     num_state_vars         - çśćĺéxçćťçť´ĺşŚ (2*n-1)
%     options                - ĺĺŤslack_bus_idç­čŽžç˝Žççťćä˝
%
%   čžĺş:
%     calculated_measurements - čŽĄçŽĺžĺ°çćľéĺé h(x)
%     jacobian_matrix         - éĺŻćŻçŠéľ H = dh/dx

define_constants;

% --- 1. äťMPCĺçśćĺéä¸­ćĺĺć° ---
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

% --- 1b. çť´ĺşŚä¸č´ć§ä¸ćľéčŽĄć°ćŁćĽ ---
% çąć ĺ°čĄ¨ç´Żĺ ĺžĺ°ćľéćťć°
num_measurements = 0;
for ii = 1:length(measurement_map)
    if isfield(measurement_map{ii}, 'count') && ~isempty(measurement_map{ii}.count)
        num_measurements = num_measurements + measurement_map{ii}.count;
    end
end

% --- 2. ĺĺ§ĺčžĺş ---
h_parts = {};
jacobian_matrix = zeros(num_measurements, num_state_vars);
row_idx = 1;

% --- 3. ĺŹĺąčŽĄçŽ: ćł¨ĺĽçľćľĺćŻčˇŻçľćľ ---
I_real_inj = G_bus * current_voltage_real - B_bus * current_voltage_imag;
I_imag_inj = B_bus * current_voltage_real + G_bus * current_voltage_imag;
If_real = G_from * current_voltage_real - B_from * current_voltage_imag;
If_imag = B_from * current_voltage_real + G_from * current_voltage_imag;
It_real = G_to * current_voltage_real - B_to * current_voltage_imag;
It_imag = B_to * current_voltage_real + G_to * current_voltage_imag;

branch_from_bus = branch(:, F_BUS);
branch_to_bus = branch(:, T_BUS);


% --- 4. ĺşäş measurement_map ćé  h/H ---
e_from = current_voltage_real(branch_from_bus); f_from = current_voltage_imag(branch_from_bus);
e_to = current_voltage_real(branch_to_bus);   f_to = current_voltage_imag(branch_to_bus);

for seg = 1:length(measurement_map)
    item = measurement_map{seg};
    cnt = item.count;
    if cnt == 0, continue; end
    field = item.field; typ = item.type;
    idx = [];
    if isfield(item, 'indices') && ~isempty(item.indices), idx = item.indices(:); end

    switch typ
        case 'scada'
            switch field
                case 'v'
                    Vmag = sqrt(current_voltage_real.^2 + current_voltage_imag.^2);
                    h_parts{end+1} = Vmag(idx);
                    for t = 1:cnt
                        i = idx(t);
                        Vi = max(Vmag(i), 1e-6);
                        jacobian_matrix(row_idx, i) = current_voltage_real(i) / Vi;
                        if i ~= slack_bus_id
                            f_col_idx = num_buses + find(f_indices == i, 1);
                            jacobian_matrix(row_idx, f_col_idx) = current_voltage_imag(i) / Vi;
                        end
                        row_idx = row_idx + 1;
                    end
                case 'pi'
                    Pi = current_voltage_real .* I_real_inj + current_voltage_imag .* I_imag_inj;
                    h_parts{end+1} = Pi(idx);
                    dPi_de = diag(I_real_inj) + diag(current_voltage_real) * G_bus + diag(current_voltage_imag) * B_bus;
                    dPi_df_full = diag(I_imag_inj) - diag(current_voltage_real) * B_bus + diag(current_voltage_imag) * G_bus;
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dPi_de(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dPi_df_full(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'qi'
                    Qi = current_voltage_imag .* I_real_inj - current_voltage_real .* I_imag_inj;
                    h_parts{end+1} = Qi(idx);
                    dQi_de = diag(-I_imag_inj) + diag(current_voltage_imag) * G_bus - diag(current_voltage_real) * B_bus;
                    dQi_df_full = diag(I_real_inj) - diag(current_voltage_imag) * B_bus - diag(current_voltage_real) * G_bus;
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dQi_de(idx, :);
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dQi_df_full(idx, f_indices);
                    row_idx = row_idx + cnt;
                case 'pf'
                    Pf = e_from .* If_real + f_from .* If_imag;
                    h_parts{end+1} = Pf(idx);
                    [dP_de, dP_df] = branch_P_block(idx, branch_from_bus, G_from, B_from, current_voltage_real, current_voltage_imag, If_real, If_imag, num_buses);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dP_de;
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dP_df(:, f_indices);
                    row_idx = row_idx + cnt;
                case 'qf'
                    Qf = f_from .* If_real - e_from .* If_imag;
                    h_parts{end+1} = Qf(idx);
                    [dQ_de, dQ_df] = branch_Q_block(idx, branch_from_bus, G_from, B_from, current_voltage_real, current_voltage_imag, If_real, If_imag, num_buses);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dQ_de;
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dQ_df(:, f_indices);
                    row_idx = row_idx + cnt;
                case 'pt'
                    Pt = e_to .* It_real + f_to .* It_imag;
                    h_parts{end+1} = Pt(idx);
                    [dP_de, dP_df] = branch_P_block(idx, branch_to_bus, G_to, B_to, current_voltage_real, current_voltage_imag, It_real, It_imag, num_buses);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dP_de;
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dP_df(:, f_indices);
                    row_idx = row_idx + cnt;
                case 'qt'
                    Qt = f_to .* It_real - e_to .* It_imag;
                    h_parts{end+1} = Qt(idx);
                    [dQ_de, dQ_df] = branch_Q_block(idx, branch_to_bus, G_to, B_to, current_voltage_real, current_voltage_imag, It_real, It_imag, num_buses);
                    jacobian_matrix(row_idx:row_idx+cnt-1, 1:num_buses) = dQ_de;
                    jacobian_matrix(row_idx:row_idx+cnt-1, num_buses+1:end) = dQ_df(:, f_indices);
                    row_idx = row_idx + cnt;
            end
        case 'pmu'
            switch field
                case 'v_real'
                    h_parts{end+1} = current_voltage_real(idx);
                    for t = 1:cnt
                        bus_k = idx(t);
                        jacobian_matrix(row_idx, bus_k) = 1;
                        row_idx = row_idx + 1;
                    end
                case 'v_imag'
                    h_parts{end+1} = current_voltage_imag(idx);
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
% Ö§ÂˇÓĐšŚ˛âÁżľÄŃĹżÉąČżéŁ¨ÖđĐĐ°˛ČŤššÔěŁŠ
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
% Ö§ÂˇÎŢšŚ˛âÁżľÄŃĹżÉąČżéŁ¨ÖđĐĐ°˛ČŤššÔěŁŠ
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

