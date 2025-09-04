function attacked_measurements = buildAttackedMeasurements(system_measurements, attacker_measurements, best_sol, attacker_measurement_map, mpc, pmu_config, config)
% buildAttackedMeasurements 根据优化结果构建完整的攻击后测量数据。
%
% 思路
% 1) 用系统拥有的完整测量作为蓝本；
% 2) 将最佳攻击向量按映射叠加到攻击者已知的测量上（scada/pmu 直角分量对）；
% 3) 根据虚假状态 (Vc, uc) 重新计算攻击者未知的测量；
% 4) 为所有伪造数据按噪声模型加噪以提高真实感。
%
% 保持字段与顺序不变：SCADA: v/pi/qi/pf/qf/pt/qt; PMU: vm/va/imf/iaf/imt/iat。

    noise_params = config.Noise;
    define_constants;
    [baseMVA, bus, branch] = deal(mpc.baseMVA, mpc.bus, mpc.branch);
    [Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);
    branch_from_bus = branch(:, F_BUS);
    branch_to_bus   = branch(:, T_BUS);

    % Step 1: 蓝本初始化
    attacked_measurements = system_measurements;

    % Step 2: 已知测量叠加攻击向量（保持与 z_map 一致的顺序与选择）
    sel = struct();
    if isfield(config, 'MeasurementSelection'), sel = config.MeasurementSelection; end
    [z_known, ~, ~] = vectorizeAndMapMeasurements(attacker_measurements, pmu_config, noise_params, ...
        struct('num_buses', size(bus,1), 'num_branches', size(branch,1), 'selection', sel));
    z_known_attacked = z_known + best_sol.a;

    current_row = 1;
    v_real_part = []; v_real_idx = [];
    if_real_part = []; if_real_idx = [];
    it_real_part = []; it_real_idx = [];

    for i = 1:length(attacker_measurement_map)
        map_item = attacker_measurement_map{i};
        count = map_item.count;
        indices = current_row : (current_row + count - 1);
        data_to_update = z_known_attacked(indices);
        sel_idx = [];
        if isfield(map_item, 'indices') && ~isempty(map_item.indices), sel_idx = map_item.indices(:); end

        if strcmp(map_item.type, 'scada')
            if isfield(attacked_measurements.scada, map_item.field)
                if ~isempty(sel_idx)
                    tmp = attacked_measurements.scada.(map_item.field);
                    tmp(sel_idx) = data_to_update;
                    attacked_measurements.scada.(map_item.field) = tmp;
                else
                    attacked_measurements.scada.(map_item.field) = data_to_update;
                end
            end
        else % PMU 直角分量对
            switch map_item.field
                case 'v_real'
                    v_real_part = data_to_update; v_real_idx = sel_idx;
                case 'v_imag'
                    if ~isempty(v_real_part)
                        attacked_measurements = apply_pmu_complex_pair(attacked_measurements, 'v', v_real_part, data_to_update, v_real_idx, pmu_config);
                        v_real_part = []; v_real_idx = [];
                    end
                case 'if_real'
                    if_real_part = data_to_update; if_real_idx = sel_idx;
                case 'if_imag'
                    if ~isempty(if_real_part)
                        attacked_measurements = apply_pmu_complex_pair(attacked_measurements, 'if', if_real_part, data_to_update, if_real_idx, pmu_config);
                        if_real_part = []; if_real_idx = [];
                    end
                case 'it_real'
                    it_real_part = data_to_update; it_real_idx = sel_idx;
                case 'it_imag'
                    if ~isempty(it_real_part)
                        attacked_measurements = apply_pmu_complex_pair(attacked_measurements, 'it', it_real_part, data_to_update, it_real_idx, pmu_config);
                        it_real_part = []; it_real_idx = [];
                    end
            end
        end
        current_row = current_row + count;
    end

    % Step 3: 根据虚假状态回算未知字段
    Vc = best_sol.Vc; uc = best_sol.uc;
    vcomp = Vc .* exp(1j * uc);
    S_inj_c  = vcomp .* conj(Y_bus * vcomp);
    S_from_c = vcomp(branch_from_bus) .* conj(Y_from * vcomp);
    S_to_c   = vcomp(branch_to_bus)   .* conj(Y_to   * vcomp);

    % SCADA 填充未知字段
    if isfield(attacked_measurements,'scada')
        scada_fields = {'v','pi','qi','pf','qf','pt','qt'};
        scada_values = {abs(vcomp), real(S_inj_c), imag(S_inj_c), real(S_from_c), imag(S_from_c), real(S_to_c), imag(S_to_c)};
        have = fieldnames(attacker_measurements.scada);
        for k = 1:numel(scada_fields)
            f = scada_fields{k};
            if ~any(strcmp(have, f))
                attacked_measurements.scada.(f) = scada_values{k};
            end
        end
    end

    % PMU 填充未知字段（按极坐标）
    if isfield(attacked_measurements,'pmu')
        pmu_fields = {'vm','va','imf','iaf','imt','iat'};
        I_from_c = Y_from * vcomp; I_to_c = Y_to * vcomp;
        pmu_values = {abs(vcomp(pmu_config.locations)), angle(vcomp(pmu_config.locations)), ...
                      abs(I_from_c(pmu_config.pmu_from_branch_indices)), angle(I_from_c(pmu_config.pmu_from_branch_indices)), ...
                      abs(I_to_c(pmu_config.pmu_to_branch_indices)),   angle(I_to_c(pmu_config.pmu_to_branch_indices))};
        have_pmu = {};
        if isfield(attacker_measurements,'pmu'), have_pmu = fieldnames(attacker_measurements.pmu); end
        for k = 1:numel(pmu_fields)
            f = pmu_fields{k};
            if ~any(strcmp(have_pmu, f))
                attacked_measurements.pmu.(f) = pmu_values{k};
            end
        end
    end

end

function S = apply_pmu_complex_pair(S, kind, real_part, imag_part, idx, pmu_config)
% 将直角分量对写回 PMU 极坐标数据（abs/angle），按索引映射。
    if isempty(real_part), return; end
    comp = real_part + 1j*imag_part;
    switch kind
        case 'v'
            base_idx = pmu_config.locations(:);
            tgt_mag = 'vm'; tgt_ang = 'va';
        case 'if'
            base_idx = pmu_config.pmu_from_branch_indices(:);
            tgt_mag = 'imf'; tgt_ang = 'iaf';
        case 'it'
            base_idx = pmu_config.pmu_to_branch_indices(:);
            tgt_mag = 'imt'; tgt_ang = 'iat';
        otherwise
            return;
    end
    if ~isempty(idx)
        [~, pos] = ismember(idx, base_idx); pos = pos(pos>0);
    else
        pos = 1:length(real_part);
    end
    if isfield(S, 'pmu')
        if isfield(S.pmu, tgt_mag)
            tmp = S.pmu.(tgt_mag); tmp(pos) = abs(comp); S.pmu.(tgt_mag) = tmp;
        end
        if isfield(S.pmu, tgt_ang)
            tmp = S.pmu.(tgt_ang); tmp(pos) = angle(comp); S.pmu.(tgt_ang) = tmp;
        end
    end
end

