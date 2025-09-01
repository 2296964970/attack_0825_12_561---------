function attacked_measurements = buildAttackedMeasurements(system_measurements, attacker_measurements, best_sol, attacker_measurement_map, mpc, pmu_config, config)
% buildAttackedMeasurements: 根据优化结果构建完整的攻击后测量数据
%
% 此函数负责将攻击向量应用到已知测量，根据虚假状态重新计算未知测量，
% 并为所有伪造数据添加噪声。
%
% 输入:
%   system_measurements      - 完整的系统测量值 (作为蓝本)
%   attacker_measurements    - 攻击者已知的测量值
%   best_sol                 - 包含优化结果的结构体 (a, Vc, uc)
%   attacker_measurement_map - 攻击者已知测量的映射表
%   mpc                      - MATPOWER案例结构体
%   pmu_config               - PMU配置信息
%   config                   - 包含所有配置参数的结构体
%
% 输出:
%   attacked_measurements (struct) - 完整的、带噪声的攻击后测量数据

    % --- 1. 参数提取与准备 ---
    noise_params = config.Noise;
    define_constants;
    [baseMVA, bus, branch] = deal(mpc.baseMVA, mpc.bus, mpc.branch);
    [Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);
    branch_from_bus = branch(:, F_BUS);
    branch_to_bus = branch(:, T_BUS);

    % Step 1: 以系统拥有的测量作为蓝本进行初始化
    attacked_measurements = system_measurements;
    
    % Step 2: 根据优化结果 (攻击向量 a) 更新攻击者已知的测量
    % 使用与 attacker_measurement_map 相同的 selection 组装顺序
    % 通过 config.MeasurementSelection 传入，以保证维度与 best_sol.a 一致
    sel = struct();
    if isfield(config, 'MeasurementSelection')
        sel = config.MeasurementSelection;
    end
    [z_vector_known, ~, ~] = vectorizeAndMapMeasurements(attacker_measurements, pmu_config, noise_params, struct('num_buses', size(bus,1), 'num_branches', size(branch,1), 'selection', sel));
    z_known_attacked = z_vector_known + best_sol.a;
    
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
        if isfield(map_item, 'indices') && ~isempty(map_item.indices)
            sel_idx = map_item.indices(:);
        end
        
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
        else % PMU
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
    
    % Step 3: 根据虚假状态 x_c 重新计算攻击者未知的测量值
    compromised_v_complex = best_sol.Vc .* exp(1j * best_sol.uc);
    
    estimated_S_injection_c = compromised_v_complex .* conj(Y_bus * compromised_v_complex);
    estimated_S_from_c = compromised_v_complex(branch_from_bus) .* conj(Y_from * compromised_v_complex);
    estimated_S_to_c = compromised_v_complex(branch_to_bus) .* conj(Y_to * compromised_v_complex);

    all_system_fields = fieldnames(system_measurements.scada);
    scada_fields = {'v','pi','qi','pf','qf','pt','qt'};
    scada_values = {abs(compromised_v_complex), real(estimated_S_injection_c), imag(estimated_S_injection_c), ...
                    real(estimated_S_from_c),   imag(estimated_S_from_c),   real(estimated_S_to_c),      imag(estimated_S_to_c)};
    for k = 1:length(scada_fields)
        f = scada_fields{k};
        if any(strcmp(all_system_fields, f)) && ~isfield(attacker_measurements.scada, f)
            attacked_measurements.scada.(f) = scada_values{k};
        end
    end
    
    % Step 3b: 根据虚假状态 x_c 重新计算攻击者未知的PMU测量值
    if isfield(system_measurements, 'pmu')
        % 计算支路电流（从虚假状态）
        estimated_If_c = Y_from * compromised_v_complex;
        estimated_It_c = Y_to * compromised_v_complex;
        
        all_system_pmu_fields = fieldnames(system_measurements.pmu);
        pmu_fields = {'vm','va','imf','iaf','imt','iat'};
        pmu_values = {abs(compromised_v_complex(pmu_config.locations)),  angle(compromised_v_complex(pmu_config.locations)), ...
                      abs(estimated_If_c(pmu_config.pmu_from_branch_indices)), angle(estimated_If_c(pmu_config.pmu_from_branch_indices)), ...
                      abs(estimated_It_c(pmu_config.pmu_to_branch_indices)),   angle(estimated_It_c(pmu_config.pmu_to_branch_indices))};
        for k = 1:length(pmu_fields)
            f = pmu_fields{k};
            if any(strcmp(all_system_pmu_fields, f)) && ~isfield(attacker_measurements.pmu, f)
                attacked_measurements.pmu.(f) = pmu_values{k};
            end
        end
    end

    % Step 4: 为伪造数据添加噪声以提高真实性
    if isfield(attacked_measurements.scada, 'v'),  attacked_measurements.scada.v  = attacked_measurements.scada.v  + noise_params.scada.v * randn(size(attacked_measurements.scada.v)); end
    if isfield(attacked_measurements.scada, 'pi'), attacked_measurements.scada.pi = attacked_measurements.scada.pi + noise_params.scada.p * randn(size(attacked_measurements.scada.pi)); end
    if isfield(attacked_measurements.scada, 'qi'), attacked_measurements.scada.qi = attacked_measurements.scada.qi + noise_params.scada.q * randn(size(attacked_measurements.scada.qi)); end
    if isfield(attacked_measurements.scada, 'pf'), attacked_measurements.scada.pf = attacked_measurements.scada.pf + noise_params.scada.p * randn(size(attacked_measurements.scada.pf)); end
    if isfield(attacked_measurements.scada, 'qf'), attacked_measurements.scada.qf = attacked_measurements.scada.qf + noise_params.scada.q * randn(size(attacked_measurements.scada.qf)); end
    if isfield(attacked_measurements.scada, 'pt'), attacked_measurements.scada.pt = attacked_measurements.scada.pt + noise_params.scada.p * randn(size(attacked_measurements.scada.pt)); end
    if isfield(attacked_measurements.scada, 'qt'), attacked_measurements.scada.qt = attacked_measurements.scada.qt + noise_params.scada.q * randn(size(attacked_measurements.scada.qt)); end
    
    if isfield(attacked_measurements, 'pmu')
        if isfield(attacked_measurements.pmu, 'vm'),  attacked_measurements.pmu.vm  = attacked_measurements.pmu.vm  + noise_params.pmu.vm * randn(size(attacked_measurements.pmu.vm)); end
        if isfield(attacked_measurements.pmu, 'va'),  attacked_measurements.pmu.va  = attacked_measurements.pmu.va  + noise_params.pmu.va * randn(size(attacked_measurements.pmu.va)); end
        if isfield(attacked_measurements.pmu, 'imf'), attacked_measurements.pmu.imf = attacked_measurements.pmu.imf + noise_params.pmu.im * randn(size(attacked_measurements.pmu.imf)); end
        if isfield(attacked_measurements.pmu, 'iaf'), attacked_measurements.pmu.iaf = attacked_measurements.pmu.iaf + noise_params.pmu.ia * randn(size(attacked_measurements.pmu.iaf)); end
        if isfield(attacked_measurements.pmu, 'imt'), attacked_measurements.pmu.imt = attacked_measurements.pmu.imt + noise_params.pmu.im * randn(size(attacked_measurements.pmu.imt)); end
        if isfield(attacked_measurements.pmu, 'iat'), attacked_measurements.pmu.iat = attacked_measurements.pmu.iat + noise_params.pmu.ia * randn(size(attacked_measurements.pmu.iat)); end
    end
end

function S = apply_pmu_complex_pair(S, kind, real_part, imag_part, idx, pmu_config)
% 将直角分量对写回 PMU 极坐标数据（abs/angle），按索引映射
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

function S = add_noise_fields(S, subname, fields, sigma)
% 对子结构 subname 下的字段批量加噪（存在才加）
    if ~isfield(S, subname), return; end
    for i = 1:numel(fields)
        f = fields{i};
        if isfield(S.(subname), f)
            S.(subname).(f) = S.(subname).(f) + sigma * randn(size(S.(subname).(f)));
        end
    end
end
