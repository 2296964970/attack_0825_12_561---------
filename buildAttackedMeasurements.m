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
    [z_vector_known, ~, ~] = vectorizeAndMapMeasurements(attacker_measurements, pmu_config, noise_params);
    z_known_attacked = z_vector_known + best_sol.a;
    
    current_row = 1;
    v_real_part = []; if_real_part = []; it_real_part = [];

    for i = 1:length(attacker_measurement_map)
        map_item = attacker_measurement_map{i};
        count = map_item.count;
        indices = current_row : (current_row + count - 1);
        data_to_update = z_known_attacked(indices);
        
        if strcmp(map_item.type, 'scada')
            if isfield(attacked_measurements.scada, map_item.field)
                attacked_measurements.scada.(map_item.field) = data_to_update;
            end
        else % PMU
            switch map_item.field
                case 'v_real',  v_real_part = data_to_update;
                case 'v_imag'
                    if ~isempty(v_real_part)
                        if isfield(attacked_measurements.pmu, 'vm'), attacked_measurements.pmu.vm = abs(v_real_part + 1j*data_to_update); end
                        if isfield(attacked_measurements.pmu, 'va'), attacked_measurements.pmu.va = angle(v_real_part + 1j*data_to_update); end
                    end
                case 'if_real', if_real_part = data_to_update;
                case 'if_imag'
                    if ~isempty(if_real_part)
                        if isfield(attacked_measurements.pmu, 'imf'), attacked_measurements.pmu.imf = abs(if_real_part + 1j*data_to_update); end
                        if isfield(attacked_measurements.pmu, 'iaf'), attacked_measurements.pmu.iaf = angle(if_real_part + 1j*data_to_update); end
                    end
                case 'it_real', it_real_part = data_to_update;
                case 'it_imag'
                    if ~isempty(it_real_part)
                        if isfield(attacked_measurements.pmu, 'imt'), attacked_measurements.pmu.imt = abs(it_real_part + 1j*data_to_update); end
                        if isfield(attacked_measurements.pmu, 'iat'), attacked_measurements.pmu.iat = angle(it_real_part + 1j*data_to_update); end
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
    for k = 1:length(all_system_fields)
        field = all_system_fields{k};
        if ~isfield(attacker_measurements.scada, field) % If it's an unknown field
            switch field
                case 'v',  attacked_measurements.scada.v = abs(compromised_v_complex);
                case 'pi', attacked_measurements.scada.pi = real(estimated_S_injection_c);
                case 'qi', attacked_measurements.scada.qi = imag(estimated_S_injection_c);
                case 'pf', attacked_measurements.scada.pf = real(estimated_S_from_c);
                case 'qf', attacked_measurements.scada.qf = imag(estimated_S_from_c);
                case 'pt', attacked_measurements.scada.pt = real(estimated_S_to_c);
                case 'qt', attacked_measurements.scada.qt = imag(estimated_S_to_c);
            end
        end
    end
    
    % Step 3b: 根据虚假状态 x_c 重新计算攻击者未知的PMU测量值
    if isfield(system_measurements, 'pmu')
        % 计算支路电流（从虚假状态）
        estimated_If_c = Y_from * compromised_v_complex;
        estimated_It_c = Y_to * compromised_v_complex;
        
        all_system_pmu_fields = fieldnames(system_measurements.pmu);
        for k = 1:length(all_system_pmu_fields)
            field = all_system_pmu_fields{k};
            if ~isfield(attacker_measurements.pmu, field) % If it's an unknown PMU field
                switch field
                    case 'vm', attacked_measurements.pmu.vm = abs(compromised_v_complex(pmu_config.locations));
                    case 'va', attacked_measurements.pmu.va = angle(compromised_v_complex(pmu_config.locations));
                    case 'imf', attacked_measurements.pmu.imf = abs(estimated_If_c(pmu_config.pmu_from_branch_indices));
                    case 'iaf', attacked_measurements.pmu.iaf = angle(estimated_If_c(pmu_config.pmu_from_branch_indices));
                    case 'imt', attacked_measurements.pmu.imt = abs(estimated_It_c(pmu_config.pmu_to_branch_indices));
                    case 'iat', attacked_measurements.pmu.iat = angle(estimated_It_c(pmu_config.pmu_to_branch_indices));
                end
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