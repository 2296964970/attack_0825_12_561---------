function [z_vector, z_map, z_weights] = vectorizeAndMapMeasurements(measurements, pmu_config, noise_params)
%VECTORIZEANDMAPMEASUREMENTS 将结构化的测量数据转换为向量和映射表
%
%   语法:
%       [z_vector, z_map, z_weights] = vectorizeAndMapMeasurements(measurements, pmu_config, noise_params)
%
%   描述:
%       此函数将一个包含SCADA和PMU测量的复杂结构体转换为一个单一的
%       数值列向量(z_vector)，并生成一个映射表(z_map)来记录向量中
%       每个分量的来源。它还将PMU的极坐标测量转换为直角坐标。
%       同时，它会根据noise_params计算每个测量对应的权重。
%
%   输入参数:
%       measurements - (struct) 包含 .scada 和 .pmu 字段的测量结构体。
%       pmu_config   - (struct) PMU配置，包含 .locations, .pmu_from_branch_indices 等。
%       noise_params - (struct) 包含各类测量噪声标准差的结构体。
%
%   输出参数:
%       z_vector  - (vector) 所有测量值组成的列向量。PMU值已转换为直角坐标。
%       z_map     - (cell) 描述z_vector构成的映射表。
%       z_weights - (vector) 与z_vector中每个元素对应的权重 (1/sigma^2)。

    z_map = {};
    vector_parts = {};
    weights_parts = {};

    % --- 1. SCADA 测量处理 ---
    if isfield(measurements, 'scada')
        scada_meas = measurements.scada;
        scada_fields = {'v', 'pi', 'qi', 'pf', 'qf', 'pt', 'qt'};
        noise_map = struct('v', noise_params.scada.v, 'pi', noise_params.scada.p, ...
                           'qi', noise_params.scada.q, 'pf', noise_params.scada.p, ...
                           'qf', noise_params.scada.q, 'pt', noise_params.scada.p, ...
                           'qt', noise_params.scada.q);
                           
        for i = 1:length(scada_fields)
            field = scada_fields{i};
            if isfield(scada_meas, field) && ~isempty(scada_meas.(field))
                data = scada_meas.(field)(:);
                count = length(data);
                z_map{end+1} = struct('type', 'scada', 'field', field, 'count', count);
                vector_parts{end+1} = data;
                weights_parts{end+1} = repmat(1 / noise_map.(field)^2, count, 1);
            end
        end
    end

    % --- 2. PMU 测量处理 (转换为直角坐标) ---
    if isfield(measurements, 'pmu')
        pmu_meas = measurements.pmu;
        
        % 电压实部 (vm存在时)
        if isfield(pmu_meas, 'vm') && ~isempty(pmu_meas.vm)
            v_real = pmu_meas.vm .* cos(pmu_meas.va);
            count = length(v_real);
            
            z_map{end+1} = struct('type', 'pmu', 'field', 'v_real', 'count', count, 'indices', pmu_config.locations);
            vector_parts{end+1} = v_real;
            
            s_vm2=noise_params.pmu.vm^2; s_va2=noise_params.pmu.va^2;
            s2_vr=(cos(pmu_meas.va)).^2*s_vm2+(pmu_meas.vm.*sin(pmu_meas.va)).^2*s_va2;
            weights_parts{end+1} = 1./s2_vr;
        end
        
        % 电压虚部 (va存在时)
        if isfield(pmu_meas, 'va') && ~isempty(pmu_meas.va)
            v_imag = pmu_meas.vm .* sin(pmu_meas.va);
            count = length(v_imag);
            
            z_map{end+1} = struct('type', 'pmu', 'field', 'v_imag', 'count', count, 'indices', pmu_config.locations);
            vector_parts{end+1} = v_imag;
            
            s_vm2=noise_params.pmu.vm^2; s_va2=noise_params.pmu.va^2;
            s2_vi=(sin(pmu_meas.va)).^2*s_vm2+(pmu_meas.vm.*cos(pmu_meas.va)).^2*s_va2;
            weights_parts{end+1} = 1./s2_vi;
        end
        
        s_im2=noise_params.pmu.im^2; s_ia2=noise_params.pmu.ia^2;
        % From-end 电流实部 (imf存在时)
        if isfield(pmu_meas, 'imf') && ~isempty(pmu_meas.imf)
            if_real = pmu_meas.imf .* cos(pmu_meas.iaf);
            count = length(if_real);

            z_map{end+1} = struct('type', 'pmu', 'field', 'if_real', 'count', count, 'indices', pmu_config.pmu_from_branch_indices);
            vector_parts{end+1} = if_real;
            
            s2_ifr=(cos(pmu_meas.iaf)).^2*s_im2+(pmu_meas.imf.*sin(pmu_meas.iaf)).^2*s_ia2;
            weights_parts{end+1} = 1./s2_ifr;
        end
        
        % From-end 电流虚部 (iaf存在时)
        if isfield(pmu_meas, 'iaf') && ~isempty(pmu_meas.iaf)
            if_imag = pmu_meas.imf .* sin(pmu_meas.iaf);
            count = length(if_imag);

            z_map{end+1} = struct('type', 'pmu', 'field', 'if_imag', 'count', count, 'indices', pmu_config.pmu_from_branch_indices);
            vector_parts{end+1} = if_imag;
            
            s2_ifi=(sin(pmu_meas.iaf)).^2*s_im2+(pmu_meas.imf.*cos(pmu_meas.iaf)).^2*s_ia2;
            weights_parts{end+1} = 1./s2_ifi;
        end
        
        % To-end 电流实部 (imt存在时)
        if isfield(pmu_meas, 'imt') && ~isempty(pmu_meas.imt)
            it_real = pmu_meas.imt .* cos(pmu_meas.iat);
            count = length(it_real);

            z_map{end+1} = struct('type', 'pmu', 'field', 'it_real', 'count', count, 'indices', pmu_config.pmu_to_branch_indices);
            vector_parts{end+1} = it_real;

            s2_itr=(cos(pmu_meas.iat)).^2*s_im2+(pmu_meas.imt.*sin(pmu_meas.iat)).^2*s_ia2;
            weights_parts{end+1} = 1./s2_itr;
        end
        
        % To-end 电流虚部 (iat存在时)
        if isfield(pmu_meas, 'iat') && ~isempty(pmu_meas.iat)
            it_imag = pmu_meas.imt .* sin(pmu_meas.iat);
            count = length(it_imag);

            z_map{end+1} = struct('type', 'pmu', 'field', 'it_imag', 'count', count, 'indices', pmu_config.pmu_to_branch_indices);
            vector_parts{end+1} = it_imag;

            s2_iti=(sin(pmu_meas.iat)).^2*s_im2+(pmu_meas.imt.*cos(pmu_meas.iat)).^2*s_ia2;
            weights_parts{end+1} = 1./s2_iti;
        end
    end
    
    % --- 3. 组装最终输出 ---
    z_vector = vertcat(vector_parts{:});
    z_weights = vertcat(weights_parts{:});
end