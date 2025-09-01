function [z_vector, z_map, z_weights] = vectorizeAndMapMeasurements(measurements, pmu_config, noise_params, expected_sizes)
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

    % 预期尺寸（可选），若提供则统一调用校验函数
    num_buses = [];
    num_branches = [];
    selection = struct();
    if nargin >= 4 && ~isempty(expected_sizes)
        if isfield(expected_sizes, 'num_buses'), num_buses = expected_sizes.num_buses; end
        if isfield(expected_sizes, 'num_branches'), num_branches = expected_sizes.num_branches; end
        if isfield(expected_sizes, 'selection') && ~isempty(expected_sizes.selection)
            selection = expected_sizes.selection;
        end
        if ~isempty(num_buses) && ~isempty(num_branches)
            % 统一校验（集中化报错信息）
            validateMeasurementDimensions(measurements, pmu_config, num_buses, num_branches);
        end
    end

    % --- 1. SCADA 测量处理 ---
    if isfield(measurements, 'scada')
        scada_meas = measurements.scada;
        scada_fields = {'v', 'pi', 'qi', 'pf', 'qf', 'pt', 'qt'};
        noise_map = struct('v', noise_params.scada.v, 'pi', noise_params.scada.p, ...
                           'qi', noise_params.scada.q, 'pf', noise_params.scada.p, ...
                           'qf', noise_params.scada.q, 'pt', noise_params.scada.p, ...
                           'qt', noise_params.scada.q);
                           
        % 尺寸校验已集中在 validateMeasurementDimensions 中（若提供 expected_sizes）

        for i = 1:length(scada_fields)
            field = scada_fields{i};
            if isfield(scada_meas, field) && ~isempty(scada_meas.(field))
                % 如果 selection 明确给出该字段且为空，视为禁用该字段
                if isfield(selection, 'SCADA') && isfield(selection.SCADA, [field '_idx']) && isempty(selection.SCADA.([field '_idx']))
                    continue;
                end
                full_data = scada_meas.(field)(:);
                % 选择索引（母线或支路）：默认全覆盖
                switch field
                    case {'v','pi','qi'}
                        if isfield(selection, 'SCADA') && isfield(selection.SCADA, [field '_idx']) && ~isempty(selection.SCADA.([field '_idx']))
                            sel_idx = selection.SCADA.([field '_idx'])(:);
                        else
                            if isempty(num_buses), num_buses = length(full_data); end
                            sel_idx = (1:num_buses)';
                        end
                    otherwise % 分支量测 pf/qf/pt/qt
                        if isfield(selection, 'SCADA') && isfield(selection.SCADA, [field '_idx']) && ~isempty(selection.SCADA.([field '_idx']))
                            sel_idx = selection.SCADA.([field '_idx'])(:);
                        else
                            if isempty(num_branches), num_branches = length(full_data); end
                            sel_idx = (1:num_branches)';
                        end
                end
                % 取子集并记录 indices
                data = full_data(sel_idx);
                count = length(data);
                z_map{end+1} = struct('type', 'scada', 'field', field, 'count', count, 'indices', sel_idx);
                vector_parts{end+1} = data;
                weights_parts{end+1} = repmat(1 / noise_map.(field)^2, count, 1);
            end
        end
    end

    % --- 2. PMU 测量处理 (转换为直角坐标) ---
    if isfield(measurements, 'pmu')
        pmu_meas = measurements.pmu;
        % 尺寸校验已集中在 validateMeasurementDimensions 中（若提供 expected_sizes）
        
        % 电压相量 -> 直角坐标（按选择的PMU母线子集）
        if isfield(pmu_meas, 'vm') && ~isempty(pmu_meas.vm)
            % 选择PMU母线索引（默认为全部 pmu_config.locations）
            if isfield(selection, 'PMU') && isfield(selection.PMU, 'bus_idx') && isempty(selection.PMU.bus_idx)
                % 明确空：禁用 PMU 电压段
                sel_bus = [];
            elseif isfield(selection, 'PMU') && isfield(selection.PMU, 'bus_idx') && ~isempty(selection.PMU.bus_idx)
                sel_bus = selection.PMU.bus_idx(:);
            else
                sel_bus = pmu_config.locations(:);
            end
            if isempty(sel_bus)
                % 跳过 PMU 电压两段
            else
            % 将母线编号映射到 pmu 测量向量的位置
            [~, pos] = ismember(sel_bus, pmu_config.locations(:)); pos = pos(pos>0);
            vm_sel = pmu_meas.vm(pos);
            va_sel = pmu_meas.va(pos);
            v_real = vm_sel .* cos(va_sel);
            v_imag = vm_sel .* sin(va_sel);
            
            count = length(v_real);
            z_map{end+1} = struct('type', 'pmu', 'field', 'v_real', 'count', count, 'indices', sel_bus);
            vector_parts{end+1} = v_real;
            s_vm2=noise_params.pmu.vm^2; s_va2=noise_params.pmu.va^2;
            s2_vr=(cos(va_sel)).^2*s_vm2+(vm_sel.*sin(va_sel)).^2*s_va2;
            weights_parts{end+1} = 1./s2_vr;
            
            % 同时入队虚部（若va存在）
            if isfield(pmu_meas, 'va') && ~isempty(pmu_meas.va)
                z_map{end+1} = struct('type', 'pmu', 'field', 'v_imag', 'count', count, 'indices', sel_bus);
                vector_parts{end+1} = v_imag;
                s2_vi=(sin(va_sel)).^2*s_vm2+(vm_sel.*cos(va_sel)).^2*s_va2;
                weights_parts{end+1} = 1./s2_vi;
            end
            end
        end
        
        s_im2=noise_params.pmu.im^2; s_ia2=noise_params.pmu.ia^2;
        % From-end 电流相量 -> 直角坐标（按选择的支路子集）
        if isfield(pmu_meas, 'imf') && ~isempty(pmu_meas.imf)
            if isfield(selection, 'PMU') && isfield(selection.PMU, 'from_branch_idx') && isempty(selection.PMU.from_branch_idx)
                sel_br = [];
            elseif isfield(selection, 'PMU') && isfield(selection.PMU, 'from_branch_idx') && ~isempty(selection.PMU.from_branch_idx)
                sel_br = selection.PMU.from_branch_idx(:);
            else
                sel_br = pmu_config.pmu_from_branch_indices(:);
            end
            if ~isempty(sel_br)
                [~, pos] = ismember(sel_br, pmu_config.pmu_from_branch_indices(:)); pos = pos(pos>0);
                imf_sel = pmu_meas.imf(pos);
                iaf_sel = pmu_meas.iaf(pos);
                if_real = imf_sel .* cos(iaf_sel);
                if_imag = imf_sel .* sin(iaf_sel);
                count = length(if_real);

                z_map{end+1} = struct('type', 'pmu', 'field', 'if_real', 'count', count, 'indices', sel_br);
                vector_parts{end+1} = if_real;
                s2_ifr=(cos(iaf_sel)).^2*s_im2+(imf_sel.*sin(iaf_sel)).^2*s_ia2;
                weights_parts{end+1} = 1./s2_ifr;

                if isfield(pmu_meas, 'iaf') && ~isempty(pmu_meas.iaf)
                    z_map{end+1} = struct('type', 'pmu', 'field', 'if_imag', 'count', count, 'indices', sel_br);
                    vector_parts{end+1} = if_imag;
                    s2_ifi=(sin(iaf_sel)).^2*s_im2+(imf_sel.*cos(iaf_sel)).^2*s_ia2;
                    weights_parts{end+1} = 1./s2_ifi;
                end
            end
        end
        
        % To-end 电流相量 -> 直角坐标（按选择的支路子集）
        if isfield(pmu_meas, 'imt') && ~isempty(pmu_meas.imt)
            if isfield(selection, 'PMU') && isfield(selection.PMU, 'to_branch_idx') && isempty(selection.PMU.to_branch_idx)
                sel_br_t = [];
            elseif isfield(selection, 'PMU') && isfield(selection.PMU, 'to_branch_idx') && ~isempty(selection.PMU.to_branch_idx)
                sel_br_t = selection.PMU.to_branch_idx(:);
            else
                sel_br_t = pmu_config.pmu_to_branch_indices(:);
            end
            if ~isempty(sel_br_t)
                [~, pos] = ismember(sel_br_t, pmu_config.pmu_to_branch_indices(:)); pos = pos(pos>0);
                imt_sel = pmu_meas.imt(pos);
                iat_sel = pmu_meas.iat(pos);
                it_real = imt_sel .* cos(iat_sel);
                it_imag = imt_sel .* sin(iat_sel);
                count = length(it_real);

                z_map{end+1} = struct('type', 'pmu', 'field', 'it_real', 'count', count, 'indices', sel_br_t);
                vector_parts{end+1} = it_real;
                s2_itr=(cos(iat_sel)).^2*s_im2+(imt_sel.*sin(iat_sel)).^2*s_ia2;
                weights_parts{end+1} = 1./s2_itr;

                if isfield(pmu_meas, 'iat') && ~isempty(pmu_meas.iat)
                    z_map{end+1} = struct('type', 'pmu', 'field', 'it_imag', 'count', count, 'indices', sel_br_t);
                    vector_parts{end+1} = it_imag;
                    s2_iti=(sin(iat_sel)).^2*s_im2+(imt_sel.*cos(iat_sel)).^2*s_ia2;
                    weights_parts{end+1} = 1./s2_iti;
                end
            end
        end
    end
    
    % --- 3. 组装最终输出 ---
    z_vector = vertcat(vector_parts{:});
    z_weights = vertcat(weights_parts{:});
end
