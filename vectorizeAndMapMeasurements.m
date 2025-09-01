function [z_vector, z_map, z_weights] = vectorizeAndMapMeasurements(measurements, pmu_config, noise_params, expected_sizes)
%VECTORIZEANDMAPMEASUREMENTS 将结构化测量转换为向量/映射/权重
%
%   [z_vector, z_map, z_weights] = vectorizeAndMapMeasurements(measurements, pmu_config, noise_params, expected_sizes)
%
%   不改变任何外部接口与字段语义：
%   - z_map: 单元格数组，元素含 type/field/count/indices
%   - z_vector: 向量化的测量值（PMU 已转为直角坐标）
%   - z_weights: 与 z_vector 对应的权重（1/方差）

    % 累积容器
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
            validateMeasurementDimensions(measurements, pmu_config, num_buses, num_branches);
        end
    end

    % --- 1) SCADA ---
    if isfield(measurements, 'scada')
        scada_meas = measurements.scada;
        scada_fields = {'v','pi','qi','pf','qf','pt','qt'};
        bus_fields = {'v','pi','qi'}; % 其余为支路量测
        noise_map = struct('v', noise_params.scada.v, 'pi', noise_params.scada.p, ...
                           'qi', noise_params.scada.q, 'pf', noise_params.scada.p, ...
                           'qf', noise_params.scada.q, 'pt', noise_params.scada.p, ...
                           'qt', noise_params.scada.q);

        for i = 1:numel(scada_fields)
            field = scada_fields{i};
            if ~isfield(scada_meas, field) || isempty(scada_meas.(field)), continue; end

            full_data = scada_meas.(field)(:);
            is_bus = any(strcmp(field, bus_fields));
            if is_bus
                if isempty(num_buses), num_buses = length(full_data); end
                default_idx = (1:num_buses)';
            else
                if isempty(num_branches), num_branches = length(full_data); end
                default_idx = (1:num_branches)';
            end
            sel_idx = pick(selection, 'SCADA', [field '_idx'], default_idx);
            if isempty(sel_idx), continue; end % 显式空：禁用该字段

            data = full_data(sel_idx);
            w = repmat(1 / noise_map.(field)^2, numel(sel_idx), 1);
            [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'scada', field, sel_idx, data, w);
        end
    end

    % --- 2) PMU（极→直 + 误差传播） ---
    if isfield(measurements, 'pmu')
        pmu_meas = measurements.pmu;

        s_vm = noise_params.pmu.vm; s_va = noise_params.pmu.va; % 电压相量噪声（幅值/角）
        s_im = noise_params.pmu.im; s_ia = noise_params.pmu.ia; % 电流相量噪声（幅值/角）

        % 2a) 电压相量（按 PMU 母线选择）
        if isfield(pmu_meas,'vm') && ~isempty(pmu_meas.vm)
            sel_bus = pick(selection,'PMU','bus_idx', pmu_config.locations(:));
            if ~isempty(sel_bus)
                [~, pos] = ismember(sel_bus, pmu_config.locations(:)); pos = pos(pos>0);
                vm_sel = pmu_meas.vm(pos);
                va_sel = pmu_meas.va(pos);
                [v_real, v_imag, w_vr, w_vi] = polar_to_rect(vm_sel, va_sel, s_vm, s_va);
                [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','v_real', sel_bus, v_real, w_vr);
                if isfield(pmu_meas,'va') && ~isempty(pmu_meas.va)
                    [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','v_imag', sel_bus, v_imag, w_vi);
                end
            end
        end

        % 2b) From-end 电流相量
        if isfield(pmu_meas,'imf') && ~isempty(pmu_meas.imf)
            sel_br = pick(selection,'PMU','from_branch_idx', pmu_config.pmu_from_branch_indices(:));
            if ~isempty(sel_br)
                [~, pos] = ismember(sel_br, pmu_config.pmu_from_branch_indices(:)); pos = pos(pos>0);
                imf_sel = pmu_meas.imf(pos);
                iaf_sel = pmu_meas.iaf(pos);
                [if_real, if_imag, w_ifr, w_ifi] = polar_to_rect(imf_sel, iaf_sel, s_im, s_ia);
                [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','if_real', sel_br, if_real, w_ifr);
                if isfield(pmu_meas,'iaf') && ~isempty(pmu_meas.iaf)
                    [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','if_imag', sel_br, if_imag, w_ifi);
                end
            end
        end

        % 2c) To-end 电流相量
        if isfield(pmu_meas,'imt') && ~isempty(pmu_meas.imt)
            sel_br_t = pick(selection,'PMU','to_branch_idx', pmu_config.pmu_to_branch_indices(:));
            if ~isempty(sel_br_t)
                [~, pos] = ismember(sel_br_t, pmu_config.pmu_to_branch_indices(:)); pos = pos(pos>0);
                imt_sel = pmu_meas.imt(pos);
                iat_sel = pmu_meas.iat(pos);
                [it_real, it_imag, w_itr, w_iti] = polar_to_rect(imt_sel, iat_sel, s_im, s_ia);
                [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','it_real', sel_br_t, it_real, w_itr);
                if isfield(pmu_meas,'iat') && ~isempty(pmu_meas.iat)
                    [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','it_imag', sel_br_t, it_imag, w_iti);
                end
            end
        end
    end

    % --- 3) 汇总 ---
    z_vector = vertcat(vector_parts{:});
    z_weights = vertcat(weights_parts{:});
end

function sel = pick(selobj, domain, key, def)
% 从 selection 结构中安全读取字段，否则返回默认值
    sel = def;
    if isstruct(selobj) && isfield(selobj, domain) && isfield(selobj.(domain), key)
        val = selobj.(domain).(key);
        if ~isempty(val)
            sel = val(:);
        else
            sel = [];
        end
    end
end

function [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, typ, field, idx, data, weights)
% 追加一个测量段到映射与向量
    cnt = numel(idx);
    z_map{end+1} = struct('type', typ, 'field', field, 'count', cnt, 'indices', idx);
    vector_parts{end+1} = data(:);
    weights_parts{end+1} = weights(:);
end

function [xr, xi, wr, wi] = polar_to_rect(mag, ang, s_mag, s_ang)
% 极坐标到直角坐标 + 误差传播（返回权重 = 1/方差）
    xr = mag .* cos(ang);
    xi = mag .* sin(ang);
    s2r = (cos(ang)).^2 .* (s_mag.^2) + (mag .* sin(ang)).^2 .* (s_ang.^2);
    s2i = (sin(ang)).^2 .* (s_mag.^2) + (mag .* cos(ang)).^2 .* (s_ang.^2);
    wr = 1 ./ s2r;
    wi = 1 ./ s2i;
end
