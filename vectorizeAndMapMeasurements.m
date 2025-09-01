function [z_vector, z_map, z_weights] = vectorizeAndMapMeasurements(measurements, pmu_config, noise_params, expected_sizes)
% vectorizeAndMapMeasurements 函数的核心作用是将结构化的测量数据（包含SCADA和PMU）转换成状态估计器所需的向量化形式。
% 它不仅将不同类型的测量值拼接成一个单一的列向量 (z_vector)，还生成一个详细的映射表 (z_map) 来记录每个值原先的
% 类型、物理意义和索引，并计算出每个测量值对应的权重 (z_weights)，该权重通常是测量误差方差的倒数。
%
% 特别地，该函数负责处理 PMU 提供的极坐标相量（幅值和相角），将其转换为直角坐标（实部和虚部），
% 并通过一阶误差传播理论计算转换后直角坐标分量的权重。
%
% 输入:
% - measurements   : struct, 包含两个可选字段 .scada 和 .pmu。
%                    .scada: struct, 包含电压幅值(v), 有功/无功注入(pi,qi), 有功/无功潮流(pf,qf,pt,qt)等字段。
%                    .pmu: struct, 包含电压相量(vm,va), "from"端和"to"端电流相量(imf,iaf,imt,iat)。
% - pmu_config     : struct, 定义PMU的配置信息，如安装位置（母线索引）、监控的支路索引等。
%                    包含 .locations, .pmu_from_branch_indices, .pmu_to_branch_indices。
% - noise_params   : struct, 定义了各类测量噪声的统计特性（标准差）。
%                    包含 .scada 和 .pmu 两个子结构，其下字段与 measurements 对应。
% - expected_sizes : (可选) struct, 用于提供系统尺寸（母线数、支路数）和测量选择信息。
%                    .num_buses: 系统中的母线总数。
%                    .num_branches: 系统中的支路总数。
%                    .selection: 一个结构，可以指定只使用哪些测量，例如 `selection.SCADA.v_idx = [1, 3, 5]`
%                                表示只使用1、3、5号母线的SCADA电压测量。
%
% 输出:
% - z_vector  : double列向量, 所有测量值按顺序拼接而成。PMU相量被转换为实部和虚部两个分量。
% - z_map     : cell数组, 描述 z_vector 的结构。每个cell是一个struct，包含：
%               .type: 'scada' 或 'pmu'。
%               .field: 原始测量字段名（如 'v', 'pi', 'v_real', 'v_imag'）。
%               .count: 该类型测量的数量。
%               .indices: 这些测量对应的原始母线或支路索引。
% - z_weights : double列向量, 与 z_vector 中的每个元素一一对应，值为 1 / (标准差^2)。

    % 初始化输出变量和内部临时变量
    z_map = {};         % 初始化测量映射表为空cell数组
    vector_parts = {};  % 用于存储各段测量向量的cell数组
    weights_parts = {}; % 用于存储各段权重向量的cell数组

    % --- 0) 参数解析与验证 ---
    % 从可选输入参数 expected_sizes 中提取系统拓扑信息和测量选择配置
    num_buses = [];
    num_branches = [];
    selection = struct();
    if nargin >= 4 && ~isempty(expected_sizes)
        if isfield(expected_sizes, 'num_buses'), num_buses = expected_sizes.num_buses; end
        if isfield(expected_sizes, 'num_branches'), num_branches = expected_sizes.num_branches; end
        % 如果提供了选择配置，则使用它
        if isfield(expected_sizes, 'selection') && ~isempty(expected_sizes.selection)
            selection = expected_sizes.selection;
        end
        % 如果系统尺寸已知，则调用验证函数检查测量数据的维度是否匹配
        if ~isempty(num_buses) && ~isempty(num_branches)
            validateMeasurementDimensions(measurements, pmu_config, num_buses, num_branches);
        end
    end

    % --- 1) 处理 SCADA 测量 ---
    if isfield(measurements, 'scada')
        scada_meas = measurements.scada;
        % 定义所有可能的SCADA测量字段
        scada_fields = {'v','pi','qi','pf','qf','pt','qt'};
        % 定义哪些字段属于母线测量（其余为支路测量）
        bus_fields = {'v','pi','qi'}; 
        % 创建一个从字段名到噪声标准差的映射，方便后续查找
        noise_map = struct('v',  noise_params.scada.v, ...
                           'pi', noise_params.scada.p, ...
                           'qi', noise_params.scada.q, ...
                           'pf', noise_params.scada.p, ...
                           'qf', noise_params.scada.q, ...
                           'pt', noise_params.scada.p, ...
                           'qt', noise_params.scada.q);

        % 遍历每一种SCADA测量类型
        for i = 1:numel(scada_fields)
            field = scada_fields{i};
            % 如果当前类型的测量不存在或为空，则跳过
            if ~isfield(scada_meas, field) || isempty(scada_meas.(field)), continue; end

            full_data = scada_meas.(field)(:);
            % 判断当前测量是母线量还是支路量
            is_bus = any(strcmp(field, bus_fields));
            if is_bus
                if isempty(num_buses), num_buses = length(full_data); end % 如果母线数未知，则从数据长度推断
                default_idx = (1:num_buses)'; % 默认使用所有母线的测量
            else
                if isempty(num_branches), num_branches = length(full_data); end % 如果支路数未知，则从数据长度推断
                default_idx = (1:num_branches)'; % 默认使用所有支路的测量
            end
            
            % 使用 pick 函数根据 'selection' 配置决定最终使用哪些索引的测量
            sel_idx = pick(selection, 'SCADA', [field '_idx'], default_idx);
            if isempty(sel_idx), continue; end % 如果配置为显式空数组，表示禁用该类型测量
            
            % 根据选择的索引提取数据
            data = full_data(sel_idx);
            % 计算权重向量，权重 = 1 / (标准差^2)
            w = repmat(1 / noise_map.(field)^2, numel(sel_idx), 1);
            % 调用 add_seg 函数将处理好的数据段追加到结果中
            [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'scada', field, sel_idx, data, w);
        end
    end

    % --- 2) 处理 PMU 测量（关键：极坐标到直角坐标转换 及 误差传播） ---
    if isfield(measurements, 'pmu')
        pmu_meas = measurements.pmu;

        % 提取PMU电压和电流相量的噪声标准差
        s_vm = noise_params.pmu.vm; s_va = noise_params.pmu.va; % 电压幅值和相角的标准差
        s_im = noise_params.pmu.im; s_ia = noise_params.pmu.ia; % 电流幅值和相角的标准差

        % 2a) 处理母线电压相量 (Vm, Va)
        if isfield(pmu_meas,'vm') && ~isempty(pmu_meas.vm)
            % 确定要使用的PMU母线索引
            sel_bus = pick(selection,'PMU','bus_idx', pmu_config.locations(:));
            if ~isempty(sel_bus)
                % 找到选中母线在原始PMU测量向量中的位置
                [~, pos] = ismember(sel_bus, pmu_config.locations(:)); pos = pos(pos>0);
                vm_sel = pmu_meas.vm(pos); va_sel = pmu_meas.va(pos);
                % 将极坐标测量转换为直角坐标，并计算新坐标下的权重
                [v_real, v_imag, w_vr, w_vi] = polar_to_rect(vm_sel, va_sel, s_vm, s_va);
                
                % 将电压实部添加到结果中
                [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','v_real', sel_bus, v_real, w_vr);
                % 如果存在相角测量，则也将虚部添加到结果中
                if isfield(pmu_meas,'va') && ~isempty(pmu_meas.va)
                    [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','v_imag', sel_bus, v_imag, w_vi);
                end
            end
        end

        % 2b) 处理 "From" 端支路电流相量 (Imf, Iaf)
        if isfield(pmu_meas,'imf') && ~isempty(pmu_meas.imf)
            % 确定要使用的 "from" 端支路索引
            sel_br = pick(selection,'PMU','from_branch_idx', pmu_config.pmu_from_branch_indices(:));
            if ~isempty(sel_br)
                [~, pos] = ismember(sel_br, pmu_config.pmu_from_branch_indices(:)); pos = pos(pos>0);
                imf_sel = pmu_meas.imf(pos); iaf_sel = pmu_meas.iaf(pos);
                % 转换并计算权重
                [if_real, if_imag, w_ifr, w_ifi] = polar_to_rect(imf_sel, iaf_sel, s_im, s_ia);
                % 添加实部
                [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','if_real', sel_br, if_real, w_ifr);
                % 添加虚部
                if isfield(pmu_meas,'iaf') && ~isempty(pmu_meas.iaf)
                    [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','if_imag', sel_br, if_imag, w_ifi);
                end
            end
        end

        % 2c) 处理 "To" 端支路电流相量 (Imt, Iat)
        if isfield(pmu_meas,'imt') && ~isempty(pmu_meas.imt)
            % 确定要使用的 "to" 端支路索引
            sel_br_t = pick(selection,'PMU','to_branch_idx', pmu_config.pmu_to_branch_indices(:));
            if ~isempty(sel_br_t)
                [~, pos] = ismember(sel_br_t, pmu_config.pmu_to_branch_indices(:)); pos = pos(pos>0);
                imt_sel = pmu_meas.imt(pos); iat_sel = pmu_meas.iat(pos);
                % 转换并计算权重
                [it_real, it_imag, w_itr, w_iti] = polar_to_rect(imt_sel, iat_sel, s_im, s_ia);
                % 添加实部
                [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','it_real', sel_br_t, it_real, w_itr);
                % 添加虚部
                if isfield(pmu_meas,'iat') && ~isempty(pmu_meas.iat)
                    [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, 'pmu','it_imag', sel_br_t, it_imag, w_iti);
                end
            end
        end
    end

    % --- 3) 汇总所有测量段 ---
    % 将cell数组中存储的各个数据段垂直拼接成最终的列向量
    if ~isempty(vector_parts)
        z_vector  = vertcat(vector_parts{:});
        z_weights = vertcat(weights_parts{:});
    else
        z_vector = [];
        z_weights = [];
    end
end

%% 辅助函数

function sel = pick(selobj, domain, key, def)
% pick 是一个辅助函数，用于从一个可能不存在的嵌套结构体 `selobj` 中安全地获取值。
% 它常用于实现灵活的配置选项，允许用户覆盖默认行为。
%
% 输入:
% - selobj : 目标结构体，例如 `selection` 配置对象。
% - domain : 结构体的一级字段名，如 'SCADA' 或 'PMU'。
% - key    : 结构的二级字段名，如 'v_idx'。
% - def    : 如果在 `selobj` 中找不到指定的路径，则返回此默认值。
%
% 输出:
% - sel    : 获取到的值（如果找到）或默认值（如果未找到）。
    sel = def;
    % 检查路径 selobj.domain.key 是否有效
    if isstruct(selobj) && isfield(selobj, domain) && isfield(selobj.(domain), key)
        val = selobj.(domain).(key);
        if ~isempty(val)
            sel = val(:); % 确保返回的是列向量
        else
            sel = []; % 如果字段值为显式空数组，则返回空数组
        end
    end
end

function [z_map, vector_parts, weights_parts] = add_seg(z_map, vector_parts, weights_parts, typ, field, idx, data, weights)
% add_seg 函数用于将一个处理好的测量数据段（segment）原子地追加到三个结果cell数组中。
% 这样做可以确保 z_map, vector_parts, 和 weights_parts 始终保持同步。
%
% 输入:
% - z_map, vector_parts, weights_parts : 当前正在构建中的结果cell数组。
% - typ     : 测量类型, 'scada' 或 'pmu'。
% - field   : 测量字段名, 如 'v', 'pi', 'v_real'。
% - idx     : 与该数据段关联的母线或支路索引向量。
% - data    : 该数据段的测量值向量。
% - weights : 该数据段的权重向量。
%
% 输出:
% - 更新后的结果cell数组。
    cnt = numel(idx);
    % 在 z_map 中添加一个新的条目来描述这个数据段
    z_map{end+1} = struct('type', typ, 'field', field, 'count', cnt, 'indices', idx);
    % 将数据和权重向量追加到各自的cell数组中
    vector_parts{end+1}  = data(:);
    weights_parts{end+1} = weights(:);
end

function [xr, xi, wr, wi] = polar_to_rect(mag, ang, s_mag, s_ang)
% polar_to_rect 函数执行两个关键任务：
% 1. 将极坐标 (幅值 mag, 相角 ang) 转换为直角坐标 (实部 xr, 虚部 xi)。
% 2. 基于一阶泰勒展开（误差传播定律），计算转换后直角坐标分量的方差，并返回其倒数作为权重。
% 假定幅值和相角的测量误差是不相关的。
%
% 输入:
% - mag   : 幅值向量。
% - ang   : 相角向量（弧度）。
% - s_mag : 幅值测量的标准差。
% - s_ang : 相角测量的标准差（弧度）。
%
% 输出:
% - xr, xi : 计算出的实部和虚部向量。
% - wr, wi : 实部和虚部对应的权重向量 (1/方差)。
    
    % 标准极坐标到直角坐标转换
    xr = mag .* cos(ang);
    xi = mag .* sin(ang);

    % --- 一阶误差传播 ---
    % 设 xr = f(mag, ang) = mag * cos(ang)。
    % var(xr) ≈ (∂xr/∂mag)² * var(mag) + (∂xr/∂ang)² * var(ang)
    % ∂xr/∂mag = cos(ang), ∂xr/∂ang = -mag * sin(ang)
    % var(mag) = s_mag², var(ang) = s_ang²
    % 所以, var(xr) = cos(ang)² * s_mag² + (-mag * sin(ang))² * s_ang²
    s2r = (cos(ang)).^2 .* (s_mag.^2) + (mag .* sin(ang)).^2 .* (s_ang.^2);

    % 同理, 设 xi = g(mag, ang) = mag * sin(ang)。
    % var(xi) ≈ (∂xi/∂mag)² * var(mag) + (∂xi/∂ang)² * var(ang)
    % ∂xi/∂mag = sin(ang), ∂xi/∂ang = mag * cos(ang)
    % 所以, var(xi) = sin(ang)² * s_mag² + (mag * cos(ang))² * s_ang²
    s2i = (sin(ang)).^2 .* (s_mag.^2) + (mag .* cos(ang)).^2 .* (s_ang.^2);

    % 权重是方差的倒数
    wr = 1 ./ s2r;
    wi = 1 ./ s2i;
end
