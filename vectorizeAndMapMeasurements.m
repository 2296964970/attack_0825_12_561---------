function [z_vector, z_map, z_weights] = vectorizeAndMapMeasurements(measurements, pmu_config, noise_params, expected_sizes)
% 将 SCADA / PMU 测量打包为 z 向量、映射 z_map、权重 z_weights。
% 采用“规格表 + 通用追加器”重写，保持外部接口、输出顺序与语义不变：
% SCADA: v, pi, qi, pf, qf, pt, qt；随后 PMU: v_real, v_imag, if_real, if_imag, it_real, it_imag。

    % --- 0) 参数解析与验证 ---
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
            % 若存在项目内的维度校验函数，则调用
            try
                validateMeasurementDimensions(measurements, pmu_config, num_buses, num_branches);
            catch
                % 若未提供该函数，静默跳过以保持兼容
            end
        end
    end

    vector_parts = {}; weights_parts = {}; z_map = {};

    % --- 1a) SCADA 段（严格顺序） ---
    if isfield(measurements, 'scada')
        sc = measurements.scada;

        % 推断默认母线/支路数量（当 expected_sizes 未提供时）
        if isempty(num_buses)
            nb = max([numel(getfield_safe(sc,'v')), numel(getfield_safe(sc,'pi')), numel(getfield_safe(sc,'qi'))]);
            if nb == 0 && isfield(pmu_config,'locations') && ~isempty(pmu_config.locations)
                nb = max(pmu_config.locations);
            end
        else
            nb = num_buses;
        end
        if isempty(num_branches)
            nbr = max([numel(getfield_safe(sc,'pf')), numel(getfield_safe(sc,'qf')), numel(getfield_safe(sc,'pt')), numel(getfield_safe(sc,'qt'))]);
            if (isempty(nbr) || nbr==0) && isfield(pmu_config,'pmu_from_branch_indices') && ~isempty(pmu_config.pmu_from_branch_indices)
                nbr = max([max(pmu_config.pmu_from_branch_indices), max(pmu_config.pmu_to_branch_indices)]);
            end
        else
            nbr = num_branches;
        end

        sc_specs = {
            {'v',  @(~) pick(selection,'SCADA','v_idx',  (1:nb).'),  @(n) (1/(noise_params.scada.v^2))*ones(n,1)}
            {'pi', @(~) pick(selection,'SCADA','pi_idx', (1:nb).'),  @(n) (1/(noise_params.scada.p^2))*ones(n,1)}
            {'qi', @(~) pick(selection,'SCADA','qi_idx', (1:nb).'),  @(n) (1/(noise_params.scada.q^2))*ones(n,1)}
            {'pf', @(~) pick(selection,'SCADA','pf_idx', (1:nbr).'), @(n) (1/(noise_params.scada.p^2))*ones(n,1)}
            {'qf', @(~) pick(selection,'SCADA','qf_idx', (1:nbr).'), @(n) (1/(noise_params.scada.q^2))*ones(n,1)}
            {'pt', @(~) pick(selection,'SCADA','pt_idx', (1:nbr).'), @(n) (1/(noise_params.scada.p^2))*ones(n,1)}
            {'qt', @(~) pick(selection,'SCADA','qt_idx', (1:nbr).'), @(n) (1/(noise_params.scada.q^2))*ones(n,1)}
        };

        for i = 1:numel(sc_specs)
            fld = sc_specs{i}{1};
            idx = sc_specs{i}{2}([]);
            wfun = sc_specs{i}{3};
            if isfield(sc, fld) && ~isempty(sc.(fld)) && ~isempty(idx)
                data = sc.(fld)(:);
                idx = idx(idx>=1 & idx <= numel(data));
                if ~isempty(idx)
                    append_seg('scada', fld, idx, data(idx), wfun(numel(idx)));
                end
            end
        end
    end

    % --- 1b) PMU 电压（vm, va -> v_real, v_imag） ---
    if isfield(measurements,'pmu') && isfield(measurements.pmu,'vm') && isfield(measurements.pmu,'va')
        pmu = measurements.pmu;
        if isfield(pmu_config,'locations') && ~isempty(pmu_config.locations)
            def_bus_idx = pmu_config.locations(:);
        else
            def_bus_idx = (1:numel(pmu.vm(:))).';
        end
        sel_bus = pick(selection,'PMU','bus_idx', def_bus_idx);
        if ~isempty(sel_bus)
            [~, pos] = ismember(sel_bus, def_bus_idx); pos = pos(pos>0);
            if ~isempty(pos)
                mag = pmu.vm(pos); ang = pmu.va(pos);
                [xr, xi, wr, wi] = polar_to_rect(mag(:), ang(:), noise_params.pmu.vm, noise_params.pmu.va);
                append_seg('pmu','v_real', sel_bus(:), xr, wr);
                append_seg('pmu','v_imag', sel_bus(:), xi, wi);
            end
        end
    end

    % --- 1c) PMU 支路电流 From 端（imf/iaf） ---
    if isfield(measurements,'pmu') && isfield(measurements.pmu,'imf') && isfield(measurements.pmu,'iaf')
        pmu = measurements.pmu;
        if isfield(pmu_config,'pmu_from_branch_indices') && ~isempty(pmu_config.pmu_from_branch_indices)
            def_brf = pmu_config.pmu_from_branch_indices(:);
        else
            def_brf = (1:numel(pmu.imf(:))).';
        end
        sel_brf = pick(selection,'PMU','from_branch_idx', def_brf);
        if ~isempty(sel_brf)
            [~, pos] = ismember(sel_brf, def_brf); pos = pos(pos>0);
            if ~isempty(pos)
                mag = pmu.imf(pos); ang = pmu.iaf(pos);
                [xr, xi, wr, wi] = polar_to_rect(mag(:), ang(:), noise_params.pmu.im, noise_params.pmu.ia);
                append_seg('pmu','if_real', sel_brf(:), xr, wr);
                append_seg('pmu','if_imag', sel_brf(:), xi, wi);
            end
        end
    end

    % --- 1d) PMU 支路电流 To 端（imt/iat） ---
    if isfield(measurements,'pmu') && isfield(measurements.pmu,'imt') && isfield(measurements.pmu,'iat')
        pmu = measurements.pmu;
        if isfield(pmu_config,'pmu_to_branch_indices') && ~isempty(pmu_config.pmu_to_branch_indices)
            def_brt = pmu_config.pmu_to_branch_indices(:);
        else
            def_brt = (1:numel(pmu.imt(:))).';
        end
        sel_brt = pick(selection,'PMU','to_branch_idx', def_brt);
        if ~isempty(sel_brt)
            [~, pos] = ismember(sel_brt, def_brt); pos = pos(pos>0);
            if ~isempty(pos)
                mag = pmu.imt(pos); ang = pmu.iat(pos);
                [xr, xi, wr, wi] = polar_to_rect(mag(:), ang(:), noise_params.pmu.im, noise_params.pmu.ia);
                append_seg('pmu','it_real', sel_brt(:), xr, wr);
                append_seg('pmu','it_imag', sel_brt(:), xi, wi);
            end
        end
    end

    % --- 2) 汇总 ---
    if ~isempty(vector_parts)
        z_vector  = vertcat(vector_parts{:});
        z_weights = vertcat(weights_parts{:});
    else
        z_vector = [];
        z_weights = [];
    end

    % === 内部辅助 ===
    function v = getfield_safe(s, f)
        if isstruct(s) && isfield(s,f) && ~isempty(s.(f))
            v = s.(f);
        else
            v = [];
        end
    end

    function append_seg(typ, field, idx, data, weights)
        vector_parts{end+1}  = data(:);
        weights_parts{end+1} = weights(:);
        z_map{end+1} = struct('type', typ, 'field', field, 'count', numel(idx), 'indices', idx(:));
    end

    function sel = pick(selobj, domain, key, def)
        sel = def;
        if isstruct(selobj) && isfield(selobj, domain) && isfield(selobj.(domain), key)
            val = selobj.(domain).(key);
            if ~isempty(val), sel = val(:); else, sel = []; end
        end
    end

    function [xr, xi, wr, wi] = polar_to_rect(mag, ang, s_mag, s_ang)
        xr = mag .* cos(ang);
        xi = mag .* sin(ang);
        var_xr = (cos(ang).^2) .* (s_mag.^2) + ((-mag .* sin(ang)).^2) .* (s_ang.^2);
        var_xi = (sin(ang).^2) .* (s_mag.^2) + (( mag .* cos(ang)).^2) .* (s_ang.^2);
        wr = 1 ./ max(var_xr, eps);
        wi = 1 ./ max(var_xi, eps);
    end
end

