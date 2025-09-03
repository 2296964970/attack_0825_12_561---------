function registry = makeMeasurementRegistry(mpc, layout, noiseCfg)
% makeMeasurementRegistry 生成全量测量注册表（稳定顺序 + 元数据 + 方差）
%
% 顺序策略：
%   1) SCADA: v (bus) -> pi (bus) -> qi (bus) -> pf/qf/pt/qt (branch)
%   2) PMU  : v_real/v_imag (bus) -> if_real/if_imag (branch from)
%             -> it_real/it_imag (branch to)

    define_constants;
    nb = size(mpc.bus, 1);
    nl = size(mpc.branch, 1);

    if nargin < 2 || isempty(layout), layout = struct(); end
    if nargin < 3 || isempty(noiseCfg), noiseCfg = struct(); end

    % 默认 PMU 安装位置（可为空）
    pmu_buses = getfield_or(layout, 'PmuBusLocations', []);
    pmu_brf   = getfield_or(layout, 'PmuFromBranchIdx', []);
    pmu_brt   = getfield_or(layout, 'PmuToBranchIdx', []);

    rows = {};
    row = 0;

    % ---- SCADA: bus measurements ----
    % v
    for i = 1:nb
        row = row + 1;
        rows{end+1} = make_row('SCADA','v','bus', i, '', row, scada_var(noiseCfg,'v'));
    end
    % pi
    for i = 1:nb
        row = row + 1;
        rows{end+1} = make_row('SCADA','pi','bus', i, '', row, scada_var(noiseCfg,'p'));
    end
    % qi
    for i = 1:nb
        row = row + 1;
        rows{end+1} = make_row('SCADA','qi','bus', i, '', row, scada_var(noiseCfg,'q'));
    end

    % ---- SCADA: branch measurements ----
    for k = 1:nl
        row = row + 1; rows{end+1} = make_row('SCADA','pf','branch', k, 'from', row, scada_var(noiseCfg,'p'));
        row = row + 1; rows{end+1} = make_row('SCADA','qf','branch', k, 'from', row, scada_var(noiseCfg,'q'));
        row = row + 1; rows{end+1} = make_row('SCADA','pt','branch', k, 'to',   row, scada_var(noiseCfg,'p'));
        row = row + 1; rows{end+1} = make_row('SCADA','qt','branch', k, 'to',   row, scada_var(noiseCfg,'q'));
    end

    % ---- PMU: bus voltage (rectangular) ----
    if ~isempty(pmu_buses)
        [vr_var, vi_var] = pmu_rect_var(noiseCfg, 'v');
        for i = pmu_buses(:)'
            row = row + 1; rows{end+1} = make_row('PMU','v_real','bus', i, '', row, vr_var);
            row = row + 1; rows{end+1} = make_row('PMU','v_imag','bus', i, '', row, vi_var);
        end
    end

    % ---- PMU: branch currents (rectangular, from/to) ----
    if ~isempty(pmu_brf)
        [ir_var, ii_var] = pmu_rect_var(noiseCfg, 'i');
        for k = pmu_brf(:)'
            row = row + 1; rows{end+1} = make_row('PMU','if_real','branch', k, 'from', row, ir_var);
            row = row + 1; rows{end+1} = make_row('PMU','if_imag','branch', k, 'from', row, ii_var);
        end
    end
    if ~isempty(pmu_brt)
        [ir_var, ii_var] = pmu_rect_var(noiseCfg, 'i');
        for k = pmu_brt(:)'
            row = row + 1; rows{end+1} = make_row('PMU','it_real','branch', k, 'to', row, ir_var);
            row = row + 1; rows{end+1} = make_row('PMU','it_imag','branch', k, 'to', row, ii_var);
        end
    end

    registry = vertcat(rows{:});
end

function r = make_row(domain, type, kind, index, side, row, var)
    if strcmp(kind,'bus')
        obj = sprintf('bus_%d', index);
    else
        obj = sprintf('br_%d_%s', index, side);
    end
    r = struct();
    r.id = sprintf('%s:%s:%s', upper(domain), type, obj);
    r.domain = upper(domain);
    r.type = type;
    r.kind = kind;      % 'bus' | 'branch'
    r.index = index;    % bus id 或 branch idx
    r.side = side;      % '' | 'from' | 'to'
    r.row = row;        % 在全量模型中的行号
    r.noise_var = var;  % 对角方差
end

function v = getfield_or(s, f, def)
    if isstruct(s) && isfield(s, f) && ~isempty(s.(f))
        v = s.(f);
    else
        v = def;
    end
end

function v = scada_var(noiseCfg, kind)
    v = 1e-4;
    if isfield(noiseCfg,'scada')
        switch kind
            case 'v'
                if isfield(noiseCfg.scada,'v'), v = max(noiseCfg.scada.v^2, 1e-12); end
            case 'p'
                if isfield(noiseCfg.scada,'p'), v = max(noiseCfg.scada.p^2, 1e-12); end
            case 'q'
                if isfield(noiseCfg.scada,'q'), v = max(noiseCfg.scada.q^2, 1e-12); end
        end
    end
end

function [vr_var, vi_var] = pmu_rect_var(noiseCfg, what)
    % 简化近似：实部/虚部使用相同数量级方差
    switch what
        case 'v'
            sm = getfield_safe(noiseCfg,'pmu','vm', 1e-3);
            sa = getfield_safe(noiseCfg,'pmu','va', 1.7e-3);
        otherwise % 'i'
            sm = getfield_safe(noiseCfg,'pmu','im', 1e-3);
            sa = getfield_safe(noiseCfg,'pmu','ia', 1.7e-3);
    end
    base = sm^2 + sa^2;  % 常量近似
    vr_var = max(base, 1e-12);
    vi_var = max(base, 1e-12);
end

function v = getfield_safe(s, f1, f2, def)
    v = def;
    if isfield(s, f1)
        s1 = s.(f1);
        if isstruct(s1) && isfield(s1, f2) && ~isempty(s1.(f2))
            v = s1.(f2);
        end
    end
end

