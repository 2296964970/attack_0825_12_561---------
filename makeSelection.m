function [sel, S, registry_sel] = makeSelection(registry, selectionSpec)
% makeSelection 将选择规格（规则式/显式ID）转为逻辑索引与稀疏选择矩阵
%
% 输入
% - registry     : 全量测量注册表（结构体数组）
% - selectionSpec: struct，可包含 include/exclude 两部分
%   规则字段：domain, type, kind, side, bus, branch, id
%   也支持 include.id = {'SCADA:pt:br_3_to', ...}
%
% 输出
% - sel         : n×1 逻辑索引
% - S           : 稀疏选择矩阵，使得 y_sel = S * y_full
% - registry_sel: 切片后的注册表（行号更新为新顺序）

    n = numel(registry);
    if nargin < 2 || isempty(selectionSpec)
        sel = true(n,1); S = speye(n); registry_sel = registry; return;
    end

    % include 为空 => 全包含；exclude 为空 => 不排除
    if isfield(selectionSpec, 'include') && ~isempty(selectionSpec.include)
        inc = build_mask(registry, selectionSpec.include);
    else
        inc = true(n,1);
    end
    if isfield(selectionSpec, 'exclude') && ~isempty(selectionSpec.exclude)
        exc = build_mask(registry, selectionSpec.exclude);
    else
        exc = false(n,1);
    end
    sel = inc & ~exc;

    idx = find(sel);
    S = sparse(1:numel(idx), idx, 1, numel(idx), n);
    registry_sel = registry(sel);
    for k = 1:numel(registry_sel)
        registry_sel(k).row = k; % 重置行为新顺序
    end
end

function m = build_mask(registry, spec)
    n = numel(registry);
    if isempty(spec)
        m = true(n,1); return;
    end
    m = true(n,1);
    % 字段匹配
    if isfield(spec,'domain') && ~isempty(spec.domain)
        doms = normalize_cellstr(spec.domain);
        m = m & ismember(upper({registry.domain})', doms);
    end
    if isfield(spec,'type') && ~isempty(spec.type)
        tys = normalize_cellstr(spec.type);
        m = m & ismember({registry.type}', tys);
    end
    if isfield(spec,'kind') && ~isempty(spec.kind)
        kinds = normalize_cellstr(spec.kind);
        m = m & ismember({registry.kind}', kinds);
    end
    if isfield(spec,'side') && ~isempty(spec.side)
        sides = normalize_cellstr(spec.side);
        m = m & ismember({registry.side}', sides);
    end
    if isfield(spec,'bus') && ~isempty(spec.bus)
        buses = spec.bus(:)';
        mask = false(n,1);
        for i=1:n
            if strcmp(registry(i).kind,'bus') && ismember(registry(i).index, buses)
                mask(i) = true;
            end
        end
        m = m & mask;
    end
    if isfield(spec,'branch') && ~isempty(spec.branch)
        brs = spec.branch(:)';
        mask = false(n,1);
        for i=1:n
            if strcmp(registry(i).kind,'branch') && ismember(registry(i).index, brs)
                mask(i) = true;
            end
        end
        m = m & mask;
    end
    if isfield(spec,'id') && ~isempty(spec.id)
        ids = normalize_cellstr(spec.id);
        m = m & ismember({registry.id}', ids);
    end
end

function c = normalize_cellstr(v)
    if ischar(v) || isstring(v)
        c = cellstr(string(v));
    else
        c = cellfun(@string, v, 'UniformOutput', false);
        c = cellstr(string([c{:}]));
    end
end

function v = getfield_or(s, f, def)
    if isstruct(s) && isfield(s, f) && ~isempty(s.(f))
        v = s.(f);
    else
        v = def;
    end
end
