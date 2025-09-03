function [fullModel, registry, map] = buildFullMeasurementModel(mpc, layout, noiseCfg, options)
% buildFullMeasurementModel 以“全量优先”的方式构建测量模型（非兼容新接口）
%
% 输入
% - mpc     : MATPOWER 案例结构
% - layout  : 测量布局（PMU 安装母线/支路等），如：
%             layout.PmuBusLocations, layout.PmuFromBranchIdx, layout.PmuToBranchIdx
% - noiseCfg: 噪声/权重参数（config.Noise）
% - options : 结构体，需包含 options.slack_bus_id（缺省=1）
%
% 输出
% - fullModel: 结构体，包含
%     .evaluate(x) -> [h, H]  按全量顺序返回 h(x)/H(x)
%     .R, .W       -> 对角方差/权重（稀疏）
%     .dim         -> 测量维度
%     .num_state   -> 状态维度（2*nbus-1）
%     .groups      -> 分组索引（便于快速计算）
%     .internals   -> 预计算导纳矩阵等
%     .registry    -> 与输出 registry 同步的只读引用
% - registry : 全量测量注册表（稳定顺序与元数据）
% - map      : 等同 registry 的轻量映射视图（便于替换旧 map 用法）

    if nargin < 4 || isempty(options)
        options = struct();
    end
    if ~isfield(options, 'slack_bus_id') || isempty(options.slack_bus_id)
        options.slack_bus_id = 1;
    end

    % 1) 注册表：定义全量测量的唯一顺序与元数据
    registry = makeMeasurementRegistry(mpc, layout, noiseCfg);

    % 2) 方差/权重（对角）
    nmeas = numel(registry);
    var_diag = zeros(nmeas, 1);
    for i = 1:nmeas
        v = registry(i).noise_var;
        if isempty(v) || ~isfinite(v) || v <= 0
            v = 1e-6; % 最小方差保护
        end
        var_diag(i) = v;
    end
    R = spdiags(var_diag, 0, nmeas, nmeas);
    W = spdiags(1./max(var_diag, eps), 0, nmeas, nmeas);

    % 3) 预计算导纳与索引
    define_constants;
    nb = size(mpc.bus, 1);
    [Ybus, Yf, Yt] = makeYbus(mpc.baseMVA, mpc.bus, mpc.branch);
    slack = options.slack_bus_id;
    f_indices = setdiff((1:nb).', slack);
    num_state = nb + numel(f_indices); % e(1..nb) + f(wo slack)

    internals = struct();
    internals.nb = nb;
    internals.branch_from_bus = mpc.branch(:, F_BUS);
    internals.branch_to_bus   = mpc.branch(:, T_BUS);
    internals.Gbus = real(Ybus);  internals.Bbus = imag(Ybus);
    internals.Gf   = real(Yf);    internals.Bf   = imag(Yf);
    internals.Gt   = real(Yt);    internals.Bt   = imag(Yt);
    internals.slack = slack;
    internals.f_indices = f_indices;

    % 4) 建立分组索引（行号与对象索引）
    groups = buildGroupsFromRegistry(registry);

    % 5) 输出模型
    fullModel = struct();
    fullModel.evaluate = @(x) evaluateFullModel(x, internals, groups, registry, num_state);
    fullModel.R = R;
    fullModel.W = W;
    fullModel.dim = nmeas;
    fullModel.num_state = num_state;
    fullModel.groups = groups;
    fullModel.internals = internals;
    fullModel.registry = registry;

    % 轻量 map：与 registry 一致，但仅保留必要字段
    map = arrayfun(@(r) struct('id', r.id, 'domain', r.domain, 'type', r.type, ...
        'kind', r.kind, 'index', r.index, 'side', r.side, 'row', r.row), registry);
end

function groups = buildGroupsFromRegistry(registry)
    % 基于 registry 按类型聚合，便于一次性批量赋值
    groups = struct();
    fields = { ...
        'SCADA_v','SCADA_pi','SCADA_qi', ...
        'SCADA_pf','SCADA_qf','SCADA_pt','SCADA_qt', ...
        'PMU_v_real','PMU_v_imag', ...
        'PMU_if_real','PMU_if_imag','PMU_it_real','PMU_it_imag' };
    for k = 1:numel(fields)
        groups.(fields{k}) = struct('rows', [], 'idx', []);
    end
    for i = 1:numel(registry)
        r = registry(i);
        key = '';
        switch r.domain
            case 'SCADA'
                switch r.type
                    case 'v',  key = 'SCADA_v';
                    case 'pi', key = 'SCADA_pi';
                    case 'qi', key = 'SCADA_qi';
                    case 'pf', key = 'SCADA_pf';
                    case 'qf', key = 'SCADA_qf';
                    case 'pt', key = 'SCADA_pt';
                    case 'qt', key = 'SCADA_qt';
                end
            case 'PMU'
                switch r.type
                    case 'v_real',  key = 'PMU_v_real';
                    case 'v_imag',  key = 'PMU_v_imag';
                    case 'if_real', key = 'PMU_if_real';
                    case 'if_imag', key = 'PMU_if_imag';
                    case 'it_real', key = 'PMU_it_real';
                    case 'it_imag', key = 'PMU_it_imag';
                end
        end
        if ~isempty(key)
            if strcmp(r.kind, 'bus')
                idx = r.index;
            else
                idx = r.index; % branch index
            end
            groups.(key).rows(end+1,1) = r.row;
            groups.(key).idx(end+1,1)  = idx;
        end
    end
end

function [h, H] = evaluateFullModel(x, it, g, registry, num_state)
    % 计算全量 h(x) / H(x)（直角坐标，SCADA 非线性，PMU 线性）
    nb = it.nb;
    e = x(1:nb);
    f = zeros(nb,1);
    f(it.f_indices) = x(nb+1:end);

    Gbus = it.Gbus;  Bbus = it.Bbus;
    Gf   = it.Gf;    Bf   = it.Bf;
    Gt   = it.Gt;    Bt   = it.Bt;

    % 电流/功率基础量
    I_real_inj = Gbus*e - Bbus*f;
    I_imag_inj = Bbus*e + Gbus*f;
    If_real = Gf*e   - Bf*f;
    If_imag = Bf*e   + Gf*f;
    It_real = Gt*e   - Bt*f;
    It_imag = Bt*e   + Gt*f;

    % Jacobian 快速矩阵（与 calculate_h_H 保持一致的形式）
    V = e + 1j*f;
    Ybus = Gbus + 1j*Bbus;
    Yf   = Gf   + 1j*Bf;
    Yt   = Gt   + 1j*Bt;

    I_inj = Ybus * V;
    dS_inj_dV = diag(conj(I_inj));
    dS_inj_dV_conj = diag(V) * conj(Ybus);
    dS_inj_de = dS_inj_dV + dS_inj_dV_conj;
    dS_inj_df = 1j * (dS_inj_dV - dS_inj_dV_conj);
    dPi_de_mat = real(dS_inj_de); dPi_df_mat = real(dS_inj_df);
    dQi_de_mat = imag(dS_inj_de); dQi_df_mat = imag(dS_inj_df);

    nbrr = numel(it.branch_from_bus);
    If = Yf * V; It = Yt * V;
    dSf_dV = sparse(1:nbrr, it.branch_from_bus, conj(If), nbrr, nb);
    dSf_dV_conj = diag(V(it.branch_from_bus)) * conj(Yf);
    dSf_de = dSf_dV + dSf_dV_conj; dSf_df = 1j*(dSf_dV - dSf_dV_conj);
    dPf_de_mat = real(dSf_de); dPf_df_mat = real(dSf_df);
    dQf_de_mat = imag(dSf_de); dQf_df_mat = imag(dSf_df);

    dSt_dV = sparse(1:nbrr, it.branch_to_bus, conj(It), nbrr, nb);
    dSt_dV_conj = diag(V(it.branch_to_bus)) * conj(Yt);
    dSt_de = dSt_dV + dSt_dV_conj; dSt_df = 1j*(dSt_dV - dSt_dV_conj);
    dPt_de_mat = real(dSt_de); dPt_df_mat = real(dSt_df);
    dQt_de_mat = imag(dSt_de); dQt_df_mat = imag(dSt_df);

    % f 列映射（slack 的 f 不在状态中）
    f_col_map = zeros(nb,1);
    f_col_map(it.f_indices) = nb + (1:numel(it.f_indices));

    % 输出初始化
    m = numel(registry);
    h = zeros(m,1);
    H = spalloc(m, num_state, 0);

    % SCADA: v
    if ~isempty(g.SCADA_v.rows)
        rows = g.SCADA_v.rows; idx = g.SCADA_v.idx; Vi = hypot(e, f);
        h(rows) = Vi(idx);
        H = assign_diagonal(H, rows, idx, e(idx)./max(Vi(idx), 1e-9));
        fcols = f_col_map(idx); mask = fcols~=0;
        H = assign_entries(H, rows(mask), fcols(mask), f(idx(mask))./max(Vi(idx(mask)),1e-9));
    end
    % SCADA: pi/qi
    if ~isempty(g.SCADA_pi.rows)
        rows = g.SCADA_pi.rows; idx = g.SCADA_pi.idx;
        Pi = e.*I_real_inj + f.*I_imag_inj; h(rows) = Pi(idx);
        H(rows,1:it.nb) = dPi_de_mat(idx,:);
        H(rows,it.nb+1:end) = dPi_df_mat(idx, it.f_indices);
    end
    if ~isempty(g.SCADA_qi.rows)
        rows = g.SCADA_qi.rows; idx = g.SCADA_qi.idx;
        Qi = f.*I_real_inj - e.*I_imag_inj; h(rows) = Qi(idx);
        H(rows,1:it.nb) = dQi_de_mat(idx,:);
        H(rows,it.nb+1:end) = dQi_df_mat(idx, it.f_indices);
    end
    % SCADA: pf/qf/pt/qt
    if ~isempty(g.SCADA_pf.rows)
        rows = g.SCADA_pf.rows; idx = g.SCADA_pf.idx; h(rows) = e(it.branch_from_bus(idx)).*If_real(idx) + f(it.branch_from_bus(idx)).*If_imag(idx);
        H(rows,1:it.nb) = dPf_de_mat(idx,:); H(rows,it.nb+1:end) = dPf_df_mat(idx, it.f_indices);
    end
    if ~isempty(g.SCADA_qf.rows)
        rows = g.SCADA_qf.rows; idx = g.SCADA_qf.idx; h(rows) = f(it.branch_from_bus(idx)).*If_real(idx) - e(it.branch_from_bus(idx)).*If_imag(idx);
        H(rows,1:it.nb) = dQf_de_mat(idx,:); H(rows,it.nb+1:end) = dQf_df_mat(idx, it.f_indices);
    end
    if ~isempty(g.SCADA_pt.rows)
        rows = g.SCADA_pt.rows; idx = g.SCADA_pt.idx; h(rows) = e(it.branch_to_bus(idx)).*It_real(idx) + f(it.branch_to_bus(idx)).*It_imag(idx);
        H(rows,1:it.nb) = dPt_de_mat(idx,:); H(rows,it.nb+1:end) = dPt_df_mat(idx, it.f_indices);
    end
    if ~isempty(g.SCADA_qt.rows)
        rows = g.SCADA_qt.rows; idx = g.SCADA_qt.idx; h(rows) = f(it.branch_to_bus(idx)).*It_real(idx) - e(it.branch_to_bus(idx)).*It_imag(idx);
        H(rows,1:it.nb) = dQt_de_mat(idx,:); H(rows,it.nb+1:end) = dQt_df_mat(idx, it.f_indices);
    end

    % PMU: v_real/v_imag（线性）
    if ~isempty(g.PMU_v_real.rows)
        rows = g.PMU_v_real.rows; idx = g.PMU_v_real.idx; h(rows) = e(idx);
        H = assign_entries(H, rows, idx, 1);
    end
    if ~isempty(g.PMU_v_imag.rows)
        rows = g.PMU_v_imag.rows; idx = g.PMU_v_imag.idx; h(rows) = f(idx);
        fcols = f_col_map(idx); mask = fcols~=0; rows2 = rows(mask); fcols2 = fcols(mask);
        H = assign_entries(H, rows2, fcols2, 1);
    end
    % PMU: if_/it_（线性）
    if ~isempty(g.PMU_if_real.rows)
        rows = g.PMU_if_real.rows; idx = g.PMU_if_real.idx; h(rows) = If_real(idx);
        H(rows,1:it.nb) = Gf(idx,:); H(rows,it.nb+1:end) = -Bf(idx, it.f_indices);
    end
    if ~isempty(g.PMU_if_imag.rows)
        rows = g.PMU_if_imag.rows; idx = g.PMU_if_imag.idx; h(rows) = If_imag(idx);
        H(rows,1:it.nb) = Bf(idx,:); H(rows,it.nb+1:end) =  Gf(idx, it.f_indices);
    end
    if ~isempty(g.PMU_it_real.rows)
        rows = g.PMU_it_real.rows; idx = g.PMU_it_real.idx; h(rows) = It_real(idx);
        H(rows,1:it.nb) = Gt(idx,:); H(rows,it.nb+1:end) = -Bt(idx, it.f_indices);
    end
    if ~isempty(g.PMU_it_imag.rows)
        rows = g.PMU_it_imag.rows; idx = g.PMU_it_imag.idx; h(rows) = It_imag(idx);
        H(rows,1:it.nb) = Bt(idx,:); H(rows,it.nb+1:end) =  Gt(idx, it.f_indices);
    end
end

function H = assign_entries(H, rows, cols, vals)
    % 将标量或等长向量填入 (rows, cols)
    if isscalar(vals)
        vals = repmat(vals, numel(rows), 1);
    end
    H = H + sparse(rows, cols, vals, size(H,1), size(H,2));
end

function H = assign_diagonal(H, rows, cols, diagvals)
    H = H + sparse(rows, cols, diagvals, size(H,1), size(H,2));
end

