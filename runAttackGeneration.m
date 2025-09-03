function attack = runAttackGeneration(selectedModel, y, mpc, opf_results, config)
% runAttackGeneration（重构版）: 基于 selectedModel/y（向量语义）生成隐蔽 FDIA
% 输出攻击后的测量向量 y_att（同维），以及解的诊断信息。

    verbose = isfield(config.Simulation,'VerboseMode') && config.Simulation.VerboseMode;
    if verbose, fprintf('\n--- 开始生成攻击向量（重构版）---\n'); end

    % 攻击参数准备
    attack_params = config.Attack;
    if isfield(config.System, 'ConstraintFile')
        attack_params.ConstraintFile = config.System.ConstraintFile;
    end
    attack_params.line_capacity_data = load(config.System.LineCapacityFile);
    attack_params.VoltageMin = config.Grid.VoltageMin;
    attack_params.VoltageMax = config.Grid.VoltageMax;

    % 随机选择要攻击的线路
    ids = attack_params.TargetableLines; k = attack_params.NumLinesToAttack;
    pick = randperm(numel(ids), k); attack_params.attacked_lines_indices = ids(pick);

    % 基于 UnknownFields 生成“攻击者已知行”掩码（对应 selectedModel.registry）
    registry = selectedModel.registry;
    known_mask = compute_known_mask(registry, attack_params);
    y_known = y(known_mask);

    % 参考电压幅值（用于目标项）：来自 OPF 结果
    VM = 8; % MATPOWER VM 列索引
    V_ref = opf_results.bus(:, VM);

    % 构建并求解优化
    yalmip('clear');
    [Cons, Obj, vars] = buildAttackModel(mpc, selectedModel, known_mask, y_known, V_ref, attack_params);
    best = solveAttackOptimization(Cons, Obj, vars, mpc, attack_params, verbose);

    % 输出打包
    attack = struct('success', best.found, 'objective', best.obj, 'attacked_lines_indices', attack_params.attacked_lines_indices);
    attack.solver_diagnostics = best.diag;
    if ~best.found
        if verbose, fprintf('攻击向量求解失败。\n'); end
        attack.y_att = [];
        return;
    end

    % 组装攻击后的测量向量（已知行：z+a；未知行：h(x_c)）
    y_att_base = assemble_attacked_vector(selectedModel, known_mask, y, best);

    % 按测量噪声模型为攻击后量测再加噪（与原实现一致的后处理）
    vdiag = full(diag(selectedModel.R));
    if isrow(vdiag), vdiag = vdiag.'; end
    eps_noise = randn(numel(vdiag), 1) .* sqrt(max(vdiag, 0));
    attack.y_att = y_att_base + eps_noise;
    attack.Vc = best.Vc; attack.uc = best.uc; attack.a = best.a;
end

function known_mask = compute_known_mask(registry, attack_params)
    unknown = {};
    if isfield(attack_params,'UnknownFields'), unknown = attack_params.UnknownFields; end
    unknown = cellfun(@(s) string(s), unknown, 'UniformOutput', false);
    unknown = upper(string([unknown{:}]));
    n = numel(registry);
    known_mask = true(n,1);
    for i=1:n
        r = registry(i);
        if r.domain == "SCADA"
            if any(upper(unknown) == upper(string(r.type)))
                known_mask(i) = false; continue;
            end
        else % PMU: 将 vm/va/imf/iaf/imt/iat 同步到直角分量
            switch r.type
                case {'v_real','v_imag'}
                    hit = any(upper(unknown)=="VM") | any(upper(unknown)=="VA");
                case {'if_real','if_imag'}
                    hit = any(upper(unknown)=="IMF") | any(upper(unknown)=="IAF");
                case {'it_real','it_imag'}
                    hit = any(upper(unknown)=="IMT") | any(upper(unknown)=="IAT");
                otherwise
                    hit = false;
            end
            if hit, known_mask(i) = false; end
        end
    end
end

function y_att = assemble_attacked_vector(selectedModel, known_mask, y, best)
    % h(x_c) 全量计算（按 registry 顺序）
    Vc = best.Vc; uc = best.uc;
    vcomp = Vc .* exp(1j*uc);

    it = selectedModel.internals; reg = selectedModel.registry;
    Ybus = it.Gbus + 1j*it.Bbus; Yf = it.Gf + 1j*it.Bf; Yt = it.Gt + 1j*it.Bt;
    I_from = Yf * vcomp; I_to = Yt * vcomp; S_inj = vcomp .* conj(Ybus * vcomp);
    S_from = vcomp(it.branch_from_bus) .* conj(I_from);
    S_to   = vcomp(it.branch_to_bus)   .* conj(I_to);

    m = numel(reg);
    h_c = zeros(m,1);
    for i=1:m
        r = reg(i);
        switch r.domain
            case 'SCADA'
                switch r.type
                    case 'v',  h_c(i) = abs(vcomp(r.index));
                    case 'pi', h_c(i) = real(S_inj(r.index));
                    case 'qi', h_c(i) = imag(S_inj(r.index));
                    case 'pf', h_c(i) = real(S_from(r.index));
                    case 'qf', h_c(i) = imag(S_from(r.index));
                    case 'pt', h_c(i) = real(S_to(r.index));
                    case 'qt', h_c(i) = imag(S_to(r.index));
                end
            case 'PMU'
                switch r.type
                    case 'v_real',  h_c(i) = Vc(r.index)*cos(uc(r.index));
                    case 'v_imag',  h_c(i) = Vc(r.index)*sin(uc(r.index));
                    case 'if_real', h_c(i) = real(I_from(r.index));
                    case 'if_imag', h_c(i) = imag(I_from(r.index));
                    case 'it_real', h_c(i) = real(I_to(r.index));
                    case 'it_imag', h_c(i) = imag(I_to(r.index));
                end
        end
    end

    y_att = h_c;                 % 默认未知行取 h(x_c)
    idx_known = find(known_mask);
    y_att(idx_known) = y(idx_known) + best.a; % 已知行 z+a
end
