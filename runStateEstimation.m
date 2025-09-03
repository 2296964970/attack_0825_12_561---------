function [state, stats] = runStateEstimation(selectedModel, y, config)
% runStateEstimation 新接口：基于“已切片”的 selectedModel 与观测向量 y 进行 WLS + LNR-BDD
%
% 输入
% - selectedModel : sliceModel 返回的子模型（包含 evaluate/R/W/registry/internals）
% - y             : 观测向量（与 selectedModel.dim 一致）
% - config        : 全局配置（StateEstimation.*）
%
% 输出
% - state : struct，包含 e/f/V/theta
% - stats : struct，包含迭代/残差/BDD 结果等

    verbose = false; if isfield(config,'Simulation') && isfield(config.Simulation,'VerboseMode'), verbose = config.Simulation.VerboseMode; end

    opts.max_iter = config.StateEstimation.MaxIter;
    opts.tolerance = config.StateEstimation.Tolerance;
    opts.confidence_level = config.StateEstimation.ConfidenceLevel;
    opts.regularization_factor = config.StateEstimation.RegularizationFactor;

    nb = selectedModel.internals.nb;
    f_indices = selectedModel.internals.f_indices;

    % 初始点：平坦 + PMU 电压（若有）
    e0 = ones(nb,1); f0 = zeros(nb,1);
    reg = selectedModel.registry;
    for i = 1:numel(reg)
        if strcmp(reg(i).domain,'PMU')
            switch reg(i).type
                case 'v_real', if i <= numel(y), e0(reg(i).index) = y(i); end
                case 'v_imag', if i <= numel(y), f0(reg(i).index) = y(i); end
            end
        end
    end
    x = [e0; f0(f_indices)];

    % 预取权重对角（强制列向量）
    wdiag = full(diag(selectedModel.W));
    if isrow(wdiag), wdiag = wdiag.'; end

    if verbose, fprintf('\n--- 状态估计(WLS)开始 ---\n'); end
    stats.iter = -1; H_last = []; r_last = []; dx = 0;
    for k = 1:opts.max_iter
        [h, H] = selectedModel.evaluate(x);
        r = y - h;

        % 等价形式：A = sqrt(W)*H, b = sqrt(W)*r
        m = size(H,1);
        sw = sqrt(max(wdiag(1:m), 0));
        A = spdiags(sw(:), 0, m, m) * H;
        b = sw(:) .* r;

        % 正规方程备用
        Wdiag = spdiags(wdiag(1:m), 0, m, m);
        G = H' * (Wdiag * H);
        rhs = H' * (wdiag(1:m) .* r);

        try
            if exist('lsqminnorm','builtin') || exist('lsqminnorm','file') == 2
                dx = lsqminnorm(A, b);
            else
                dx = A \ b;
            end
        catch
            % 对稀疏矩阵使用 rcond 需先 full 化，或使用 condest
            rc = rcond(full(G));
            if rc < 1e-10
                dmax = max(1.0, full(max(abs(diag(G)))));
                tau = max([opts.regularization_factor, 1e-6, 1e-3 * dmax]);
                if rc < 1e-14, tau = max(tau, 1e-2 * dmax); end
                G = G + tau * speye(size(G));
            end
            dx = G \ rhs;
        end

        x = x + dx;
        mx = max(abs(dx));
        if verbose, fprintf('  - 迭代 %2d: max|Δx| = %.2e\n', k, mx); end
        H_last = H; r_last = r; stats.iter = k;
        if mx < opts.tolerance, break; end
    end

    % 输出状态
    e = x(1:nb);
    f = zeros(nb,1); f(f_indices) = x(nb+1:end);
    state = struct();
    state.e = e; state.f = f; state.V = hypot(e,f); state.theta = atan2(f,e);

    % BDD: LNR 统计量
    m = size(H_last,1);
    Wdiag = spdiags(wdiag(1:m), 0, m, m);
    G = H_last' * (Wdiag * H_last);
    % rcond 不直接支持稀疏，这里转为 full 判断数值稳定性
    rc = rcond(full(G));
    if rc < 1e-10
        dmax = max(1.0, full(max(abs(diag(G)))));
        tau = max([opts.regularization_factor, 1e-6, 1e-3 * dmax]);
        if rc < 1e-14, tau = max(tau, 1e-2 * dmax); end
        G = G + tau * speye(size(G));
    end
    Ginv = pinv(full(G), 1e-10);
    HG = H_last * Ginv;
    diag_HGHT = sum(HG .* H_last, 2);
    Rdiag = 1 ./ max(wdiag(1:m), 1e-18);
    diag_cov = abs(Rdiag - diag_HGHT);
    diag_cov(diag_cov < 1e-9) = 1e-9;

    nr = abs(r_last) ./ sqrt(diag_cov);
    lnr = max(nr);
    m = numel(y); alpha = 1 - opts.confidence_level; alpha_corr = alpha / m;
    critical = sqrt(2) * erfcinv(alpha_corr);

    stats.residual = r_last;
    stats.maxNormalizedResidual = lnr;
    stats.criticalValue = critical;
    stats.detectionFlag = (lnr > critical);
    stats.converged = (stats.iter > 0) && (max(abs(dx)) < opts.tolerance);
    stats.H = H_last;

    if verbose
        fprintf('--> LNR = %.4f, 阈值(%.0f%%) = %.4f，%s\n', lnr, opts.confidence_level*100, critical, ...
            ternary(stats.detectionFlag,'报警','正常'));
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
