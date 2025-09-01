function best_sol = solveAttackOptimization(Constraints, Objective, optimization_vars, mpc, attack_params, verbose)
% solveAttackOptimization 使用多起点策略调用 YALMIP 求解器并选优。
%
% 输入
% - Constraints, Objective: YALMIP 模型
% - optimization_vars      : sdpvar 变量结构体
% - mpc                    : MATPOWER 案例（用于尺寸）
% - attack_params          : 求解参数（重启次数、求解器名等）
% - verbose                : 是否打印日志
%
% 输出
% - best_sol: struct {found,obj,a,Vc,uc,diag}

if nargin < 6, verbose = false; end

% 基础依赖检查（给出更清晰的错误提示）
assert(exist('sdpsettings','file')==2 && exist('optimize','file')==2, ...
    'YALMIP 未就绪：请确保已安装并在路径上（yalmiptest）。');

num_buses = size(mpc.bus, 1);

V_compromised     = optimization_vars.V_compromised;
theta_compromised = optimization_vars.theta_compromised;
attack_vector_a   = optimization_vars.attack_vector_a;

if verbose, fprintf('开始使用多起点策略求解优化问题...\n'); end

num_restarts = 1;
if isfield(attack_params, 'NumRestarts') && attack_params.NumRestarts > 1
    num_restarts = attack_params.NumRestarts;
    if verbose, fprintf('将执行 %d 次重启以寻找更优解。\n', num_restarts); end
end

yalmip_verbose_level = double(logical(verbose));
% 构造求解器选项
solver_name = '';
if isfield(attack_params,'Solver') && ~isempty(attack_params.Solver)
    solver_name = attack_params.Solver;
end
solver_options = sdpsettings('verbose', yalmip_verbose_level, 'solver', solver_name);

best_sol = struct('found', false, 'obj', inf, 'a', [], 'Vc', [], 'uc', [], 'diag', []);

for k = 1:num_restarts
    if verbose && num_restarts > 1, fprintf('\n--- 重启 %d / %d ---\n', k, num_restarts); end

    % 随机初始化（在运行域内）
    assign(V_compromised, attack_params.VoltageMin + (attack_params.VoltageMax - attack_params.VoltageMin).*rand(num_buses, 1));
    assign(theta_compromised, -pi + 2*pi*rand(num_buses, 1));

    % 主求解
    diag = optimize(Constraints, Objective, solver_options);
    
    % 可选回退：若指定求解器失败，尝试 fmincon（若可用）
    if diag.problem ~= 0 && ~strcmpi(solver_name,'fmincon')
        try
            alt_opts = sdpsettings('verbose', yalmip_verbose_level, 'solver', 'fmincon');
            diag_alt = optimize(Constraints, Objective, alt_opts);
            if diag_alt.problem == 0
                diag = diag_alt;
            end
        catch
            % 忽略回退失败，保留原诊断
        end
    end
    if diag.problem == 0
        obj = double(Objective);
        if verbose, fprintf('尝试 %d: 求解成功, 目标值 = %.6f\n', k, obj); end
        if obj < best_sol.obj
            if verbose, fprintf('找到更优: 由 %.6f -> %.6f\n', best_sol.obj, obj); end
            best_sol.found = true;
            best_sol.obj = obj;
            best_sol.a  = double(attack_vector_a);
            best_sol.Vc = double(V_compromised);
            best_sol.uc = double(theta_compromised);
            best_sol.diag = diag;
        end
    else
        if verbose, fprintf('尝试 %d: 求解失败. YALMIP: %s\n', k, diag.info); end
    end
end

if ~best_sol.found && verbose
    fprintf('在 %d 次尝试中未找到可行解。\n', num_restarts);
end

end
