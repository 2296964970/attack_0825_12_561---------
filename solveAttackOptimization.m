function best_sol = solveAttackOptimization(Constraints, Objective, optimization_vars, mpc, attack_params, verbose)
% solveAttackOptimization: 使用多起点策略求解攻击优化问题
%
% 输入:
%   Constraints         - YALMIP 约束
%   Objective           - YALMIP 目标函数
%   optimization_vars   - 包含YALMIP优化变量的结构体
%   mpc                 - MATPOWER案例 (用于获取总线数)
%   attack_params       - 攻击参数 (重启次数, 求解器等)
%   verbose             - 是否显示详细输出
%
% 输出:
%   best_sol (struct):
%     .found      (boolean) - 是否找到解
%     .obj        (double)  - 找到的最优目标函数值
%     .a          (vector)  - 最优攻击向量
%     .Vc         (vector)  - 最优虚假电压幅值
%     .uc         (vector)  - 最优虚假电压相角
%     .diag       (struct)  - YALMIP 求解诊断信息

if nargin < 6, verbose = false; end

num_buses = size(mpc.bus, 1);

% 从结构体中解包优化变量
V_compromised = optimization_vars.V_compromised;
theta_compromised = optimization_vars.theta_compromised;
attack_vector_a = optimization_vars.attack_vector_a;

if verbose, fprintf('开始使用多起点策略求解优化问题...\n'); end

% 获取重启次数，如果未定义则默认为1
num_restarts = 1;
if isfield(attack_params, 'NumRestarts') && attack_params.NumRestarts > 1
    num_restarts = attack_params.NumRestarts;
    if verbose, fprintf('将执行 %d 次重启以寻找更优的解。\n', num_restarts); end
end

yalmip_verbose_level = 0;
if verbose, yalmip_verbose_level = 1; end
solver_options = sdpsettings('verbose', yalmip_verbose_level, 'solver', attack_params.Solver);

best_sol = struct('found', false, 'obj', inf, 'a', [], 'Vc', [], 'uc', [], 'diag', []);

for k = 1:num_restarts
    if verbose && num_restarts > 1, fprintf('\n--- 重启 %d / %d ---\n', k, num_restarts); end
    
    % 设置随机初始值以探索不同的求解路径
    assign(V_compromised, attack_params.VoltageMin + (attack_params.VoltageMax - attack_params.VoltageMin).*rand(num_buses, 1));
    assign(theta_compromised, -pi + 2*pi*rand(num_buses, 1));
    
    diagnostics = optimize(Constraints, Objective, solver_options);
    
    if diagnostics.problem == 0 % 求解成功
        current_obj = double(Objective);
        if verbose, fprintf('尝试 %d: 求解成功, 目标函数值: %.6f\n', k, current_obj); end
        
        if current_obj < best_sol.obj
            if verbose, fprintf('找到更优解! 旧:%.6f, 新:%.6f\n', best_sol.obj, current_obj); end
            best_sol.found = true;
            best_sol.obj = current_obj;
            best_sol.a = double(attack_vector_a);
            best_sol.Vc = double(V_compromised);
            best_sol.uc = double(theta_compromised);
            best_sol.diag = diagnostics;
        end
    elseif verbose
        fprintf('尝试 %d: 求解失败. YALMIP: %s\n', k, diagnostics.info);
    end
end

if ~best_sol.found && verbose
    fprintf('在 %d 次尝试中均未找到可行解。\n', num_restarts);
end

end