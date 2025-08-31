function attack_results = runAttackGeneration(mpc, attacker_measurements, system_measurements, pmu_config, config)
% runAttackGeneration: 生成隐蔽的虚假数据注入攻击（重构后的健壮版本）
%
% 输入:
%   mpc                   - MATPOWER案例结构体
%   attacker_measurements - 攻击者已知的测量值 (用于优化)
%   system_measurements   - 完整的系统测量值 (作为攻击后结果的蓝本)
%   pmu_config            - PMU配置信息
%   config                - 包含所有配置参数的结构体
%
% 输出:
%   attack_results (struct):
%     .success                (boolean) - 攻击是否成功找到解
%     .attacked_measurements  (struct)  - 包含了伪造测量值的新结构体
%     .attack_vector          (vector)  - 注入的攻击向量 a
%     .compromised_state      (struct)  - 攻击得出的虚假电网状态 (Vc, uc)
%     .solver_diagnostics     (struct)  - YALMIP求解器的诊断信息
%     .attacked_lines_indices (vector)  - 本次运行攻击的目标线路索引

%% --- 1. 初始化与设置 (INITIALIZATION & SETUP) ---
verbose = config.Simulation.VerboseMode;
if verbose, fprintf('\n--- 开始生成攻击向量 ---\n'); end

% 定义常量并提取必要的电网参数
define_constants;
[baseMVA, bus, ~, branch] = deal(mpc.baseMVA, mpc.bus, mpc.gen, mpc.branch);
[Y_bus, Y_from, Y_to] = makeYbus(baseMVA, bus, branch);

% 初始化输出结构体
attack_results = struct('success', false, 'objective', NaN, 'attacked_lines_indices', []);

%% --- 2. 准备攻击参数 (PREPARE ATTACK PARAMETERS) ---
if verbose, fprintf('正在准备攻击参数...\n'); end
attack_params = config.Attack;

% 将系统特定的约束文件配置传递给 attack_params
if isfield(config.System, 'ConstraintFile')
    attack_params.ConstraintFile = config.System.ConstraintFile;
end
noise_params = config.Noise;

% 注入外部数据和配置到攻击参数中
attack_params.line_capacity_data = load(config.System.LineCapacityFile);
attack_params.VoltageMin = config.Grid.VoltageMin;
attack_params.VoltageMax = config.Grid.VoltageMax;

% 随机选择要攻击的线路
attack_target_indices = randperm(length(attack_params.TargetableLines), attack_params.NumLinesToAttack);
attack_params.attacked_lines_indices = attack_params.TargetableLines(attack_target_indices);
attack_results.attacked_lines_indices = attack_params.attacked_lines_indices; % 提前存入输出

%% --- 3. 动态测量处理 (DYNAMIC MEASUREMENT PROCESSING) ---
if verbose, fprintf('正在向量化测量数据...\n'); end
[z_vector, attacker_measurement_map, ~] = vectorizeAndMapMeasurements(attacker_measurements, pmu_config, noise_params, struct('num_buses', size(bus,1), 'num_branches', size(mpc.branch,1)));
num_measurements = length(z_vector);
if verbose, fprintf('测量向量化完成, 总测量数量: %d\n', num_measurements); end

%% --- 4. 优化问题构建 (OPTIMIZATION PROBLEM FORMULATION) ---
if verbose, fprintf('正在构建YALMIP攻击模型...\n'); end
yalmip('clear');
[Constraints, Objective, optimization_vars] = buildAttackModel(mpc, attacker_measurements, pmu_config, attack_params, attacker_measurement_map, z_vector);

%% --- 5. 求解优化问题 (SOLVE OPTIMIZATION PROBLEM) ---
if verbose, fprintf('正在调用优化求解器模块...\n'); end
best_sol = solveAttackOptimization(Constraints, Objective, optimization_vars, mpc, attack_params, verbose);

%% --- 6. 结果处理与输出 (PROCESS & FORMAT RESULTS) ---
attack_results.solver_diagnostics = best_sol.diag;

if best_sol.found
    if verbose, fprintf('\n多起点优化完成。最终最优目标: %.6f\n', best_sol.obj); end
    
    % 填充成功的结果
    attack_results.success = true;
    attack_results.objective = best_sol.obj;
    attack_results.attack_vector = best_sol.a;
    attack_results.compromised_state.V = best_sol.Vc;
    attack_results.compromised_state.theta = best_sol.uc;
    
    % 构建完整的攻击后测量值
    attacked_measurements = buildAttackedMeasurements(system_measurements, attacker_measurements, best_sol, attacker_measurement_map, mpc, pmu_config, config);
    attack_results.attacked_measurements = attacked_measurements;
    
else
    if verbose, fprintf('攻击向量求解失败。在 %d 次尝试中均未找到可行解。\n', attack_params.NumRestarts); end
    attack_results.success = false;
end

end
