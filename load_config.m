function config = load_config()
% LOAD_CONFIG - 创建并返回一个包含所有仿真参数的配置结构体
%
%   通过将所有硬编码的参数集中到此文件中，可以轻松地管理和修改
%   整个仿真的行为，而无需更改核心代码。
%
%   输出:
%       config - 包含所有配置参数的结构体

    config = struct();

    % =========================================================================
    %  1. 系统与文件路径配置 (System & File Paths)
    % =========================================================================
    config.System = struct();
    config.System.CaseName = 'case57';                    % MATPOWER 电网案例名称
    % 新增: 根据案例名称自动生成约束文件名
    config.System.ConstraintFile = sprintf('constraints_%s', config.System.CaseName);
    config.System.LoadDataFile = '2-7_week.txt';          % 负荷曲线数据文件
    config.System.LineCapacityFile = 'Ptrmax57.mat';      % 线路容量数据文件
    config.System.ResultsFile = 'attack_simulation_results.mat'; % 保存仿真日志的文件

    % =========================================================================
    %  2. 仿真控制参数 (Simulation Control)
    % =========================================================================
    config.Simulation = struct();
    config.Simulation.VerboseMode = false;                % 是否在子函数中显示详细输出
    config.Simulation.NumScenarios = 96;                  % 总仿真场景数
    config.Simulation.LoadScaleFactor = 0.5295;           % 基础负荷缩放因子

    % =========================================================================
    %  3. 电网模型参数 (Grid Model Parameters)
    % =========================================================================
    config.Grid = struct();
    config.Grid.SlackBusId = 1;                           % 参考节点ID
    config.Grid.PmuBusLocations = [4 10 15 20 24 29 32 37 41 48 54]'; % PMU 安装位置
    config.Grid.VoltageMin = 0.94;                        % 运行约束：最小电压 (p.u.)
    config.Grid.VoltageMax = 1.06;                        % 运行约束：最大电压 (p.u.)

    % =========================================================================
    %  4. 测量噪声参数 (Measurement Noise Parameters)
    % =========================================================================
    config.Noise = struct();
    % --- SCADA 噪声标准差 ---
    config.Noise.scada.v = 0.01;   % 电压幅值 (p.u.)
    config.Noise.scada.p = 0.02;   % 有功功率 (p.u.)
    config.Noise.scada.q = 0.02;   % 无功功率 (p.u.)
    % --- PMU 噪声标准差 ---
    config.Noise.pmu.va = 0.0017;  % 电压相角 (rad)
    config.Noise.pmu.vm = 0.001;   % 电压幅值 (p.u.)
    config.Noise.pmu.ia = 0.0017;  % 电流相角 (rad)
    config.Noise.pmu.im = 0.001;   % 电流幅值 (p.u.)

    % =========================================================================
    %  5. 攻击生成参数 (Attack Generation Parameters)
    % =========================================================================
    config.Attack = struct();
    config.Attack.TargetableLines = [2 3 4 5 6 7 8 9 10 11 12 13 14 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34:40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55:78];
    config.Attack.NumLinesToAttack = 2;                   % 每次攻击的目标线路数量
    config.Attack.OverloadFactor = 1.05;                  % 目标过载率 (例如, 105%)
    config.Attack.NumRestarts = 5;                        % 优化求解器的多起点重启次数
    config.Attack.UnknownFields = {'pt', 'qt'};           % 定义攻击者未知的测量字段
    config.Attack.VoltageDeviationPenalty = 5;           % 攻击优化中电压偏差的惩罚权重 (lambda)
    config.Attack.Solver = 'ipopt';                       % YALMIP 使用的求解器

    % =========================================================================
    %  6. 状态估计算法参数 (State Estimation Parameters)
    % =========================================================================
    config.StateEstimation = struct();
    config.StateEstimation.MaxIter = 20;                  % WLS 最大迭代次数
    config.StateEstimation.Tolerance = 1e-5;              % WLS 收敛容忍度
    config.StateEstimation.ConfidenceLevel = 0.95;        % 坏数据检测的置信水平
    config.StateEstimation.RegularizationFactor = 1e-8;   % 增益矩阵的正则化因子，防止奇异

end