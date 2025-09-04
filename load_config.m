function config = load_config()
% LOAD_CONFIG 统一创建并返回本项目的配置结构体。
%
% 目的
% - 将可调参数集中管理，避免分散硬编码，便于排查与复现实验。
%
% 输出
% - config: struct，包含系统/仿真/噪声/攻击/SE 等全部配置项。

    config = struct();

    % =========================================================================
    %  1) 系统与文件路径 (System & Files)
    % =========================================================================
    config.System = struct();
    config.System.CaseName = 'case57';                    % MATPOWER 电网案例名称
    % 根据案例名称自动生成专用约束函数名
    config.System.ConstraintFile = sprintf('constraints_%s', config.System.CaseName);
    config.System.LoadDataFile = '2-7_week.txt';          % 负荷曲线数据文件
    config.System.LineCapacityFile = 'Ptrmax57.mat';      % 线路容量数据文件
    config.System.ResultsFile = 'attack_simulation_results.mat'; % 仿真结果保存文件

    % =========================================================================
    %  2) 仿真控制 (Simulation Control)
    % =========================================================================
    config.Simulation = struct();
    config.Simulation.VerboseMode = false;                % 子函数是否打印详细日志
    config.Simulation.NumScenarios = 96;                  % 总仿真场景数
    config.Simulation.LoadScaleFactor = 0.5295;           % 基础负荷缩放因子

    % =========================================================================
    %  3) 电网模型参数 (Grid)
    % =========================================================================
    config.Grid = struct();
    config.Grid.SlackBusId = 1;                           % 参考节点 ID
    config.Grid.PmuBusLocations = [4 10 15 20 24 29 32 37 41 48 54]'; % PMU 安装母线
    config.Grid.VoltageMin = 0.94;                        % 运行约束: 最小电压 (p.u.)
    config.Grid.VoltageMax = 1.06;                        % 运行约束: 最大电压 (p.u.)

    % =========================================================================
    %  4) 测量噪声 (Measurement Noise)
    % =========================================================================
    config.Noise = struct();
    % SCADA 噪声标准差 (p.u. / rad)
    config.Noise.scada.v = 0.01;   % 节点电压幅值 (p.u.)
    config.Noise.scada.p = 0.02;   % 有功功率 (p.u.)
    config.Noise.scada.q = 0.02;   % 无功功率 (p.u.)
    % PMU 噪声标准差 (p.u. / rad)
    config.Noise.pmu.va = 0.0017;  % 电压相角 (rad)
    config.Noise.pmu.vm = 0.001;   % 电压幅值 (p.u.)
    config.Noise.pmu.ia = 0.0017;  % 电流相角 (rad)
    config.Noise.pmu.im = 0.001;   % 电流幅值 (p.u.)

    % =========================================================================
    %  5) 攻击生成参数 (Attack)
    % =========================================================================
    config.Attack = struct();
    config.Attack.TargetableLines = [2 3 4 5 6 7 8 9 10 11 12 13 14 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34:40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55:78];
    config.Attack.NumLinesToAttack = 2;                   % 每次随机选择的目标线路数量
    config.Attack.OverloadFactor = 1.05;                  % 目标过载比例 (如 1.05 表示 105%)
    config.Attack.NumRestarts = 5;                        % 优化多起点重启次数
    config.Attack.UnknownFields = {};                    % 攻击者未知的测量字段（空表示掌握所有测量）
    config.Attack.VoltageDeviationPenalty = 5;            % 电压偏差惩罚权重 lambda
    config.Attack.Solver = 'ipopt';                       % YALMIP 使用的求解器

    % =========================================================================
    %  6) 量测覆盖选择 (Sparse Measurement Selection)
    % =========================================================================
    % 留空表示使用“全覆盖”。如需稀疏量测，请按字段填入母线/支路索引：
    %   config.MeasurementSelection.SCADA.v_idx          = [1 5 9];
    %   config.MeasurementSelection.SCADA.pf_idx         = [2 7 11];
    %   config.MeasurementSelection.PMU.bus_idx          = [4 10 15];
    %   config.MeasurementSelection.PMU.from_branch_idx  = [3 8];
    %   config.MeasurementSelection.PMU.to_branch_idx    = [5 12];
    % 默认空结构，等价于全覆盖。
    config.MeasurementSelection = struct();

    % =========================================================================
    %  7) 状态估计参数 (State Estimation)
    % =========================================================================
    config.StateEstimation = struct();
    config.StateEstimation.MaxIter = 20;                  % WLS 最大迭代次数
    config.StateEstimation.Tolerance = 1e-5;              % WLS 收敛阈值
    config.StateEstimation.ConfidenceLevel = 0.95;        % 坏数据检测置信水平
    config.StateEstimation.RegularizationFactor = 1e-8;   % 增益矩阵正则化，防止奇异

end

