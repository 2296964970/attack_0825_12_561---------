function demo_baseline_and_sparse()
% 演示脚本：对比“全覆盖(基线)”与“稀疏覆盖(示例)”在SE/攻击建模中的生效情况
% 使用单一场景，打印测量向量维度、SE收敛与残差信息。

clc; fprintf('\n=== DEMO: 基线 vs 稀疏 覆盖 验证 ===\n');

% 1) 加载配置并缩短仿真规模
config = load_config();
config.Simulation.VerboseMode = true;
config.Simulation.NumScenarios = 1;   % 单场景
config.Attack.NumRestarts = 1;        % 减少时间
config.Attack.NumLinesToAttack = 1;

% 2) 生成测量（系统侧仍是全量）
define_constants;
mpc_base = loadcase(config.System.CaseName);
load_multipliers = calculate_rate(config.System.LoadDataFile);
[true_meas, noisy_meas, opf_res, ~, pmu_config] = generateMeasurements(1, config, load_multipliers);
if ~isfield(opf_res, 'success') || ~opf_res.success
    error('OPF 未收敛，无法演示。');
end

% 构造 mpc 用于 SE/攻击
mpc = mpc_base;
mpc.bus(:, PD) = mpc_base.bus(:, PD) * config.Simulation.LoadScaleFactor * load_multipliers(1);
mpc.bus(:, QD) = mpc_base.bus(:, QD) * config.Simulation.LoadScaleFactor * load_multipliers(1);

% ========== 基线：全覆盖、攻击者未知 pt/qt ==========
fprintf('\n--- 基线（全覆盖，UnknownFields={pt,qt}）---\n');
config.MeasurementSelection = struct(); % 留空 = 全覆盖

% 状态估计（攻击前）
[est_state0, ~, ~, ~, residual0, bdd0] = runStateEstimation(mpc, noisy_meas, pmu_config, config);
if isempty(residual0)
    fprintf('SE(基线) 未收敛\n');
else
    fprintf('SE(基线) 收敛: 残差范数 = %.6f, BDD=%d\n', norm(residual0), bdd0);
end

% 攻击者输入（移除 pt/qt）
atk_known = true_meas;
if isfield(atk_known.scada,'pt'), atk_known.scada = rmfield(atk_known.scada,'pt'); end
if isfield(atk_known.scada,'qt'), atk_known.scada = rmfield(atk_known.scada,'qt'); end

atk_res0 = runAttackGeneration(mpc, atk_known, true_meas, pmu_config, config);
obj0 = NaN; if isfield(atk_res0,'objective'), obj0 = atk_res0.objective; end
fprintf('攻击生成(基线): success=%d, obj=%.6f, lines=%s\n', atk_res0.success, obj0, mat2str(atk_res0.attacked_lines_indices));

% ========== 稀疏：示例选择若干条目 ==========
fprintf('\n--- 稀疏覆盖（示例子集）---\n');
sel = struct();
% 稀疏但可观：保留所有母线的 V 与 Pi，丢弃 Qi/Pf/Qf/Pt/Qt；保留全部 PMU 电压相量
num_buses = size(mpc.bus,1);
sel.SCADA.v_idx  = (1:num_buses)';
sel.SCADA.pi_idx = (1:num_buses)';
sel.SCADA.qi_idx = [];      % 显式禁用该字段
sel.SCADA.pf_idx = [];
sel.SCADA.qf_idx = [];
sel.SCADA.pt_idx = [];
sel.SCADA.qt_idx = [];
% PMU：保留全部安装的母线相量(去除参考母线的虚部观测冗余)；禁用支路电流相量
slack = config.Grid.SlackBusId;
pmu_buses = pmu_config.locations(:);
pmu_buses = pmu_buses(pmu_buses ~= slack);
sel.PMU.bus_idx = pmu_buses;
sel.PMU.from_branch_idx = [];
sel.PMU.to_branch_idx   = [];

config.MeasurementSelection = sel;

% 打印向量化维度
[z_full, ~, ~] = vectorizeAndMapMeasurements(noisy_meas, pmu_config, config.Noise, struct('num_buses', size(mpc.bus,1), 'num_branches', size(mpc.branch,1), 'selection', struct()));
[z_sparse, ~, ~] = vectorizeAndMapMeasurements(noisy_meas, pmu_config, config.Noise, struct('num_buses', size(mpc.bus,1), 'num_branches', size(mpc.branch,1), 'selection', sel));
num_states = 2*size(mpc.bus,1)-1;
fprintf('测量维度: 全覆盖=%d, 稀疏=%d\n', length(z_full), length(z_sparse));
fprintf('状态变量数: %d (稀疏需 >= 此值以保障可观)\n', num_states);

% 为增强可观性，增加一组首端有功潮流（全部支路）
sel.SCADA.pf_idx = (1:size(mpc.branch,1))';
config.MeasurementSelection = sel;

% 状态估计（稀疏）
[est_state1, ~, ~, ~, residual1, bdd1] = runStateEstimation(mpc, noisy_meas, pmu_config, config);
if isempty(residual1)
    fprintf('SE(稀疏) 未收敛\n');
else
    fprintf('SE(稀疏) 收敛: 残差范数 = %.6f, BDD=%d\n', norm(residual1), bdd1);
end

% 攻击（稀疏、攻击者仍未知 pt/qt）
atk_res1 = runAttackGeneration(mpc, atk_known, true_meas, pmu_config, config);
obj1 = NaN; if isfield(atk_res1,'objective'), obj1 = atk_res1.objective; end
fprintf('攻击生成(稀疏): success=%d, obj=%.6f, lines=%s\n', atk_res1.success, obj1, mat2str(atk_res1.attacked_lines_indices));

fprintf('\n=== DEMO 结束 ===\n');
end
