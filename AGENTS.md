# AGENTS.md

本文件面向通用 AI 代码助手（如 GPT-4/Code Interpreter、Copilot、Codeium 等），说明在本仓库中应如何安全、有效地进行协作与修改。

## 项目概述
本项目实现并评估电力系统中的隐蔽式虚假数据注入攻击（FDIA）。经重构后，测量处理改为“全量优先”：先基于全量测量定义构建统一模型，再通过选择器切片得到任意子集，WLS/BDD 与攻击优化均直接消费该子模型与向量化观测。
核心依赖：MATLAB R2022b、MATPOWER 8.0、YALMIP、IPOPT。

## 依赖与运行
- 必需：MATLAB R2022b+、MATPOWER 8.0、YALMIP、IPOPT（通过 YALMIP 调用）。
- 快速验证命令（MATLAB 命令行）：
  - `attack57`
  - `define_constants`（MATPOWER 路径可用性）
  - `yalmiptest`（YALMIP/求解器可用性）
- 主要输出：
  - `attack_simulation_results.mat`（按配置保存）
  - `Ptrmax.mat`（由 `case57.m` 生成，p.u.）
  - `Ptrmax57.mat`（预置容量矩阵）

## 代码结构与职责（重构后）
- `attack57.m`：主流程调度；按场景运行 OPF → 生成全量观测 → 切片 → 基线 SE/BDD → 攻击生成与注入 → 攻击后 SE/BDD。
- `load_config.m`：集中配置；优先参数化所有可调项（含测量选择 DSL、攻击者知识、求解器等）。
- `calculate_rate.m`：读取 `2-7_week.txt` 生成 96 点负荷倍率。
- `case57.m`：IEEE-57 案例；
- `buildFullMeasurementModel.m`：编译全量测量模型，提供 `fullModel.evaluate(x)->[h,H]`、`R/W`、`registry`（稳定顺序/元数据）。
- `makeMeasurementRegistry.m`：枚举全量测量（SCADA/PMU）并定义唯一顺序、单位与等效方差。
- `makeSelection.m`：选择器 DSL（include/exclude by domain/type/bus/branch/side/id）→ 逻辑索引 `sel` 与稀疏矩阵 `S`。
- `sliceModel.m`：从全量模型按 `sel` 切片得到 `selectedModel`（evaluate/H/R/W 均切片后对齐）。
- `generateMeasurementsFromState.m`：由真实状态 `x_true` 生成全量无噪/含噪观测向量。
- `runStateEstimation.m`：新接口 `[state, stats] = runStateEstimation(selectedModel, y, config)`；WLS + LNR-BDD。
- `runAttackGeneration.m`：新接口 `attack = runAttackGeneration(selectedModel, y, mpc, opf_results, config)`；构造“已知行”掩码、调用优化并合成 `y_att`。
- `buildAttackModel.m`：重构后的 YALMIP 模型（基于 `registry/known_mask` 的隐蔽性约束 + 潮流一致性 + 运行/机组/自定义/过载约束）。
- `solveAttackOptimization.m`：多起点求解与选优（保留）。
- `constraints_case57.m`：57 节点定制约束（保留）。



## 数值与稳定性要点
- WLS 收敛：可通过增加阻尼/正则、放宽阈值、改初值提升稳定性；避免极端权重比例失衡。
- 噪声建模：PMU 极坐标噪声在注册阶段已传播为直角等效方差；`R/W` 在全量层构建一次并被切片复用。
- 优化稳健：IPOPT 失败时可降低 λ（`Attack.VoltageDeviationPenalty`）、增大 `Attack.NumRestarts`、或适度放宽过载目标。
- BDD 一致：归一化残差定义与阈值含义不变；输入直接来自 `(y, H_sel, R_sel/W_sel)`。

## 典型任务清单（建议流程）
- 新增/替换负荷场景：更新 `2-7_week.txt` 或 `calculate_rate.m`；保持 96 点规范。
- 调整 PMU 布局：在 `attack57.m` 的 `pmu_config` 更新母线与支路索引，并验证维度。
- 调整噪声/权重：在 `load_config.m` 的 `Noise` 段更新；保持 PMU 权重与误差传播一致。
- 配置攻击者知识：更新 `config.Attack.UnknownFields`（如 `{'pt','qt'}`）。
- 调参与性能：调 `Attack.*`、`StateEstimation.*`；必要时增加日志打印或保存 `.mat` 结果。

## 运行与验证
- 基线对比：先运行无攻击路径（attack57 的基线分支），记录 OPF/SE/BDD 正常性，再启用攻击生成进行对比。
- 关键检查：
  - OPF 与测量维度是否匹配（`selectedModel.dim == numel(y)`）；
  - WLS 是否收敛、LNR 是否在阈值内；
  - 攻击注入后目标线路是否满足过载判定；
  - `registry` 顺序稳定且 `makeSelection/sliceModel` 切片后 `evaluate/R/W` 对齐一致。

## 故障排查速查
- MATPOWER 未就绪：`define_constants` 报错 → 确认已 addpath。
- YALMIP/求解器：`yalmiptest` 失败 → 安装并配置 IPOPT，检查路径。
- 维度不一致：检查 `registry` 与 `sel/S` 是否与 `y` 对齐；构造 `A = spdiags(sqrt(w),0,m,m)*H` 时 `m == size(H,1)`。
- WLS 不收敛：调权重、加阻尼/正则、增迭代、改初值（PMU v_real/v_imag 会自动播种初值）。
- 过载未触发：核对容量矩阵、攻击线路索引、目标过载率与约束是否过紧。

## 变更影响评估清单
- 是否改变测量定义/顺序/单位（`registry` 稳定性）？
- 是否影响 `makeSelection/sliceModel` 的切片一致性与 R/W 对齐？
- 是否改变 SE/BDD 判定逻辑或阈值含义？
- 物理约束是否仍闭合（KCL/KVL、支路功率一致性）？
- 性能影响是否可接受（全量模型复用；避免重复构建）。


## 谨慎/禁止修改
- 不随意更改 `case57.m` 的基准量、母线/支路排序；若必须变更，需同步审计所有索引。
- 不破坏 `makeMeasurementRegistry.m` 的顺序稳定性及字段含义；新增测量类型需定义清晰的 id/单位/方差。
- 不改变 `buildAttackModel.m` 中隐蔽性与潮流一致性约束语义；新增项须参数化。
-

## 性能建议
- 预分配与向量化优先，避免多层 for 循环。
- 复用全量模型与稀疏选择矩阵（模式不变时不重复编译）。
- 在 WLS 中使用稀疏对角缩放（`spdiags`）与正规方程的正则回退，提升稳健性。
- 仅在需要时打印详细日志，避免 I/O 成为瓶颈。

## 术语约定
- SCADA：V、Pf/Qf、Pt/Qt、Pi/Qi；PMU（向量层）：v_real/v_imag、if_real/if_imag、it_real/it_imag。
- 直角坐标：e、f；极坐标：幅值、相角（注册时已传播为直角等效方差）。
- LNR：最大归一化残差；BDD：坏数据检测。

