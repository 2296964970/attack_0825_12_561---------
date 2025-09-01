# AGENTS.md

本文件面向通用 AI 代码助手（如 GPT-4/Code Interpreter、Copilot、Codeium 等），说明在本仓库中应如何安全、有效地进行协作与修改。

## 项目概述
本项目实现并评估电力系统中的隐蔽式虚假数据注入攻击（FDIA），闭环包含：带 PMU 的混合测量 → WLS 状态估计 → 坏数据检测（BDD）→ 攻击生成与注入 → 过载验证。核心依赖：MATLAB R2022b、MATPOWER 8.0、YALMIP、IPOPT。

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

## 代码结构与职责
- `attack57.m`：主流程调度、循环场景、统计与绘图。
- `load_config.m`：集中配置；所有可调项应优先参数化到此处。
- `calculate_rate.m`：读取 `2-7_week.txt` 生成 96 点负荷倍率。
- `case57.m`：IEEE-57 案例；加载时生成 `Ptrmax.mat`。
- `generateMeasurements.m`：从 OPF 到测量生成（SCADA/PMU + 噪声）。
- `runStateEstimation.m`：WLS + LNR-BDD（PMU 线性、SCADA 非线性，直角坐标）。
- `calculate_h_H.m`：构建 h(x)/H，受当前“测量开关”控制。
- `vectorizeAndMapMeasurements.m`：测量向量化、映射表与权重（含 PMU 极→直误差传播）。
- `runAttackGeneration.m`：攻击模块调度与结果整合。
- `buildAttackModel.m`：YALMIP 模型（变量/目标/约束）。
- `solveAttackOptimization.m`：多起点求解与选优。
- `buildAttackedMeasurements.m`：注入 a、重算“未知类”、再加噪。
- `constraints_case57.m`：57 节点定制约束。

## 修改原则（务必遵守）
- 最小变更：聚焦问题本身，避免牵连跨模块大改；能参数化就不硬编码。
- 不破坏接口：不要随意更改函数签名、返回字段名与测量字段名。
- 维度严谨：任何改动须保证 z/H/权重/映射表维度一致，索引不偏移。
- 物理一致：潮流方程、PMU 线性模型需与 MATPOWER 的 Ybus/Yf/Yt 保持一致。
- 单位清晰：容量矩阵为 p.u.，谨防与实际功率单位混用。
- 可回滚：新增配置项统一加到 `load_config.m` 并给默认值与注释。
- 避免新依赖：不引入新的第三方工具箱/外部脚本。

## 数值与稳定性要点
- WLS 收敛：可通过增加阻尼/正则、放宽阈值、改初值提升稳定性；避免极端权重比例失衡。
- 噪声建模：PMU 极坐标噪声需正确传播到直角坐标权重矩阵。
- 优化稳健：IPOPT 失败时可降低 λ（`Attack.VoltageDeviationPenalty`）、增大 `Attack.NumRestarts`、或适度放宽过载目标。
- BDD 一致：请勿改变 LNR 统计量及置信度阈值的含义；改动请参数化。

## 典型任务清单（建议流程）
- 新增/替换负荷场景：更新 `2-7_week.txt` 或 `calculate_rate.m`；保持 96 点规范。
- 调整 PMU 布局：在 `attack57.m` 的 `pmu_config` 更新母线与支路索引，并验证维度。
- 调整噪声/权重：在 `load_config.m` 的 `Noise` 段更新；保持 PMU 权重与误差传播一致。
- 配置攻击者知识：更新 `config.Attack.UnknownFields`（如 `{'pt','qt'}`）。
- 调参与性能：调 `Attack.*`、`StateEstimation.*`；必要时增加日志打印或保存 `.mat` 结果。

## 运行与验证
- 基线对比：先运行无攻击路径，记录 OPF/SE/BDD 正常性，再启用攻击生成进行对比。
- 关键检查：
  - OPF 与测量维度是否匹配；
  - WLS 是否收敛、LNR 是否在阈值内；
  - 攻击注入后目标线路是否满足过载判定；
  - `measurement_map` 的映射是否可正确还原结构化测量。

## 故障排查速查
- MATPOWER 未就绪：`define_constants` 报错 → 确认已 addpath。
- YALMIP/求解器：`yalmiptest` 失败 → 安装并配置 IPOPT，检查路径。
- 维度不一致：多出现在向量化映射与 H 的构造阶段 → 回溯 `measurement_map` 索引。
- WLS 不收敛：调权重、加阻尼/正则、增迭代、改初值（如平坦起始或 OPF 结果）。
- 过载未触发：核对容量矩阵、攻击线路索引、目标过载率与约束是否过紧。

## 变更影响评估清单
- 是否改变测量定义/顺序/单位？
- 是否影响 `measurement_map` 的索引与还原？
- 是否改变 SE/BDD 判定逻辑或阈值含义？
- 物理约束是否仍闭合（KCL/KVL、支路功率一致性）？
- 性能影响是否可接受（大规模场景循环）？

## 提交与说明（建议模板）
- 变更概述：一句话说明目标与范围。
- 详细改动：列出受影响文件与关键接口变化。
- 验证结果：说明运行命令、关键日志、是否通过基线对比。
- 回滚方案：如何通过配置恢复旧行为。

## 谨慎/禁止修改
- 不随意更改 `case57.m` 的基准量、母线/支路排序；若必须变更，需同步审计所有索引。
- 不改变 `vectorizeAndMapMeasurements.m` 输出顺序与字段含义。
- 不改变 `buildAttackModel.m` 中隐蔽性与潮流一致性约束语义；新增项须参数化。
- 不提交大型数据文件或与研究无关的脚本/依赖。

## 性能建议
- 预分配与向量化优先，避免多层 for 循环。
- 复用稀疏矩阵/映射（若模式不变），减少重复构建。
- 仅在需要时打印详细日志，避免 I/O 成为瓶颈。

## 术语约定
- SCADA：V、Pf/Qf、Pt/Qt、Pi/Qi；PMU：Vm/Va、Imf/Iaf、Imt/Iat。
- 直角坐标：e、f；极坐标：幅值、相角。
- LNR：最大归一化残差；BDD：坏数据检测。

