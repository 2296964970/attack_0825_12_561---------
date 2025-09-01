# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述
本项目实现并评估电力系统中的隐蔽式虚假数据注入攻击（FDIA），面向"带PMU的混合测量+WLS状态估计+坏数据检测（BDD）"的完整仿真闭环。核心依赖为 MATLAB R2022b、MATPOWER 8.0、YALMIP 和 IPOPT。

## 依赖环境与运行

### 必需软件
- MATLAB R2022b 或更高版本
- MATPOWER 8.0（电力系统潮流计算工具包）
- YALMIP（优化建模工具包）
- IPOPT 求解器（通过 YALMIP 接口调用）

### 运行命令
```matlab
% 主仿真脚本
attack57

% 验证MATPOWER工具包是否正常
define_constants

% 检查YALMIP环境
yalmiptest
```

### 输出文件
- `attack_simulation_results.mat`：完整仿真结果与统计数据（如果已配置）
- `Ptrmax.mat`：线路容量矩阵（由case57.m生成）
- `Ptrmax57.mat`：预置的57节点线路容量矩阵

## 测量类型
- SCADA：节点电压幅值 V，线路潮流（首端/末端有功 Pf/Pt、无功 Qf/Qt），节点注入有功/无功 Pi/Qi。
- PMU：节点电压相量（幅值 Vm、相角 Va），支路电流相量（首端 Imf/Iaf、末端 Imt/Iat）。

## 总体架构与大模块
本项目由三大模块构成：
1) 主仿真流程（场景生成与评估）
2) 状态估计模块（WLS + BDD）
3) 攻击生成模块（YALMIP 优化 + 多起点求解）

### 大模块与关键文件

#### 1) 主仿真流程
- **attack57.m**：顶层脚本，按场景循环执行"负荷设定 → OPF → 测量生成 → 攻击前SE/BDD → 生成攻击 → 注入后SE/BDD → 过载验证 → 绘图与统计"。
- **load_config.m**：集中化配置（系统路径、仿真规模、PMU位置、噪声、攻击参数、SE参数等）。
- **calculate_rate.m**：从 `2-7_week.txt` 读取并计算96点负荷变化率，用于逐场景负荷缩放。
- **case57.m**：IEEE 57 节点算例（MATPOWER 格式），并在加载时计算并保存 `Ptrmax.mat`（线路容量矩阵，p.u.）。
- **2-7_week.txt**：负荷曲线原始数据文件（96 点）。
- **Ptrmax57.mat**：预置的57节点线路容量矩阵（字段 `Ptrmax57`），主流程中作为容量参考加载。

#### 2) 状态估计模块（WLS + BDD）
- **runStateEstimation.m**：
  - 输入混合测量与噪声模型，采用直角坐标（e, f）构造线性 PMU 模型与非线性 SCADA 模型，进行 WLS 迭代。
  - 内部调用 calculate_h_H.m 构建 h(x) 与雅可比 H；调用 vectorizeAndMapMeasurements.m 将结构化测量向量化并生成权重。
  - 收敛后计算残差 r 及"最大归一化残差（LNR）"坏数据检测，输出最终状态、估计测量与 BDD 标志。
- **calculate_h_H.m**：给定状态 x 和网络模型，按当前"测量开关"计算 h(x) 与 H（SCADA 非线性，PMU 线性）。
- **vectorizeAndMapMeasurements.m**：将 {SCADA, PMU} 结构化测量转换为统一向量 z，并生成映射表与权重（含 PMU 极坐标→直角坐标的误差传播加权）。

#### 3) 攻击生成模块（YALMIP 优化）
- **runAttackGeneration.m**：模块调度器。对攻击者"已知测量"向量化，构建模型并多起点求解，输出：
  - success/objective/attack_vector（a）/compromised_state（Vc, uc）/solver_diagnostics；
  - attacked_measurements：将 a 注入已知测量，并按虚假状态重算未知测量。
- **buildAttackModel.m**：YALMIP 建模（变量、目标、约束）。
  - 变量：虚假电压（V, θ）、节点注入（Pi, Qi）、支路潮流（Pf/Qf, Pt/Qt）、攻击向量 a。
  - 目标：min ||a||₁ + λ·||V − V_true||₂（λ 由配置 `Attack.VoltageDeviationPenalty` 决定）。
  - 约束：
    • 潮流物理一致性（Ybus/Yf/Yt）；
    • 隐蔽性 h(x_c) = z + a（按测量映射逐段约束）；
    • 运行约束（Vmin ≤ V ≤ Vmax、参考角/压、发电机 Pg 边界）；
    • 自定义系统约束（constraints_case57.m）；
    • 目标线路过载约束（按 `Attack.attacked_lines_indices` 和容量矩阵实现）。
- **solveAttackOptimization.m**：多起点（`Attack.NumRestarts`）+ 指定求解器（默认 IPOPT），择优保留最优解。
- **buildAttackedMeasurements.m**：根据最优解 a 与虚假状态 (Vc, uc)：
  - 对攻击者"已知"的测量直接加 a；
  - 对"未知"的测量按虚假状态重算；
  - 对伪造数据再叠加噪声，以提高真实感。
- **constraints_case57.m**：57节点场景的定制有功注入边界等附加约束。

## 测量生成与攻击者知识建模
- **generateMeasurements.m**：
  - 从基础电网模型开始，完整地处理从OPF计算到测量数据生成的整个流程。
  - 输入场景索引、配置和负荷倍率，输出真实测量、含噪测量、OPF结果、系统参数和PMU配置。
  - SCADA：V、Pf/Qf、Pt/Qt、Pi/Qi；PMU：Vm/Va、Im(Iaf)/It(Iat)。
- **攻击者知识（UnknownFields）**：
  - 在主脚本中以 `config.Attack.UnknownFields` 指定某些测量类型对攻击者"不可见"（整类缺失）。
  - 示例：{'pt','qt'} 表示攻击者不知道所有末端潮流，优化仅基于其余"可见测量"求解 a，但最终可同时篡改系统测量（通过重算未知类的伪造值）。

## 关键数据与结构
- **pmu_config**（在 attack57.m 内构造）：
  - fields：`locations`（PMU母线），`pmu_from_branch_indices`/`pmu_to_branch_indices`（对应支路索引）。
- **measurement_map**（由 vectorizeAndMapMeasurements 生成）：
  - 记录 z 向量每一段对应的测量类型/字段/数量及在原结构中的索引，供建模/还原使用。
- **line_capacity 矩阵**：
  - `Ptrmax57.mat`（字段 `Ptrmax57`）在主流程中加载；`case57.m` 也会计算并保存 `Ptrmax.mat`（p.u.）。

## 运行流程（简要）
1. 在 MATLAB 中打开仓库根目录，确保已安装并加入路径：MATPOWER、YALMIP、IPOPT。
2. 打开并运行 `attack57.m`。
3. 结果：
   - 命令窗打印各场景 OPF/SE/BDD 状态、攻击生成结果；
   - 控制台输出详细的仿真进度和结果统计；
   - 可选择保存 `attack_simulation_results.mat` 整体日志与统计（需配置）。

## 配置项要点（load_config.m）
- **System**：算例名、数据文件、输出目录与结果文件名。
- **Simulation**：是否详细输出、场景数、负荷缩放因子。
- **Grid**：Slack 母线、PMU 安装母线、电压上下限。
- **Noise**：SCADA/PMU 各类噪声标准差（用于权重与仿真）。
- **Attack**：可攻击线路集合、每次攻击线路数、目标过载率、重启次数、UnknownFields、λ、求解器。
- **StateEstimation**：最大迭代、收敛阈值、坏数据检测置信度、正则化因子。

## 文件作用速查
- **attack57.m**：主流程调度、统计与绘图。
- **load_config.m**：集中配置（参数一站式管理）。
- **case57.m**：57节点算例与容量矩阵生成（保存 `Ptrmax.mat`）。
- **calculate_rate.m**：读取并生成 96 点负荷倍率。
- **generateMeasurements.m**：从 OPF 结果生成真实/含噪测量。
- **runStateEstimation.m**：WLS 状态估计 + LNR 坏数据检测。
- **calculate_h_H.m**：按当前测量组合计算 h(x)/H。
- **vectorizeAndMapMeasurements.m**：测量向量化、映射与权重计算。
- **runAttackGeneration.m**：攻击生成主控与结果拼装。
- **buildAttackModel.m**：YALMIP 变量/目标/约束建模。
- **solveAttackOptimization.m**：多起点求解并选优。
- **buildAttackedMeasurements.m**：应用 a + 重算未知 + 加噪。
- **constraints_case57.m**：场景定制约束集合。
- **Ptrmax57.mat**：线路容量矩阵（字段 `Ptrmax57`）。
- **2-7_week.txt**：负荷数据源文件（96 点）。

## 给 Claude 的工作提示
- 保持最小变更：任何修改应聚焦单一问题，避免牵连联动文件；优先在 `load_config.m` 做可回滚的参数化调整。
- 维度一致性：修改 `vectorizeAndMapMeasurements.m`、`calculate_h_H.m`、`runStateEstimation.m` 时，务必同步检查测量向量 z、H 的维度和与 `measurement_map` 的索引映射。
- 物理一致：涉及潮流与 PMU 方程的改动需同时校验 Ybus/Yf/Yt 的一致性，避免违反基于 MATPOWER 的网络模型假设。
- 隐蔽性约束：在 `buildAttackModel.m` 中不要改变“隐蔽性”约束的语义（h(x_c) = z + a）；如需新增松弛或罚项，请以可配置方式挂到 `load_config.m`。
- 数值稳定：WLS 不收敛可考虑增加阻尼、正则或放宽收敛阈值；优化失败可降低 λ、调大重启次数、或放宽过载目标。
- 不破坏接口：不要改变关键函数对外接口与返回结构字段名，除非同步更新所有调用点并在注释中说明。

