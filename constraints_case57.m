function custom_constraints = constraints_case57(Pi_expr, mpc)
% constraints_case57: 返回IEEE 57节点系统的特定有功功率约束
%
% 输入:
%   Pi_expr (sdpvar): YALMIP中代表节点有功注入功率的表达式向量
%   mpc (struct): MATPOWER case 结构体 (可选，用于未来扩展)
%
% 输出:
%   custom_constraints (constraint): 包含所有自定义约束的YALMIP约束对象

custom_constraints = [];

% 这些是针对IEEE 57节点系统的特定约束
% 它们来自于旧代码，现在被模块化地管理在这里

% 负荷节点 (纯消耗功率)
load_buses_part1 = 13:20;
load_buses_part2 = 27:33;
load_buses_part3 = 41:44;
load_buses_part4 = 49:57;
all_load_buses = [load_buses_part1, load_buses_part2, load_buses_part3, load_buses_part4];
for i = all_load_buses
    custom_constraints = [custom_constraints, Pi_expr(i) <= 0];
end

% 特殊发电机/联络点约束
% 添加从旧代码迁移过来的，针对非发电机节点的特定约束
custom_constraints = [custom_constraints, Pi_expr(4) <= 0.1];
custom_constraints = [custom_constraints, Pi_expr(5) <= 0];
custom_constraints = [custom_constraints, Pi_expr(7) <= 1]; % 注意：这个约束允许正注入，请根据物理意义确认
custom_constraints = [custom_constraints, Pi_expr(10) <= 0];
custom_constraints = [custom_constraints, -0.01 <= Pi_expr(11) <= 0.01];

% 其他原有的特殊约束
custom_constraints = [custom_constraints, -0.01 <= Pi_expr(21) <= 0.01];
custom_constraints = [custom_constraints, Pi_expr(22) <= 0.1];
custom_constraints = [custom_constraints, Pi_expr(23) <= 0];
custom_constraints = [custom_constraints, -0.01 <= Pi_expr(24) <= 0.01];
custom_constraints = [custom_constraints, Pi_expr(25) <= 0];
custom_constraints = [custom_constraints, -0.01 <= Pi_expr(26) <= 0.01];
custom_constraints = [custom_constraints, Pi_expr(34) <= 0.1];
custom_constraints = [custom_constraints, Pi_expr(35) <= 0];
custom_constraints = [custom_constraints, Pi_expr(36) <= 0];
custom_constraints = [custom_constraints, Pi_expr(37) <= 0.1];
custom_constraints = [custom_constraints, Pi_expr(38) <= 0];
custom_constraints = [custom_constraints, Pi_expr(39) <= 0];
custom_constraints = [custom_constraints, Pi_expr(40) <= 0.1];
custom_constraints = [custom_constraints, Pi_expr(45) <= 0.1];
custom_constraints = [custom_constraints, Pi_expr(46) <= 0.1];
custom_constraints = [custom_constraints, Pi_expr(47) <= 0];
custom_constraints = [custom_constraints, -0.01 <= Pi_expr(48) <= 0.01];

end
