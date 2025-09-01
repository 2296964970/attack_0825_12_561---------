function custom_constraints = constraints_case57(Pi_expr, mpc)
% constraints_case57 IEEE-57 节点系统的特定有功注入约束（示例）。
%
% 输入
% - Pi_expr: sdpvar，节点有功注入向量表达式
% - mpc    : MATPOWER case（保留用于扩展）
%
% 输出
% - custom_constraints: YALMIP 约束对象集合

custom_constraints = [];

% 典型负荷节点（纯消耗功率）约束：Pi <= 0
load_buses_part1 = 13:20;
load_buses_part2 = 27:33;
load_buses_part3 = 41:44;
load_buses_part4 = 49:57;
all_load_buses = [load_buses_part1, load_buses_part2, load_buses_part3, load_buses_part4];
for i = all_load_buses
    custom_constraints = [custom_constraints, Pi_expr(i) <= 0]; %#ok<AGROW>
end

% 其他系统特定约束（从原逻辑整理迁移）
custom_constraints = [custom_constraints, Pi_expr(4)  <= 0.1];
custom_constraints = [custom_constraints, Pi_expr(5)  <= 0];
custom_constraints = [custom_constraints, Pi_expr(7)  <= 1];    % 注意：允许正注入
custom_constraints = [custom_constraints, Pi_expr(10) <= 0];
custom_constraints = [custom_constraints, -0.01 <= Pi_expr(11) <= 0.01];

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

