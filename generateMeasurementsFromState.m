function [y_full, y0_full] = generateMeasurementsFromState(fullModel, x_true, rngState)
% generateMeasurementsFromState 基于真实状态生成全量测量（含噪/无噪）
%
% 输入
% - fullModel: buildFullMeasurementModel 返回的全量模型
% - x_true  : 真实状态向量 [e; f_wo_slack]
% - rngState: 可选，随机数种子
%
% 输出
% - y_full : 含噪测量向量
% - y0_full: 无噪 h(x_true)

    if nargin >= 3 && ~isempty(rngState)
        rng(rngState);
    end

    [y0_full, ~] = fullModel.evaluate(x_true);

    % 对角方差噪声
    v = fullModel.R(sub2ind(size(fullModel.R), (1:fullModel.dim)', (1:fullModel.dim)'));
    stdv = sqrt(max(v, 0));

    noise = randn(fullModel.dim, 1) .* stdv;
    y_full = y0_full + noise;
end
