function selectedModel = sliceModel(fullModel, sel)
% sliceModel 基于逻辑索引 sel 对全量模型进行切片，返回子模型

    if islogical(sel)
        idx = find(sel);
    else
        idx = sel(:);
        sel = false(fullModel.dim,1); sel(idx) = true;
    end

    selectedModel = struct();
    selectedModel.dim = numel(idx);
    selectedModel.num_state = fullModel.num_state;
    selectedModel.internals = fullModel.internals;
    selectedModel.groups = []; % 子模型直接在 evaluate 中切片
    selectedModel.registry = fullModel.registry(sel);
    selectedModel.R = fullModel.R(sel, sel);
    selectedModel.W = fullModel.W(sel, sel);

    selectedModel.evaluate = @(x) eval_slice(x, fullModel, idx);
end

function [h_sel, H_sel] = eval_slice(x, fullModel, idx)
    [h_full, H_full] = fullModel.evaluate(x);
    h_sel = h_full(idx);
    H_sel = H_full(idx, :);
end

