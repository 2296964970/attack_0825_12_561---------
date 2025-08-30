function expected_total = validateMeasurementDimensions(measured_measurements, pmu_config, num_buses, num_branches)
%VALIDATEMEASUREMENTDIMENSIONS 严格校验测量各字段长度与系统规模一致
%
% 输入:
%   measured_measurements - struct, 包含 .scada / .pmu 字段
%   pmu_config            - struct, 含 locations / pmu_from_branch_indices / pmu_to_branch_indices
%   num_buses             - 总母线数
%   num_branches          - 总支路数
%
% 输出:
%   expected_total        - 依据可用字段计算的总测量数，用于与 z 向量长度或 num_measurements 对比

    expected_total = 0;

    % --- SCADA ---
    if isfield(measured_measurements, 'scada')
        sc = measured_measurements.scada;
        if isfield(sc, 'v') && ~isempty(sc.v)
            assert(length(sc.v) == num_buses, 'SCADA.v 长度与总母线数不一致: got %d, expect %d', length(sc.v), num_buses);
            expected_total = expected_total + num_buses;
        end
        if isfield(sc, 'pi') && ~isempty(sc.pi)
            assert(length(sc.pi) == num_buses, 'SCADA.pi 长度与总母线数不一致: got %d, expect %d', length(sc.pi), num_buses);
            expected_total = expected_total + num_buses;
        end
        if isfield(sc, 'qi') && ~isempty(sc.qi)
            assert(length(sc.qi) == num_buses, 'SCADA.qi 长度与总母线数不一致: got %d, expect %d', length(sc.qi), num_buses);
            expected_total = expected_total + num_buses;
        end
        if isfield(sc, 'pf') && ~isempty(sc.pf)
            assert(length(sc.pf) == num_branches, 'SCADA.pf 长度与支路数不一致: got %d, expect %d', length(sc.pf), num_branches);
            expected_total = expected_total + num_branches;
        end
        if isfield(sc, 'qf') && ~isempty(sc.qf)
            assert(length(sc.qf) == num_branches, 'SCADA.qf 长度与支路数不一致: got %d, expect %d', length(sc.qf), num_branches);
            expected_total = expected_total + num_branches;
        end
        if isfield(sc, 'pt') && ~isempty(sc.pt)
            assert(length(sc.pt) == num_branches, 'SCADA.pt 长度与支路数不一致: got %d, expect %d', length(sc.pt), num_branches);
            expected_total = expected_total + num_branches;
        end
        if isfield(sc, 'qt') && ~isempty(sc.qt)
            assert(length(sc.qt) == num_branches, 'SCADA.qt 长度与支路数不一致: got %d, expect %d', length(sc.qt), num_branches);
            expected_total = expected_total + num_branches;
        end
    end

    % --- PMU ---
    if isfield(measured_measurements, 'pmu') && ~isempty(fieldnames(measured_measurements.pmu))
        pmu = measured_measurements.pmu;
        % bus 量测
        if isfield(pmu, 'vm') && ~isempty(pmu.vm)
            assert(length(pmu.vm) == length(pmu_config.locations), 'PMU.vm 长度与 PMU 位置数不一致: got %d, expect %d', length(pmu.vm), length(pmu_config.locations));
            expected_total = expected_total + length(pmu_config.locations);
        end
        if isfield(pmu, 'va') && ~isempty(pmu.va)
            assert(length(pmu.va) == length(pmu_config.locations), 'PMU.va 长度与 PMU 位置数不一致: got %d, expect %d', length(pmu.va), length(pmu_config.locations));
            expected_total = expected_total + length(pmu_config.locations);
        end
        if isfield(pmu, 'vm') && isfield(pmu, 'va') && ~isempty(pmu.vm) && ~isempty(pmu.va)
            assert(length(pmu.vm) == length(pmu.va), 'PMU.vm 与 PMU.va 长度不一致: %d vs %d', length(pmu.vm), length(pmu.va));
        end
        % from 端支路电流
        if isfield(pmu, 'imf') && ~isempty(pmu.imf)
            assert(length(pmu.imf) == length(pmu_config.pmu_from_branch_indices), 'PMU.imf 长度与 pmu_from_branch_indices 不一致: got %d, expect %d', length(pmu.imf), length(pmu_config.pmu_from_branch_indices));
            expected_total = expected_total + length(pmu_config.pmu_from_branch_indices);
        end
        if isfield(pmu, 'iaf') && ~isempty(pmu.iaf)
            assert(length(pmu.iaf) == length(pmu_config.pmu_from_branch_indices), 'PMU.iaf 长度与 pmu_from_branch_indices 不一致: got %d, expect %d', length(pmu.iaf), length(pmu_config.pmu_from_branch_indices));
            expected_total = expected_total + length(pmu_config.pmu_from_branch_indices);
        end
        if isfield(pmu, 'imf') && isfield(pmu, 'iaf') && ~isempty(pmu.imf) && ~isempty(pmu.iaf)
            assert(length(pmu.imf) == length(pmu.iaf), 'PMU.imf 与 PMU.iaf 长度不一致: %d vs %d', length(pmu.imf), length(pmu.iaf));
        end
        % to 端支路电流
        if isfield(pmu, 'imt') && ~isempty(pmu.imt)
            assert(length(pmu.imt) == length(pmu_config.pmu_to_branch_indices), 'PMU.imt 长度与 pmu_to_branch_indices 不一致: got %d, expect %d', length(pmu.imt), length(pmu_config.pmu_to_branch_indices));
            expected_total = expected_total + length(pmu_config.pmu_to_branch_indices);
        end
        if isfield(pmu, 'iat') && ~isempty(pmu.iat)
            assert(length(pmu.iat) == length(pmu_config.pmu_to_branch_indices), 'PMU.iat 长度与 pmu_to_branch_indices 不一致: got %d, expect %d', length(pmu.iat), length(pmu_config.pmu_to_branch_indices));
            expected_total = expected_total + length(pmu_config.pmu_to_branch_indices);
        end
        if isfield(pmu, 'imt') && isfield(pmu, 'iat') && ~isempty(pmu.imt) && ~isempty(pmu.iat)
            assert(length(pmu.imt) == length(pmu.iat), 'PMU.imt 与 PMU.iat 长度不一致: %d vs %d', length(pmu.imt), length(pmu.iat));
        end
    end
end

