function expected_total = validateMeasurementDimensions(measured_measurements, pmu_config, num_buses, num_branches)
% validateMeasurementDimensions 校验测量字段长度是否与系统规模一致。
%
% 输入
% - measured_measurements: struct，包含 .scada / .pmu
% - pmu_config           : struct，locations / pmu_from_branch_indices / pmu_to_branch_indices
% - num_buses            : 母线总数
% - num_branches         : 支路总数
%
% 输出
% - expected_total       : 按字段可用性计算出的总测量数（便于和 z 长度对比）

    expected_total = 0;

    % --- SCADA ---
    if isfield(measured_measurements, 'scada')
        sc = measured_measurements.scada;
        if isfield(sc, 'v')  && ~isempty(sc.v),  assert(length(sc.v)  == num_buses,    'SCADA.v 长度应为 %d (got %d)',  num_buses, length(sc.v));   expected_total = expected_total + num_buses; end
        if isfield(sc, 'pi') && ~isempty(sc.pi), assert(length(sc.pi) == num_buses,    'SCADA.pi 长度应为 %d (got %d)', num_buses, length(sc.pi)); expected_total = expected_total + num_buses; end
        if isfield(sc, 'qi') && ~isempty(sc.qi), assert(length(sc.qi) == num_buses,    'SCADA.qi 长度应为 %d (got %d)', num_buses, length(sc.qi)); expected_total = expected_total + num_buses; end
        if isfield(sc, 'pf') && ~isempty(sc.pf), assert(length(sc.pf) == num_branches, 'SCADA.pf 长度应为 %d (got %d)', num_branches, length(sc.pf)); expected_total = expected_total + num_branches; end
        if isfield(sc, 'qf') && ~isempty(sc.qf), assert(length(sc.qf) == num_branches, 'SCADA.qf 长度应为 %d (got %d)', num_branches, length(sc.qf)); expected_total = expected_total + num_branches; end
        if isfield(sc, 'pt') && ~isempty(sc.pt), assert(length(sc.pt) == num_branches, 'SCADA.pt 长度应为 %d (got %d)', num_branches, length(sc.pt)); expected_total = expected_total + num_branches; end
        if isfield(sc, 'qt') && ~isempty(sc.qt), assert(length(sc.qt) == num_branches, 'SCADA.qt 长度应为 %d (got %d)', num_branches, length(sc.qt)); expected_total = expected_total + num_branches; end
    end

    % --- PMU ---
    if isfield(measured_measurements, 'pmu') && ~isempty(fieldnames(measured_measurements.pmu))
        pmu = measured_measurements.pmu;
        if isfield(pmu, 'vm') && ~isempty(pmu.vm)
            assert(length(pmu.vm) == length(pmu_config.locations), 'PMU.vm 长度应为 %d (got %d)', length(pmu_config.locations), length(pmu.vm));
            expected_total = expected_total + length(pmu_config.locations);
        end
        if isfield(pmu, 'va') && ~isempty(pmu.va)
            assert(length(pmu.va) == length(pmu_config.locations), 'PMU.va 长度应为 %d (got %d)', length(pmu_config.locations), length(pmu.va));
            expected_total = expected_total + length(pmu_config.locations);
        end
        if isfield(pmu, 'vm') && isfield(pmu, 'va') && ~isempty(pmu.vm) && ~isempty(pmu.va)
            assert(length(pmu.vm) == length(pmu.va), 'PMU.vm / PMU.va 长度不一致: %d vs %d', length(pmu.vm), length(pmu.va));
        end

        if isfield(pmu, 'imf') && ~isempty(pmu.imf)
            assert(length(pmu.imf) == length(pmu_config.pmu_from_branch_indices), 'PMU.imf 长度应为 %d (got %d)', length(pmu_config.pmu_from_branch_indices), length(pmu.imf));
            expected_total = expected_total + length(pmu_config.pmu_from_branch_indices);
        end
        if isfield(pmu, 'iaf') && ~isempty(pmu.iaf)
            assert(length(pmu.iaf) == length(pmu_config.pmu_from_branch_indices), 'PMU.iaf 长度应为 %d (got %d)', length(pmu_config.pmu_from_branch_indices), length(pmu.iaf));
            expected_total = expected_total + length(pmu_config.pmu_from_branch_indices);
        end
        if isfield(pmu, 'imf') && isfield(pmu, 'iaf') && ~isempty(pmu.imf) && ~isempty(pmu.iaf)
            assert(length(pmu.imf) == length(pmu.iaf), 'PMU.imf / PMU.iaf 长度不一致: %d vs %d', length(pmu.imf), length(pmu.iaf));
        end

        if isfield(pmu, 'imt') && ~isempty(pmu.imt)
            assert(length(pmu.imt) == length(pmu_config.pmu_to_branch_indices), 'PMU.imt 长度应为 %d (got %d)', length(pmu_config.pmu_to_branch_indices), length(pmu.imt));
            expected_total = expected_total + length(pmu_config.pmu_to_branch_indices);
        end
        if isfield(pmu, 'iat') && ~isempty(pmu.iat)
            assert(length(pmu.iat) == length(pmu_config.pmu_to_branch_indices), 'PMU.iat 长度应为 %d (got %d)', length(pmu_config.pmu_to_branch_indices), length(pmu.iat));
            expected_total = expected_total + length(pmu_config.pmu_to_branch_indices);
        end
        if isfield(pmu, 'imt') && isfield(pmu, 'iat') && ~isempty(pmu.imt) && ~isempty(pmu.iat)
            assert(length(pmu.imt) == length(pmu.iat), 'PMU.imt / PMU.iat 长度不一致: %d vs %d', length(pmu.imt), length(pmu.iat));
        end
    end
end

