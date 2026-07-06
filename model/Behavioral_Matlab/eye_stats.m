function SummaryTable = eye_stats(tx_eye_matrix, eq_eye_matrix, t_vector, upsample_factor, max_violator_pct)
    % BENCHMARK_EYE_PERFORMANCE Dynamically locates the optimal eye center phase,
    % calculates vertical/horizontal metrics, and outputs a clean table.
    %
    % Inputs:
    %   tx_eye_matrix    - 2D matrix of the ideal transmitter eye traces
    %   eq_eye_matrix    - 2D matrix of the post-equalization receiver eye traces
    %   t_vector         - Time vector matching the trace width (in UI)
    %   upsample_factor  - Samples per symbol used in interpolation
    %   max_violator_pct - Percentage of allowed outlier traces (e.g., 2 for 2%)
    
    samples_per_trace = length(t_vector);

    % 1. DYNAMICALLY FIND EYE CENTER (Vertical Max Opening of Ideal TX)
    tx_heights = zeros(1, samples_per_trace);
    for t_idx = 1:samples_per_trace
        v_slice = tx_eye_matrix(t_idx, :);
        pos_inner = min(v_slice(v_slice > 0));
        neg_inner = max(v_slice(v_slice < 0));
        if ~isempty(pos_inner) && ~isempty(neg_inner)
            tx_heights(t_idx) = pos_inner - neg_inner;
        end
    end
    [tx_height_v, best_center_idx] = max(tx_heights);

    % 2. COMPUTE VERTICAL HEIGHT AT THE TRACKED CENTER PHASE
    eq_center_voltages = eq_eye_matrix(best_center_idx, :);
    pos_inner_rx = min(eq_center_voltages(eq_center_voltages > 0));
    neg_inner_rx = max(eq_center_voltages(eq_center_voltages < 0));

    if isempty(pos_inner_rx) || isempty(neg_inner_rx) || (pos_inner_rx < neg_inner_rx)
        eq_height_v = 0.0; % Eye is vertically closed
    else
        eq_height_v = pos_inner_rx - neg_inner_rx;
    end

    % 3. INLINE HORIZONTAL WIDTH ENGINE (Processes TX, then RX)
    widths_ui = [0, 0];
    matrices = {tx_eye_matrix, eq_eye_matrix};

    for m = 1:2
        current_matrix = matrices{m};
        v_center = current_matrix(best_center_idx, :);

        % Group traces into the stable +1V and -1V inner level families
        idx_p1 = find(v_center > 0 & v_center < 2);
        idx_m1 = find(v_center > -2 & v_center < 0);

        if isempty(idx_p1) || isempty(idx_m1)
            widths_ui(m) = 0.0;
            continue;
        end

        % Sweep horizontally to find where the inner family "lids" stay open
        open_mask = zeros(1, samples_per_trace);
        for t_idx = 1:samples_per_trace
            v_p1 = sort(current_matrix(t_idx, idx_p1));
            v_m1 = sort(current_matrix(t_idx, idx_m1));

            % Apply statistical outlier filter percentage
            p1_out = max(1, round((max_violator_pct/100) * length(v_p1)));
            m1_out = min(length(v_m1), round((1 - max_violator_pct/100) * length(v_m1)));

            % Eye is open horizontally if separation is at least 15% of ideal height
            if (v_p1(p1_out) - v_m1(m1_out)) >= 0.15 * tx_height_v
                open_mask(t_idx) = 1;
            end
        end

        % Extract the continuous window that contains the center sample phase
        edges = diff([0, open_mask, 0]);
        starts = find(edges == 1); ends = find(edges == -1) - 1;
        for w = 1:length(starts)
            if best_center_idx >= starts(w) && best_center_idx <= ends(w)
                widths_ui(m) = (ends(w) - starts(w) + 1) / upsample_factor;
                break;
            end
        end
    end
    
    % 5. DISPLAY GENERATED RESULTS
    fprintf('--- EQUALIZER PERFORMANCE BENCHMARK ---\n');
    fprintf('   (Dynamic Center Index Tracked to Sample: %d)   \n', best_center_idx);
    %fprintf('   (Dynamic Margin Boundary Set to: +/-%.2f V)    \n', voltage_margin);

    Metric = {'Horizontal Eye Width (UI)'; 'Vertical Eye Height (Volts)'};
    TX_Ideal = [widths_ui(1); tx_height_v];
    RX_Equalized = [widths_ui(2); eq_height_v];

    SummaryTable = table(TX_Ideal, RX_Equalized, 'RowNames', Metric);
    disp(SummaryTable);
end