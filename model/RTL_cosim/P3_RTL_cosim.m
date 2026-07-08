%{
Kenneth Lin
PAM4 eye metrics post-processing: ideal TX vs noisy RX vs behavioral EQ vs RTL EQ

Measures all THREE PAM4 sub-eyes (bottom: -3/-1, middle: -1/+1, top: +1/+3)
the way an oscilloscope does:
  1. Classify each trace into a level family (-3,-1,+1,+3) by its voltage
     at the eye center
  2. Eye HEIGHT  = gap between adjacent families' inner percentile edges
     at the center (2% outliers ignored, like a scope's hit-ratio setting)
  3. Eye WIDTH   = the time span around the center where (almost) no trace
     passes through a voltage band placed at the decision threshold
     (-2/0/+2 V). This is the scope definition: it answers "how far can my
     sampling phase drift before decisions start hitting transitions."
Also reports per-level noise RMS, RLM (relative level mismatch), and a
direct behavioral-vs-RTL numerical match check.
%}

clear; clc; close all;

%% CONFIG
num_symbols = 5000;
upsample_factor = 8;
samples_per_trace = upsample_factor * 2; % 2 UI wide
settled_start = 301;                     % discard first 300 for settling
volts_to_lsb = 31;
lsb_to_volt = 1 / volts_to_lsb;
outlier_pct = 2;                         % % of traces ignored as outliers
open_frac = 0.15;                        % eye "open" if gap >= 15% of TX ref

%% REGENERATE CHANNEL DATA (identical to P3_Channel_Model_cosim.m)
rng(42);
raw_symbols = randi([0, 3], num_symbols, 1);
tx_waveform = 2 * raw_symbols - 3;

h_lossy = [0.8, 0.3, 0.1];
rx_waveform_lossy = filter(h_lossy, 1, tx_waveform);
noise = thermal_noise(length(rx_waveform_lossy), 50, 28e9, 300);
rx_waveform_noisy = rx_waveform_lossy + noise;

%% BEHAVIORAL FIXED-POINT MODEL (matches P3, mu matched to RTL 1/256 step)
num_ffe_taps = 3;
ffe_weights = [1.0; 0.0; 0.0];
dfe_weight = 0.0;
mu = 1/256;

adc_bits = 8;
adc_max = 2^(adc_bits-1) - 1;
adc_min = -2^(adc_bits-1);

equalized_waveform = zeros(num_symbols, 1);
rx_shift_reg = zeros(num_ffe_taps, 1);
prev_decision = 0;

for i = 1:num_symbols
    digital_rx = round(rx_waveform_noisy(i) * volts_to_lsb);
    if digital_rx > adc_max
        digital_rx = adc_max;
    elseif digital_rx < adc_min
        digital_rx = adc_min;
    end

    rx_shift_reg = [digital_rx; rx_shift_reg(1:end-1)];
    ffe_out = ffe_weights' * rx_shift_reg;
    dfe_out = dfe_weight * prev_decision;

    eq_signal = ffe_out - dfe_out;
    equalized_waveform(i) = eq_signal;

    if eq_signal >= round(2.0 * volts_to_lsb)
        decision = round(3.0 * volts_to_lsb);
    elseif eq_signal >= 0
        decision = round(1.0 * volts_to_lsb);
    elseif eq_signal >= round(-2.0 * volts_to_lsb)
        decision = round(-1.0 * volts_to_lsb);
    else
        decision = round(-3.0 * volts_to_lsb);
    end

    error_val = eq_signal - decision;
    ffe_weights = ffe_weights - mu * (sign(error_val) * sign(rx_shift_reg));
    dfe_weight = dfe_weight + mu * (sign(error_val) * sign(prev_decision));
    prev_decision = decision;
end
equalized_waveform = equalized_waveform * lsb_to_volt;

%% LOAD RTL COSIM OUTPUT
rtl_fid = fopen('eq_output_dump.hex', 'r');
rtl_hex = textscan(rtl_fid, '%s');
fclose(rtl_fid);
rtl_output_int = hex2dec(rtl_hex{1});
rtl_output_int(rtl_output_int > 32767) = rtl_output_int(rtl_output_int > 32767) - 65536;
rtl_waveform = rtl_output_int * lsb_to_volt;

%% BEHAVIORAL vs RTL NUMERICAL MATCH (before any interpolation)
n_cmp = min(length(equalized_waveform), length(rtl_waveform));
delta = equalized_waveform(settled_start:n_cmp) - rtl_waveform(settled_start:n_cmp);
delta_lsb = delta * volts_to_lsb;
fprintf('--- Behavioral vs RTL match (settled samples) ---\n');
fprintf('max |delta| : %.1f LSB (%.3f V)\n', max(abs(delta_lsb)), max(abs(delta)));
fprintf('rms  delta  : %.2f LSB\n', sqrt(mean(delta_lsb.^2)));
fprintf('(a few LSB is expected: float weights vs Q8.8 hardware)\n\n');

%% BUILD EYE MATRICES
settled_tx  = tx_waveform(settled_start:end);
settled_rx  = rx_waveform_noisy(settled_start:end);
settled_eq  = equalized_waveform(settled_start:end);
settled_rtl = rtl_waveform(settled_start:end);

tx_smooth  = interp(settled_tx,  upsample_factor);
rx_smooth  = interp(settled_rx,  upsample_factor);
eq_smooth  = interp(settled_eq,  upsample_factor);
rtl_smooth = interp(settled_rtl, upsample_factor);

interp_delay = 29; % strip interpolation filter group delay
tx_smooth  = tx_smooth(interp_delay:end);
rx_smooth  = rx_smooth(interp_delay:end);
eq_smooth  = eq_smooth(interp_delay:end);
rtl_smooth = rtl_smooth(interp_delay:end);

min_len = min([length(tx_smooth), length(rx_smooth), length(eq_smooth), length(rtl_smooth)]);
num_traces = floor(min_len / samples_per_trace) - 1;
t_vector = (0:samples_per_trace-1) / upsample_factor;

tx_eye  = reshape(tx_smooth(1:num_traces*samples_per_trace),  samples_per_trace, num_traces);
rx_eye  = reshape(rx_smooth(1:num_traces*samples_per_trace),  samples_per_trace, num_traces);
eq_eye  = reshape(eq_smooth(1:num_traces*samples_per_trace),  samples_per_trace, num_traces);
rtl_eye = reshape(rtl_smooth(1:num_traces*samples_per_trace), samples_per_trace, num_traces);

%% MEASURE ALL FOUR EYES
% TX first: its middle-eye height is the reference for the open/closed
% threshold applied to everything else
tx_m  = measure_pam4_eyes(tx_eye,  upsample_factor, outlier_pct, NaN,                open_frac);
rx_m  = measure_pam4_eyes(rx_eye,  upsample_factor, outlier_pct, tx_m.height(2),     open_frac);
eq_m  = measure_pam4_eyes(eq_eye,  upsample_factor, outlier_pct, tx_m.height(2),     open_frac);
rtl_m = measure_pam4_eyes(rtl_eye, upsample_factor, outlier_pct, tx_m.height(2),     open_frac);

names   = {'TX Ideal', 'RX Noisy', 'EQ Behav', 'EQ RTL'};
results = {tx_m, rx_m, eq_m, rtl_m};

%% SUMMARY TABLES
eye_names = {'Bottom (-3/-1)', 'Middle (-1/+1)', 'Top (+1/+3)'};

fprintf('=============== PER-EYE BREAKDOWN ===============\n');
fprintf('%-16s', 'Eye Height (V)'); fprintf(' | %9s', names{:}); fprintf('\n');
for e = 1:3
    fprintf('%-16s', ['  ' eye_names{e}]);
    for s = 1:4, fprintf(' | %9.3f', results{s}.height(e)); end
    fprintf('\n');
end
fprintf('%-16s', 'Eye Width (UI)'); fprintf(' | %9s', names{:}); fprintf('\n');
for e = 1:3
    fprintf('%-16s', ['  ' eye_names{e}]);
    for s = 1:4, fprintf(' | %9.3f', results{s}.width(e)); end
    fprintf('\n');
end

fprintf('\n================ WORST-CASE SUMMARY ================\n');
fprintf('%-22s', 'Metric'); fprintf(' | %9s', names{:}); fprintf('\n');
fprintf('%s\n', repmat('-', 1, 22 + 4*12));
rows = {'Worst Eye Height (V)', 'Worst Eye Width (UI)', ...
        'Voltage Margin (V)', 'Timing Margin (UI)', 'RLM'};
for r = 1:5
    fprintf('%-22s', rows{r});
    for s = 1:4
        m = results{s};
        switch r
            case 1, v = min(m.height);
            case 2, v = min(m.width);
            case 3, v = min(m.height) / 2;
            case 4, v = min(m.width) / 2;
            case 5, v = m.rlm;
        end
        fprintf(' | %9.3f', v);
    end
    fprintf('\n');
end

fprintf('\n--- Level stats at eye center (EQ RTL) ---\n');
fprintf('%-8s | %10s | %10s\n', 'Level', 'Mean (V)', 'RMS noise (V)');
lvl_names = {'-3', '-1', '+1', '+3'};
for L = 1:4
    fprintf('%-8s | %10.3f | %10.4f\n', lvl_names{L}, rtl_m.level_mean(L), rtl_m.level_std(L));
end
fprintf('====================================================\n\n');

%% PLOTS
num_traces_plot = min(200, num_traces);
colors = {'g', 'r', 'b', 'm'};
eyes   = {tx_eye, rx_eye, eq_eye, rtl_eye};

fig1 = figure('Name', 'Eye Diagram Comparison', 'Position', [80, 80, 1400, 1000]);
for s = 1:4
    subplot(2, 2, s);
    plot(t_vector, eyes{s}(:, 1:num_traces_plot), colors{s}, 'LineWidth', 0.5);
    hold on; grid on;
    draw_eye_boxes(results{s});
    m = results{s};
    title(sprintf('%s  (worst H=%.3f V, W=%.3f UI)', names{s}, min(m.height), min(m.width)), ...
          'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Time (UI)'); ylabel('Voltage (V)'); ylim([-4.5 4.5]); xlim([0 2]);
end
sgtitle('PAM4 Eye Comparison - measured openings boxed in black', 'FontSize', 14, 'FontWeight', 'bold');

fig2 = figure('Name', 'Behavioral vs RTL Overlay', 'Position', [120, 120, 640, 520]);
h_eq  = plot(t_vector, eq_eye(:, 1:num_traces_plot), 'b',  'LineWidth', 0.8);
hold on;
h_rtl = plot(t_vector, rtl_eye(:, 1:num_traces_plot), 'm--', 'LineWidth', 0.6);
grid on;
xlabel('Time (UI)'); ylabel('Voltage (V)'); ylim([-4.5 4.5]);
title('Behavioral (blue) vs RTL (magenta) Eye Overlay', 'FontSize', 12, 'FontWeight', 'bold');
legend([h_eq(1), h_rtl(1)], {'Behavioral MATLAB', 'Verilog RTL'}, 'Location', 'best');

fig3 = figure('Name', 'Time-Domain Match', 'Position', [160, 160, 640, 520]);
plot(equalized_waveform(settled_start:settled_start+199), 'b-',  'LineWidth', 1.5); hold on;
plot(rtl_waveform(settled_start:settled_start+199),       'm--', 'LineWidth', 1.5);
grid on;
xlabel('Sample Index'); ylabel('Voltage (V)');
title('First 200 Equalized Samples (Post-Settling)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Behavioral', 'RTL', 'Location', 'best');

% Headless runs (matlab -batch) dump PNGs instead of showing windows
if ~usejava('desktop')
    exportgraphics(fig1, 'eye_comparison.png', 'Resolution', 110);
    exportgraphics(fig2, 'eye_overlay.png',    'Resolution', 110);
    exportgraphics(fig3, 'time_domain.png',    'Resolution', 110);
    fprintf('Headless mode: saved eye_comparison.png, eye_overlay.png, time_domain.png\n');
end

%% ================= LOCAL FUNCTIONS (must be at end of script) =============

function m = measure_pam4_eyes(eye_matrix, upsample_factor, outlier_pct, ref_height, open_frac)
    % Measure the 3 sub-eyes of a PAM4 eye matrix (time x traces).
    % ref_height: TX middle-eye height used for the open threshold;
    %             pass NaN to self-reference (used for the TX eye itself).
    [n_time, ~] = size(eye_matrix);
    ideal_levels = [-3, -1, 1, 3];

    % --- 1. Find the eye center: time index with max robust middle opening
    opening = -inf(1, n_time);
    for t = 1:n_time
        v = eye_matrix(t, :);
        pos = sort(v(v > 0));
        neg = sort(v(v < 0));
        if isempty(pos) || isempty(neg), continue; end
        opening(t) = pct_val(pos, outlier_pct) - pct_val(neg, 100 - outlier_pct);
    end
    [~, c] = max(opening);
    m.center_idx = c;

    % --- 2. Classify traces into level families by voltage at center
    vc = eye_matrix(c, :);
    [~, fam] = min(abs(vc(:) - ideal_levels), [], 2); % nearest ideal level

    m.level_mean = nan(1, 4);
    m.level_std  = nan(1, 4);
    for L = 1:4
        vals = vc(fam == L);
        if ~isempty(vals)
            m.level_mean(L) = mean(vals);
            m.level_std(L)  = std(vals);
        end
    end

    % --- 3. Per-eye height and width (eye e sits between family e and e+1)
    m.height  = zeros(1, 3);
    m.width   = zeros(1, 3);
    m.run     = nan(3, 2); % open-run phase endpoints [ts, te] per eye
    m.box     = nan(3, 4); % [v_lo, v_len, valid, -] per eye, for plotting

    if isnan(ref_height)
        % self-reference bootstrap: rough middle opening at center
        ref_height = max(opening(c), eps);
    end

    thr_levels = [-2, 0, 2];               % PAM4 decision thresholds (V)
    band = open_frac * ref_height / 2;     % half-height of the scope band
    n_phase = upsample_factor;             % eye is periodic in 1 UI
    n_tr = size(eye_matrix, 2);

    for e = 1:3
        lo_idx = find(fam == e);     % lower level family
        hi_idx = find(fam == e + 1); % upper level family
        if numel(lo_idx) < 20 || numel(hi_idx) < 20
            continue; % not enough traces to measure -> closed
        end

        % height at center: inner edges with outlier trimming
        hi_inner = pct_val(sort(eye_matrix(c, hi_idx)), outlier_pct);
        lo_inner = pct_val(sort(eye_matrix(c, lo_idx)), 100 - outlier_pct);
        m.height(e) = max(0, hi_inner - lo_inner);

        % width: per phase (pooling the two UI copies of the 2-UI window),
        % count traces inside the threshold band; a phase is "open" when
        % violators stay within the outlier allowance
        viol = zeros(1, n_phase);
        for p = 1:n_phase
            v = eye_matrix([p, p + n_phase], :);
            viol(p) = sum(abs(v(:) - thr_levels(e)) < band);
        end
        allowed = (outlier_pct / 100) * 2 * n_tr;
        is_open = viol <= allowed;

        cp = mod(c - 1, n_phase) + 1;      % center phase
        if ~is_open(cp), continue; end     % closed at the sampling instant

        % expand the open run outward from the center, wrapping across the
        % UI boundary (phase space is circular), capped at 1 full UI
        cnt = 1; ts = cp; te = cp;
        while cnt < n_phase
            nxt = mod(ts - 2, n_phase) + 1; % phase to the left
            if ~is_open(nxt) || nxt == te, break; end
            ts = nxt; cnt = cnt + 1;
        end
        while cnt < n_phase
            nxt = mod(te, n_phase) + 1;     % phase to the right
            if ~is_open(nxt) || nxt == ts, break; end
            te = nxt; cnt = cnt + 1;
        end
        m.width(e) = cnt / upsample_factor;

        % store the actual open run (phase indices) so the plot can draw
        % the box exactly where the eye is open, not merely centered
        m.run(e, :) = [ts, te];
        m.box(e, :) = [lo_inner, m.height(e), 1, 1]; % [v_lo, v_len, valid, -]
    end
    m.n_phase = n_phase;

    % --- 4. RLM (Relative Level Mismatch, PAM4 linearity metric)
    if all(~isnan(m.level_mean))
        V = m.level_mean;
        mid = (V(1) + V(4)) / 2;
        ES1 = (V(2) - mid) / (V(1) - mid);
        ES2 = (V(3) - mid) / (V(4) - mid);
        m.rlm = min([3*ES1, 3*ES2, 2 - 3*ES1, 2 - 3*ES2]);
    else
        m.rlm = 0;
    end
end

function v = pct_val(sorted_vec, p)
    % p-th percentile of an ASCENDING-sorted vector, no toolbox needed
    n = numel(sorted_vec);
    idx = min(n, max(1, ceil(p / 100 * n)));
    v = sorted_vec(idx);
end

function draw_eye_boxes(m)
    % Draw the measured opening of each sub-eye as dashed black boxes,
    % placed exactly over the open phase run (split in two when the run
    % wraps across the UI boundary). Phases repeat each UI, so the box is
    % drawn in both UI copies of the 2-UI view.
    for e = 1:3
        if any(isnan(m.box(e, :))) || m.width(e) <= 0, continue; end
        v_lo = m.box(e, 1); v_len = m.box(e, 2);
        ts = m.run(e, 1); te = m.run(e, 2); n = m.n_phase;
        if ts <= te
            segs = [ts, te];                    % one contiguous segment
        else
            segs = [ts, n; 1, te];              % run wraps: two segments
        end
        for k = 1:size(segs, 1)
            x = (segs(k, 1) - 1) / n;
            w = (segs(k, 2) - segs(k, 1) + 1) / n;
            for ui = 0:1                        % both UI copies
                rectangle('Position', [x + ui, v_lo, w, v_len], ...
                          'EdgeColor', 'k', 'LineWidth', 1.2, 'LineStyle', '--');
            end
        end
    end
end
