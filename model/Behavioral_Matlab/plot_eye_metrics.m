%{
Kenneth Lin
Comprehensive eye diagram metrics: ideal TX vs noisy RX vs behavioral EQ vs RTL EQ
Computes: eye width, eye height, vertical/horizontal margins, BER-relevant statistics
%}

clear; clc; close all;

%% CONFIG
num_symbols = 5000;
upsample_factor = 8;
samples_per_trace = upsample_factor * 2; % 2 UI wide
settled_start = 301; % discard first 300 for filter settling
volts_to_lsb = 31;
lsb_to_volt = 1 / volts_to_lsb;

%% LOAD DATA (same as P3_Channel_Model_cosim.m)
rng(42);
raw_symbols = randi([0, 3], num_symbols, 1);
tx_waveform = 2 * raw_symbols - 3;

h_lossy = [0.8, 0.3, 0.1];
rx_waveform_lossy = filter(h_lossy, 1, tx_waveform);
noise = thermal_noise(length(rx_waveform_lossy), 50, 28e9, 300);
rx_waveform_noisy = rx_waveform_lossy + noise;

%% BEHAVIORAL MATLAB MODEL (matches P3)
num_ffe_taps = 3;
ffe_weights = [1.0; 0.0; 0.0];
dfe_weight = 0.0;
mu = 1/256; % matched to RTL Q8.8 step size

equalized_waveform = zeros(num_symbols, 1);
rx_shift_reg = zeros(num_ffe_taps, 1);
prev_decision = 0;

adc_bits = 8;
adc_max = 2^(adc_bits-1) - 1;
adc_min = -2^(adc_bits-1);

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

    % Slicer
    if eq_signal >= round(2.0 * volts_to_lsb)
        decision = round(3.0 * volts_to_lsb);
    elseif eq_signal >= 0 && eq_signal < round(2.0 * volts_to_lsb)
        decision = round(1.0 * volts_to_lsb);
    elseif eq_signal >= round(-2.0 * volts_to_lsb) && eq_signal < 0
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

%% LOAD RTL OUTPUT
rtl_fid = fopen('eq_output_dump.hex', 'r');
rtl_hex = textscan(rtl_fid, '%s');
fclose(rtl_fid);
rtl_output_int = hex2dec(rtl_hex{1});
rtl_output_int(rtl_output_int > 32767) = rtl_output_int(rtl_output_int > 32767) - 65536;
rtl_waveform = rtl_output_int * lsb_to_volt;

%% PREPARE DATA FOR PLOTTING
settled_tx = tx_waveform(settled_start:end);
settled_rx = rx_waveform_noisy(settled_start:end);
settled_eq = equalized_waveform(settled_start:end);
settled_rtl = rtl_waveform(settled_start:end);

% Interpolate for smooth eye plots
tx_smooth = interp(settled_tx, upsample_factor);
rx_smooth = interp(settled_rx, upsample_factor);
eq_smooth = interp(settled_eq, upsample_factor);
rtl_smooth = interp(settled_rtl, upsample_factor);

% Remove interpolation filter group delay
interp_delay = 29;
tx_smooth = tx_smooth(interp_delay:end);
rx_smooth = rx_smooth(interp_delay:end);
eq_smooth = eq_smooth(interp_delay:end);
rtl_smooth = rtl_smooth(interp_delay:end);

% Ensure all same length for reshaping
min_len = min([length(tx_smooth), length(rx_smooth), length(eq_smooth), length(rtl_smooth)]);
num_traces = floor(min_len / samples_per_trace) - 1;
t_vector = (0:samples_per_trace-1) / upsample_factor;

tx_eye = reshape(tx_smooth(1:num_traces*samples_per_trace), samples_per_trace, num_traces);
rx_eye = reshape(rx_smooth(1:num_traces*samples_per_trace), samples_per_trace, num_traces);
eq_eye = reshape(eq_smooth(1:num_traces*samples_per_trace), samples_per_trace, num_traces);
rtl_eye = reshape(rtl_smooth(1:num_traces*samples_per_trace), samples_per_trace, num_traces);

num_traces_plot = min(200, num_traces);

%% EYE METRICS FUNCTION
function [eye_width_ui, eye_height_v, v_margin, h_margin, snr_db] = compute_eye_stats(eye_matrix, t_vector, upsample_factor, slicer_threshold)
    samples_per_trace = length(t_vector);

    % Find eye center (time of maximum opening)
    opening = zeros(1, samples_per_trace);
    for t_idx = 1:samples_per_trace
        v_slice = eye_matrix(t_idx, :);
        pos_vals = v_slice(v_slice > 0);
        neg_vals = v_slice(v_slice < 0);
        if ~isempty(pos_vals) && ~isempty(neg_vals)
            opening(t_idx) = min(pos_vals) - max(neg_vals);
        end
    end
    [~, center_idx] = max(opening);

    % Vertical eye height at center
    center_trace = eye_matrix(center_idx, :);
    pos_inner = min(center_trace(center_trace > 0));
    neg_inner = max(center_trace(center_trace < 0));
    if isempty(pos_inner) || isempty(neg_inner)
        eye_height_v = 0;
    else
        eye_height_v = pos_inner - neg_inner;
    end

    % Horizontal eye width (measure how many samples stay within +/-slicer_threshold of center)
    center_level = center_trace(center_idx);
    mask = zeros(1, samples_per_trace);
    for t_idx = 1:samples_per_trace
        v_slice = eye_matrix(t_idx, :);
        within = sum(abs(v_slice - center_level) < slicer_threshold);
        if within > 0.8 * length(v_slice) % 80% of traces within margin
            mask(t_idx) = 1;
        end
    end

    edges = diff([0, mask, 0]);
    starts = find(edges == 1);
    ends = find(edges == -1) - 1;
    if isempty(starts)
        eye_width_ui = 0;
    else
        widths = (ends - starts + 1) / upsample_factor;
        [~, max_idx] = max(widths);
        eye_width_ui = widths(max_idx);
    end

    % Margins: vertical (noise margin) and horizontal (timing margin)
    v_margin = eye_height_v / 2; % half-opening is the margin to the slicer threshold
    h_margin = (eye_width_ui / 2 - 0.5); % margin beyond center UI

    % Simple SNR estimate: signal power / noise power in steady state
    steady_idx = find(eye_matrix > -2 & eye_matrix < 2); % mid-range samples
    if ~isempty(steady_idx)
        noise_std = std(eye_matrix(steady_idx));
        signal_power = mean(eye_matrix(:).^2);
        snr_db = 10 * log10(signal_power / (noise_std^2 + eps));
    else
        snr_db = 0;
    end
end

%% COMPUTE METRICS FOR ALL FOUR EYES
slicer_threshold = 0.2; % volts, how far from ideal level is considered an error
[tx_width, tx_height, ~, ~, ~] = compute_eye_stats(tx_eye, t_vector, upsample_factor, slicer_threshold);
[rx_width, rx_height, rx_v_margin, rx_h_margin, rx_snr] = compute_eye_stats(rx_eye, t_vector, upsample_factor, slicer_threshold);
[eq_width, eq_height, eq_v_margin, eq_h_margin, eq_snr] = compute_eye_stats(eq_eye, t_vector, upsample_factor, slicer_threshold);
[rtl_width, rtl_height, rtl_v_margin, rtl_h_margin, rtl_snr] = compute_eye_stats(rtl_eye, t_vector, upsample_factor, slicer_threshold);

%% PRINT SUMMARY TABLE
fprintf('\n========== EYE DIAGRAM METRICS ==========\n');
fprintf('%-25s | %10s | %10s | %10s | %10s\n', 'Metric', 'TX Ideal', 'RX Noisy', 'EQ Behav', 'EQ RTL');
fprintf('%-25s | %10s | %10s | %10s | %10s\n', repmat('-', 1, 25), repmat('-', 1, 10), repmat('-', 1, 10), repmat('-', 1, 10), repmat('-', 1, 10));
fprintf('%-25s | %10.3f | %10.3f | %10.3f | %10.3f\n', 'Eye Width (UI)', tx_width, rx_width, eq_width, rtl_width);
fprintf('%-25s | %10.3f | %10.3f | %10.3f | %10.3f\n', 'Eye Height (V)', tx_height, rx_height, eq_height, rtl_height);
fprintf('%-25s | %10.3f | %10.3f | %10.3f | %10.3f\n', 'Vertical Margin (V)', tx_height/2, rx_v_margin, eq_v_margin, rtl_v_margin);
fprintf('%-25s | %10.3f | %10.3f | %10.3f | %10.3f\n', 'Horizontal Margin (UI)', 0.5, rx_h_margin, eq_h_margin, rtl_h_margin);
fprintf('%-25s | %10.3f | %10.3f | %10.3f | %10.3f\n', 'SNR (dB)', 0, rx_snr, eq_snr, rtl_snr);
fprintf('==========================================\n\n');

%% PLOT ALL FOUR EYES
figure('Name', 'Eye Diagram Comparison', 'Position', [100, 100, 1400, 1000]);

% TX Ideal
subplot(2, 2, 1);
plot(t_vector, tx_eye(:, 1:num_traces_plot), 'g', 'LineWidth', 0.5);
grid on;
title('TX Ideal PAM4 Eye', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (UI)'); ylabel('Voltage (V)');
ylim([-4 4]);

% RX Noisy
subplot(2, 2, 2);
plot(t_vector, rx_eye(:, 1:num_traces_plot), 'r', 'LineWidth', 0.5);
grid on;
title(sprintf('RX Before EQ (W=%.3f UI, H=%.3f V)', rx_width, rx_height), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (UI)'); ylabel('Voltage (V)');
ylim([-4 4]);

% Behavioral EQ
subplot(2, 2, 3);
plot(t_vector, eq_eye(:, 1:num_traces_plot), 'b', 'LineWidth', 0.5);
grid on;
title(sprintf('Behavioral EQ (W=%.3f UI, H=%.3f V)', eq_width, eq_height), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (UI)'); ylabel('Voltage (V)');
ylim([-4 4]);

% RTL EQ
subplot(2, 2, 4);
plot(t_vector, rtl_eye(:, 1:num_traces_plot), 'm', 'LineWidth', 0.5);
grid on;
title(sprintf('Verilog RTL EQ (W=%.3f UI, H=%.3f V)', rtl_width, rtl_height), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (UI)'); ylabel('Voltage (V)');
ylim([-4 4]);

sgtitle('4-Way Eye Diagram Comparison: TX vs RX vs Behavioral vs RTL', 'FontSize', 14, 'FontWeight', 'bold');

%% OVERLAY PLOT: Behavioral vs RTL (direct comparison)
figure('Name', 'Behavioral vs RTL Overlay', 'Position', [100, 1100, 600, 500]);
plot(t_vector, eq_eye(:, 1:num_traces_plot), 'b', 'LineWidth', 0.8, 'DisplayName', 'Behavioral MATLAB');
hold on;
plot(t_vector, rtl_eye(:, 1:num_traces_plot), 'm--', 'LineWidth', 0.6, 'DisplayName', 'Verilog RTL');
grid on;
xlabel('Time (UI)'); ylabel('Voltage (V)');
title('Behavioral vs RTL Eye Overlay', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-4 4]);
legend('Location', 'best');

%% DIAGNOSTIC: Time-domain waveform first 200 samples (post-settling)
figure('Name', 'Time-Domain Waveforms', 'Position', [750, 1100, 600, 500]);
plot(equalized_waveform(settled_start:settled_start+199), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Behavioral');
hold on;
plot(rtl_waveform(settled_start:settled_start+199), 'm--', 'LineWidth', 1.5, 'DisplayName', 'RTL');
grid on;
xlabel('Sample Index'); ylabel('Voltage (V)');
title('First 200 Equalized Samples (Post-Settling)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

fprintf('Plots complete. Ready for synthesis & APR!\n');
