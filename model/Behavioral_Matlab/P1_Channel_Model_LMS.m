%{
Kenneth Lin
Modeling noisy channel behavior featuring adaptive FFE/DFE w/ LMS algorithm
%}

clear; clc; close all; 

% config 
num_symbols = 5000; 
baud_rate = 28e9;
samples_per_sym = 1; 

%% STEP 1: Generate random PAM4 symbols 
% Generate integers uniformly distributed in the range [0, 3]
rng(42);
raw_symbols = randi([0, 3], num_symbols, 1);

%% STEP 2: PAM4 symbol mapping
% mapping: 0 -> -3,  1 -> -1,  2 -> +1,  3 -> +3 
mapped_signals = zeros(num_symbols, 1);
tx_waveform = 2 * raw_symbols - 3;

%% STEP 3: Defining and applying noise
h_ideal = [1.0]; %ideal channel 
h_lossy = [0.8, 0.3, 0.1]; % lossy channel w/ ISI

rx_waveform_lossy = filter(h_lossy, 1, tx_waveform);

%{
noise_amplitude = 0.1; 
thermal_noise = noise_amplitude * randn(size(rx_waveform_lossy));
rx_waveform_noisy = rx_waveform_lossy + thermal_noise;
%}
% thermal_noise(# samples, impedance, sample rate, temp (K)
noise = thermal_noise(length(rx_waveform_lossy), 50, 28e9, 300); 
rx_waveform_noisy = rx_waveform_lossy + noise;

%% STEP 4: Calculating PAM4 slicer and error test

test_samples = rx_waveform_noisy(1:100);

guessed_levels = zeros(size(test_samples));

%logical indexing
guessed_levels(test_samples >= 2) = 3;
guessed_levels(test_samples >= 0 & test_samples < 2) = 1;
guessed_levels(test_samples >= -2 & test_samples < 0) = -1;
guessed_levels(test_samples < -2) = -3;

error_vals = test_samples - guessed_levels; 

mean_abs_error = mean(abs(error_vals));
mean_squared_error = mean(error_vals .^ 2);

fprintf('--- Vectorized Slicer Test ---\n');
fprintf('Mean Absolute Error (MAE) : %.3f Volts\n', mean_abs_error);
fprintf('Mean Squared Error (MSE)  : %.3f\n', mean_squared_error);

%% STEP 5: Adaptive equalizer (FFE + DFE) 
num_ffe_taps = 15; % 15-tap FFE
ffe_weights = [1.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0]; % effectively a standard impulse --> does nothing
% num_ffe_taps = 3; % 3-tap FFE
% ffe_weights = [1.0; 0.0; 0.0];

dfe_weight = 0.0;
mu = 0.005; % Least mean squares (LMS) step size (learning rate)

% preallocate arrays 
equalized_waveform = zeros(num_symbols, 1);
error_history = zeros(num_symbols, 1);

% shift registers, mimicking DFF in Verilog 
rx_shift_reg = zeros(num_ffe_taps, 1); 
prev_decision = 0;

% Training adaptive equalizer 
for i = 1:num_symbols
    rx_shift_reg = [rx_waveform_noisy(i); rx_shift_reg(1:end-1)];
    ffe_out = ffe_weights' * rx_shift_reg; 
    dfe_out = (dfe_weight * prev_decision); 

    % equalized signal 
    eq_signal = ffe_out - dfe_out; 
    equalized_waveform(i) = eq_signal;

    % slicer 
    if eq_signal >= 2
        decision = 3;
    elseif eq_signal >= 0 && eq_signal < 2
        decision = 1;
    elseif eq_signal >= -2 && eq_signal < 0
        decision = -1;
    else
        decision = -3;
    end
    
    % Calculate Error (Actual - Ideal)
    error_val = eq_signal - decision;
    error_history(i) = error_val;

    % LMS adaptation 
    ffe_weights = ffe_weights - (mu * error_val * rx_shift_reg);
    dfe_weight = dfe_weight + (mu * error_val * prev_decision);

    prev_decision = decision; 
end 

figure;
plot(abs(error_history), 'LineWidth', 1.5);
grid on;
title('LMS Equalizer Learning Curve (Error vs. Time)');
xlabel('Clock Cycles (Symbols)');
ylabel('Absolute Error (Volts)');

%% STEP 6: eye diagrams 

upsample_factor = 8;
samples_per_trace = upsample_factor * 2;
tx_smooth = interp(tx_waveform, upsample_factor);
rx_smooth = interp(rx_waveform_noisy, upsample_factor);

% Discard the first 300 samples to let the filter settle
settled_tx = tx_waveform(301:end); 
settled_rx = rx_waveform_noisy(301:end); 
settled_eq = equalized_waveform(301:end);

upsample_factor = 8;
samples_per_trace = upsample_factor * 2; % 2 Unit Intervals wide

% Interpolate to create smooth oscilloscope traces
tx_smooth = interp(settled_tx, upsample_factor);
rx_smooth = interp(settled_rx, upsample_factor);
eq_smooth = interp(settled_eq, upsample_factor);

% Strip away the interpolation filter group delay
interp_delay_offset = 29; 
tx_smooth = tx_smooth(interp_delay_offset:end);
rx_smooth = rx_smooth(interp_delay_offset:end);
eq_smooth = eq_smooth(interp_delay_offset:end);

% Calculate how many full 2-UI traces we can draw
num_traces = floor(length(eq_smooth) / samples_per_trace) - 1;
t_vector = (0:samples_per_trace-1) / upsample_factor;

% Reshape the long 1D arrays into 2D matrices for overlapping plots
tx_eye_matrix = reshape(tx_smooth(1:num_traces*samples_per_trace), samples_per_trace, num_traces);
rx_eye_matrix = reshape(rx_smooth(1:num_traces*samples_per_trace), samples_per_trace, num_traces);
eq_eye_matrix = reshape(eq_smooth(1:num_traces*samples_per_trace), samples_per_trace, num_traces);

% eye diagrams, metrics 

% eye width calculation
voltage_margin = 0.4; % visually from Tx
max_violators = 0.02 * num_traces; % Allow 2% statistical noise outliers
tx_open_mask = zeros(1, samples_per_trace);

for t_idx = 1:samples_per_trace
    violators = sum(tx_eye_matrix(t_idx, :) > -voltage_margin & tx_eye_matrix(t_idx, :) < voltage_margin);
    if violators <= max_violators
        tx_open_mask(t_idx) = 1;
    end
end
% Focus on the central eye opening (around the center of the 2-UI plot)
tx_edges = diff([0, tx_open_mask, 0]);
tx_starts = find(tx_edges == 1);
tx_ends = find(tx_edges == -1) - 1;
[~, max_idx] = max(tx_ends - tx_starts);
tx_start_t = t_vector(tx_starts(max_idx));
tx_end_t = t_vector(tx_ends(max_idx));
tx_width_ui = (tx_end_t - tx_start_t);

% 2. Calculate RX Equalized Eye Width
eq_open_mask = zeros(1, samples_per_trace);
for t_idx = 1:samples_per_trace
    violators = sum(eq_eye_matrix(t_idx, :) > -voltage_margin & eq_eye_matrix(t_idx, :) < voltage_margin);
    if violators <= max_violators
        eq_open_mask(t_idx) = 1;
    end
end
eq_edges = diff([0, eq_open_mask, 0]);
eq_starts = find(eq_edges == 1);
eq_ends = find(eq_edges == -1) - 1;
if ~isempty(eq_starts)
    [~, max_idx_eq] = max(eq_ends - eq_starts);
    eq_start_t = t_vector(eq_starts(max_idx_eq));
    eq_end_t = t_vector(eq_ends(max_idx_eq));
    eq_width_ui = (eq_end_t - eq_start_t);
else
    eq_start_t = 1.0; eq_end_t = 1.0; eq_width_ui = 0.0; % Completely closed
end

% Ideal TX Eye
figure('Name', 'TX Eye', 'Position', [100, 100, 450, 450]);
plot(t_vector, tx_eye_matrix(:, 1:min(200, num_traces)), 'g', 'LineWidth', 0.5);
grid on; 
title('Ideal TX PAM4 Eye'); 
xlabel('Symbol Time (UI)'); ylabel('Voltage (V)');
ylim([-5 5]);

% 2. Noisy Rx eye before equalization
figure('Name', 'RX Before EQ', 'Position', [350, 100, 450, 450]);
plot(t_vector, rx_eye_matrix(:, 1:min(200, num_traces)), 'r', 'LineWidth', 0.5);
grid on; 
title('Rx Input Before Equalization'); 
xlabel('Symbol Time (UI)'); ylabel('Voltage (V)');
ylim([-5 5]);

% 3. Equalized eye after FFE/DFE
figure('Name', 'RX After Equalization', 'Position', [700, 100, 450, 450]);
plot(t_vector, eq_eye_matrix(:, 1:min(200, num_traces)), 'b', 'LineWidth', 0.5);
grid on;
title('After LMS Equalization (FFE + DFE)');
xlabel('Symbol Time (UI)'); ylabel('Voltage (V)');
ylim([-5 5]);

%% STEP 7: performance metrics 
fprintf('\n--- Learning Algorithm Performance Analysis ---\n');

% steady-state MSE for last 20% of symbols
steady_start_idx = round(num_symbols * 0.8);
steady_state_errors = error_history(steady_start_idx:end);
ss_mse = mean(steady_state_errors .^ 2);
fprintf('Steady-State MSE: %.4f\n', ss_mse);

PerformanceTable = ... 
    eye_stats(tx_eye_matrix, eq_eye_matrix, t_vector, upsample_factor, 0.02);