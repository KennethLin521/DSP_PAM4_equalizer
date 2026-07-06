function n = thermal_noise(num_samples, R, fs, T)
% Johnson-Nyquist thermal noise generator
% num_samples : number of samples to generate
% R           : resistance in ohms
% fs          : sample rate (Hz)
% T           : temperature (Kelvin)

k = 1.380649e-23; % Boltzmann constant
B = fs/2;         % noise bandwidth
v_rms = sqrt(4 * k * T * R * B);

n = v_rms * randn(num_samples, 1);
end
