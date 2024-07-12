%% Produce filters INIT

% the frequency parameters
% for bandpass, are specified here.

fs = 48828.125;
f_nyquist = fs/2;


f_max = 9500 / f_nyquist; 

% Frequency bands of special interest. 

band1 = [6375, 9000]; % 8500, max frequency we want to look at
band2 = [3541, 6375];
band3 = [1950, 3541];
band4 = [956, 1950];
band5 = [779, 956];
band6 = [657, 779];
band7 = [550, 657]; % 550- lowest we want to look at subject to change


bands_optimized = zeros(7, 2);

% Band 1

lower_freq = band1(1);
upper_freq = band1(2);

bands_optimized(1,1) =  (lower_freq/f_nyquist) + 0.068;
bands_optimized(1,2) = (upper_freq/f_nyquist);

% Band 2

lower_freq = band2(1);
upper_freq = band2(2);

bands_optimized(2,1) = (lower_freq/f_nyquist) -0.059;
bands_optimized(2,2) = (upper_freq/f_nyquist);

% Band 3

lower_freq = band3(1);
upper_freq = band3(2);

bands_optimized(3,1) =  (lower_freq/f_nyquist) - 0.001;
bands_optimized(3,2) = (upper_freq/f_nyquist) + 0.0075;

% Band 4

lower_freq = band4(1);
upper_freq = band4(2);

bands_optimized(4, 1) =  (lower_freq/f_nyquist) ;
bands_optimized(4,2) = (upper_freq/f_nyquist) - 0.01;

% Band 5

lower_freq = band5(1);
upper_freq = band5(2);

bands_optimized(5, 1) =  (lower_freq/f_nyquist);
bands_optimized(5,2) = (upper_freq/f_nyquist);

% Band 6
lower_freq = band6(1);
upper_freq = band6(2);

bands_optimized(6, 1) =  (lower_freq/f_nyquist);
bands_optimized(6,2) = (upper_freq/f_nyquist);


% Band 7

lower_freq = band7(1);
upper_freq = band7(2);

bands_optimized(7, 1) =  (lower_freq/f_nyquist);
bands_optimized(7,2) = (upper_freq/f_nyquist);

% bands_norm is tweaked from the above stated frequencies


%% Windowed sinc-delay 
% Viktigt: N är JÄMN integer , fhigh är i Hz, delay 0 < delay < 1
% Hur genererar man dessa på bästa rimliga vis??? 


function a = getCoeffs(N, fhigh, delay, f_nyquist)

wc = pi * fhigh / f_nyquist;
a = zeros(1, 2*N + 1);

w = blackman(2*N +1);

for i = -N: N
    a(i + N +1) = w(i + N + 1) * sin(wc * (i - delay)) / (pi * (i - delay));
end

end


%% Mode 1 - filter generator

function returner = getCoeffsMode1(delay, lower, upper, f_nyquist)

input_lower = lower;
input_upper = upper;

b = fir1(20, [input_lower, input_upper], 'bandpass');
sinc_delay = getCoeffs(18, f_nyquist, delay, f_nyquist);

temp_filter = conv(b, sinc_delay);
[h, ~] = freqz(temp_filter, 1, 9500);

[peak_gain, ~] = max(h);
peak_gain = abs(peak_gain);

norm = 1/peak_gain;
returner = temp_filter * norm;


end

%% Mode 2 - filter generator

function returner = getCoeffsMode2(delay, lower, upper, f_nyquist)

input_lower = lower;
input_upper = upper;

% make the filter

b = fir1(28, [input_lower, input_upper], 'bandpass');
sinc_delay = getCoeffs(14, f_nyquist, delay, f_nyquist);
temp_filter = conv(b, sinc_delay);


% normilize, very High definition
[h, ~] = freqz(temp_filter, 1, 5000);
[peak_gain, ~] = max(h);
peak_gain = abs(peak_gain);
norm = 1/peak_gain;
returner = temp_filter * norm;

end

%% Mode 3 - filter generator

function returner = getCoeffsMode3(delay, lower, upper, f_nyquist)

input_lower = lower;
input_upper = upper;

b = fir1(36, [input_lower, input_upper], 'bandpass');
sinc_delay = getCoeffs(10, f_nyquist, delay, f_nyquist);
temp_filter = conv(b, sinc_delay);


[h, ~] = freqz(temp_filter, 1, 1024);
[peak_gain, ~] = max(h);
peak_gain = abs(peak_gain);
norm = 1/peak_gain;
returner = temp_filter * norm;

end

%% mode 4

function returner = getCoeffsMode4(delay, lower, upper, f_nyquist)

input_lower = lower;
input_upper = upper;

b = fir1(44, [input_lower, input_upper], 'bandpass');
sinc_delay = getCoeffs(6, f_nyquist, delay, f_nyquist);
temp_filter = conv(b, sinc_delay);


[h, ~] = freqz(temp_filter, 1, 1024);
[peak_gain, ~] = max(h);
peak_gain = abs(peak_gain);
norm = 1/peak_gain;
returner = temp_filter * norm;


% Band 4 is decent. The lower ones are not amazing...


end

%% mode 5 - 7

function returner = getCoeffsMode57(delay, lower, upper, f_nyquist)

input_lower = lower;
input_upper = upper;

b = fir1(44, [input_lower, input_upper], 'bandpass');
sinc_delay = getCoeffs(6, f_nyquist, delay, f_nyquist);
temp_filter = conv(b, sinc_delay);


[h, ~] = freqz(temp_filter, 1, 1024);
[peak_gain, ~] = max(h);
peak_gain = abs(peak_gain);
norm = 1/peak_gain;
returner = temp_filter * norm;


end



%% All modes - filter generator

function returner = generateCoeffs(mode, delay, bands_norm, f_nyquist)

if mode == 1
    returner = getCoeffsMode1(delay, bands_norm(1,1), bands_norm(1,2), f_nyquist);

elseif mode == 2

    returner = getCoeffsMode2(delay, bands_norm(2,1), bands_norm(2,2), f_nyquist);

elseif mode == 3

    returner = getCoeffsMode3(delay, bands_norm(3,1), bands_norm(3,2), f_nyquist);

elseif mode == 4

    returner = getCoeffsMode4(delay, bands_norm(4,1), bands_norm(4,2), f_nyquist);

elseif mode == 5

    returner = getCoeffsMode57(delay, bands_norm(5,1), bands_norm(5,2), f_nyquist);

elseif mode == 6

    returner = getCoeffsMode57(delay, bands_norm(6,1), bands_norm(6,2), f_nyquist);

elseif mode == 7

    returner = getCoeffsMode57(delay, bands_norm(7,1), bands_norm(7,2), f_nyquist);
end

end

%% Generate all filters.

% Creates a table structured in the following way
% mode1 delay1 coeff0, coeff2, ... coeff54 0 
% .
% .
% . 
% mode7 delayLast coeff0 coeff2 ... coeff54 0
%
% Zero for optimal cpp implementation


% parameters

definition_delay = 101;
delays_vec = linspace(0, 1, definition_delay);
amount_of_modes = 7;

all_coefficients = cell(definition_delay * amount_of_modes, 59);

for mode = 1:amount_of_modes
    for delay = 1: length(delays_vec)


        delay_input = delays_vec(delay);


        % No divide by zero
        if delay_input == 0

            delay_input = delay_input + 0.00001;
        end

        if delay_input == 1

            delay_input = delay_input - 0.00001;
        end

        
        
        coefficients = generateCoeffs(mode, delay_input, bands_optimized, f_nyquist);

        index = (mode - 1) * definition_delay + delay;

        all_coefficients{index, 1} = mode;
        all_coefficients{index, 2} = delays_vec(delay);
        
        all_coefficients(index, 3:end) = num2cell(coefficients);

    end

end

%% make matrix
% Assuming all_coefficients is your cell array

[numRows, numCols] = size(all_coefficients);

% Initialize a numeric array
numeric_array = zeros(numRows, numCols);

% Loop through the cell array and convert each element to a numeric value
for i = 1:numRows
    for j = 1:numCols
        if isnumeric(all_coefficients{i, j})
            numeric_array(i, j) = all_coefficients{i, j};
        else
            numeric_array(i, j) = NaN; % Handle non-numeric values appropriately
        end
    end
end

% Display the resulting numeric array
disp(numeric_array);

%filename = 'coefficients.txt';
%dlmwrite(filename, numeric_array, 'delimiter', ' ', 'precision', '%.6f');


%% TroubleShoot

coos = check_if_valid(4, 50, :);


%% C++ code generation

% make one in matlab to check if valid

% init
amount_modes = 7;
amount_coeffs = 56; % skip last zero
amount_delays = 101;

% nake variable
filename = 'cppcode.txt';
fileID = fopen(filename, 'w');
fprintf(fileID, 'float filter_coeffs [');
fprintf(fileID, num2str(amount_modes));
fprintf(fileID, '][');
fprintf(fileID,num2str(amount_delays));
fprintf(fileID, '][');
fprintf(fileID, num2str(amount_coeffs));
fprintf(fileID, '] = {');

%matlab equivalent
check_if_valid = zeros(amount_of_modes, amount_delays, amount_coeffs);

for m = 1:amount_modes
    fprintf(fileID, '{');
    
    for d = 1:amount_delays
        fprintf(fileID, '{');
        
        index = (m-1)*amount_delays + d;
        print_array = cell2mat(all_coefficients(index, 3:end));

        for c=1: amount_coeffs

            fprintf(fileID, num2str(print_array(c), 7));
            check_if_valid(m, d, c) = print_array(c);

            if c ~= amount_coeffs
                fprintf(fileID, ',');

            end

        end

        fprintf(fileID, '},');
        fprintf(fileID, '\n');

    end
    fprintf(fileID, '},');

end

fprintf(fileID, '};');




%% comment
%{

double coefficients[MAX_MODE][MAX_DELAY][MAX_COEFFICIENTS] = {
        // Mode 0
        {
            {1.1, 2.2, 3.3, 4.4},    // Delay 0 coefficients
            {5.5, 6.6, 7.7, 8.8},    // Delay 1 coefficients
            {9.9, 10.1, 11.11, 12.12} // Delay 2 coefficients
        },
        // Mode 1
        {
            {13.13, 14.14, 15.15, 16.16}, // Delay 0 coefficients
            {17.17, 18.18, 19.19, 20.20}, // Delay 1 coefficients
            {21.21, 22.22, 23.23, 24.24}  // Delay 2 coefficients
        }
    };


important functions:

{ }

single(double) converts to float
cell2mat(numb, 3:end)

    
%}


%% inspect

total_filter = squeeze(coos);

[gd, w] = grpdelay(total_filter, 1, 512); % Specify 'whole' and number of points directly

figure;
plot(w/pi, gd, 'b', 'LineWidth', 1.5); hold on;
plot(w/pi, delay * ones(size(w)), 'r--', 'LineWidth', 1.5);
title('Group Delay of the Total Filter');
xlabel('Normalized Frequency (\times\pi rad/sample)');
ylabel('Group Delay (samples)');
legend('Actual Group Delay', 'Desired Group Delay');
grid on;

figure;
freqz(total_filter, 1);
title('Frequency Response of the Total Filter');
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');

[b_total, a_total] = tf(total_filter, 1);
disp('Numerator coefficients of the transfer function:');
disp(b_total);
disp('Denominator coefficients of the transfer function:');
disp(a_total);
