% <purpose of this file>
%
% Author:  J.-A. Adrian (JA) <jensalrik.adrian AT gmail.com>
% Date  :  05-Feb-2017 14:04:51
%

clear;
close all;


lenSec     = 10;
sampleRate = 10;

duration = round(lenSec * sampleRate);


stateTransition = [
    1 1/sampleRate;
    0  1
    ];

controlMatrix = [
    (1/sampleRate)^2/2; 
    1/sampleRate
    ];

measurementMatrix = [
    1 0
    ];

% define acceleration magnitude
acceleration = 1.5;

% initial state
state = [0; 0]; 

processNoiseMag = 0.05; % process noise: the variability in how fast 
                        % the Quail is speeding up (stdv of acceleration:
                        % meters/sec^2)
                              
measurementNoiseMag = 10;   % measurement noise: How blind is the 
                            % observer (stdv of location, in meters)

% Ez convert the measurement noise (stdv) into covariance matrix
Ez = measurementNoiseMag^2;
                              
% Ex convert the process noise (stdv) into covariance matrix
Ex = controlMatrix * controlMatrix.' * processNoiseMag^2;

% estimate of initial state variance (covariance matrix)
P = Ex;


measurement  = zeros(duration, 1);
velocityTrue = zeros(duration, 1);
measurementTrue = zeros(duration, 1);
%% Get the Measurement Vector
for iSample = 1:duration
    % Generate the object flight
    accelNoise = processNoiseMag * ...
        [
            ((1/sampleRate)^2/2) * randn(); 
            (1/sampleRate) * randn()
        ];
    
    state = stateTransition * state + controlMatrix * acceleration + accelNoise;
    
    % Generate what the observer sees
    observerVisionNoise = measurementNoiseMag * randn();
    y = measurementMatrix * state + observerVisionNoise;
    
    measurementTrue(iSample) = state(1);
    velocityTrue(iSample)    = state(2);
    
    measurement(iSample) = y;
end


%% Kalman Filtering

% initial state
initialState = [0; 0];

kalman = LinearKalmanFilter(...
    'StateTransition',           stateTransition, ...
    'ControlMatrix',             controlMatrix, ...
    'MeasurementMatrix' ,        measurementMatrix, ...
    'ControlInput',              acceleration, ...
    'InitialState',              initialState, ...
    'InitialEstimateCovariance', P, ...
    'ProcessNoise',              processNoiseMag, ...
    'MeasurementNoise',          measurementNoiseMag ...
    );


thisState = zeros(duration, 2);
for thisSample = 1:duration
    thisState(thisSample, :) = kalman(measurement(thisSample));
end


%% plotting
timeVec = (0:duration-1)' / sampleRate;

figure;
hold on;
plot(timeVec, measurementTrue, 'linewidth', 2);
plot(timeVec, measurement);
plot(timeVec, thisState(:, 1), 'linewidth', 2);
grid on;

xlabel('Time in s');
ylabel('Position in m');
title('Position Estimate');

legend('True Measurement', 'Noisy Measurement', 'Estimate');

figure;
hold on;
plot(timeVec, velocityTrue, 'linewidth', 2);
plot(timeVec, thisState(:, 2));
grid on;

xlabel('Time in s');
ylabel('Velocity in m/s');
title('Velocity Estimate');



%-------------------- Licence ---------------------------------------------
% Copyright (c) 2017, J.-A. Adrian
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%
%	1. Redistributions of source code must retain the above copyright
%	   notice, this list of conditions and the following disclaimer.
%
%	2. Redistributions in binary form must reproduce the above copyright
%	   notice, this list of conditions and the following disclaimer in
%	   the documentation and/or other materials provided with the
%	   distribution.
%
%	3. Neither the name of the copyright holder nor the names of its
%	   contributors may be used to endorse or promote products derived
%	   from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
% IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
% TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
% PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
% HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
% SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
% TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

% End of file: runKalmanFilter.m
