classdef LinearKalmanFilter < matlab.System
%LINEARKALMANFILTER <purpose in one line!>
% -------------------------------------------------------------------------
% Implementation and naming convention following Wikipedia:
% https://en.wikipedia.org/wiki/Kalman_filter
%
% Linear Properties:
%	propA - <description>
%	propB - <description>
%
% Linear Methods:
%	doThis - <description>
%	doThat - <description>
%
% Author :  J.-A. Adrian (JA) <jensalrik.adrian AT gmail.com>
% Date   :  05-Feb-2017 13:58:22
%

% History:  v0.1   initial version, 05-Feb-2017 (JA)
%




properties (Nontunable)
    StateTransition;
    ControlMatrix;
    MeasurementMatrix;
    ControlInput;
    
    InitialState = 0;
    InitialEstimateCovariance;
    
    MeasurementNoise = 0;
    ProcessNoise = 0;
end


properties (Access = private)
    EstimateCovariance;
    MeasurementCovariance;
    ProcessNoiseCovariance
    
    StateEstimate;
end



methods
    function [self] = LinearKalmanFilter(varargin)
        setProperties(self, nargin, varargin{:});
    end
end


methods (Access = protected)
    function setupImpl(self)
        self.StateEstimate      = self.InitialState;
        self.EstimateCovariance = self.InitialEstimateCovariance;
        
        self.MeasurementCovariance  = self.MeasurementNoise .^ 2;
        self.ProcessNoiseCovariance = self.InitialEstimateCovariance;
    end
    
    function [thisState] = stepImpl(self, measurement)
        %%%%%%%%%%%%%%%%%%%%%%%
        %%% Predcition step %%%
        %%%%%%%%%%%%%%%%%%%%%%%
        % state prediction
        self.StateEstimate = ...
            self.StateTransition * self.StateEstimate + ...
            self.ControlMatrix * self.ControlInput;
        
        % state covariance prediction
        self.EstimateCovariance = ...
            (self.StateTransition * self.EstimateCovariance * self.StateTransition.') + ...
            self.ProcessNoiseCovariance;
        
        % Kalman Gain
        denom = ...
            self.MeasurementMatrix * ...
            self.EstimateCovariance * ...
            self.MeasurementMatrix.' + ...
            self.MeasurementCovariance;
        KalGain = self.EstimateCovariance * self.MeasurementMatrix.' * pinv(denom);
        
        %%%%%%%%%%%%%%%%%%%
        %%% Update step %%%
        %%%%%%%%%%%%%%%%%%%
        residual = measurement - self.MeasurementMatrix * self.StateEstimate;
        
        % state update
        self.StateEstimate = self.StateEstimate + KalGain * residual;
        
        % Covariance update
        identMatrix = eye(size(self.StateEstimate, 1));
        self.EstimateCovariance = ...
            (identMatrix - KalGain * self.MeasurementMatrix) * ...
            self.EstimateCovariance;
        
        
        thisState = self.StateEstimate;
    end
    
    function resetImpl(self)
        self.StateEstimate      = self.InitialState;
        self.EstimateCovariance = self.InitialEstimateCovariance;
        
        self.MeasurementCovariance  = self.MeasurementNoise .^ 2;
        self.ProcessNoiseCovariance = self.InitialEstimateCovariance;
    end
end



end



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

% End of file: LinearKalmanFilter.m
