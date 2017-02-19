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


% End of file: LinearKalmanFilter.m
