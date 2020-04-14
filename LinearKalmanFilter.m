classdef LinearKalmanFilter < matlab.System
%LINEARKALMANFILTER Implements the Kalman filter as a System Object
% -------------------------------------------------------------------------
% Implementation and naming convention following Wikipedia:
% https://en.wikipedia.org/wiki/Kalman_filter
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
        self.StateEstimate = self.InitialState;
        self.EstimateCovariance = self.InitialEstimateCovariance;
        
        self.MeasurementCovariance = self.MeasurementNoise .^ 2;
        self.ProcessNoiseCovariance = self.InitialEstimateCovariance;
    end
    
    function [state] = stepImpl(self, measurement)
        % state prediction
        self.StateEstimate = ...
            self.StateTransition * self.StateEstimate ...
            + self.ControlMatrix * self.ControlInput;
        
        % state covariance prediction
        self.EstimateCovariance = ...
            (self.StateTransition * self.EstimateCovariance * self.StateTransition.') ...
            + self.ProcessNoiseCovariance;
        
        % Kalman Gain
        innovationCovariance = self.MeasurementMatrix * self.EstimateCovariance * self.MeasurementMatrix.' ...
            + self.MeasurementCovariance;
        kalmanGain = self.EstimateCovariance * self.MeasurementMatrix.' * pinv(innovationCovariance);
        
        % the residual between measurement and prediction
        residual = measurement - self.MeasurementMatrix * self.StateEstimate;
        
        % state update
        self.StateEstimate = self.StateEstimate + kalmanGain*residual;
        
        % Covariance update
        identyMatrix = eye(size(self.StateEstimate, 1));
        self.EstimateCovariance = ...
            (identyMatrix - kalmanGain*self.MeasurementMatrix) ...
            * self.EstimateCovariance;
        
        state = self.StateEstimate;
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
