classdef MovingAverageFilter < handle
    properties
        windowSize = 4;
        sensorValues = [];
        sum = 0;
    end
    methods
        function obj = MovingAverageFilter (windowSize)
            if nargin == 1
                obj.windowSize = windowSize;
            end
        end
        function result = movingAverageCalculation(obj,value)
            obj.sensorValues = [obj.sensorValues, value];
            obj.sum = obj.sum + value;
            if(length(obj.sensorValues) > obj.windowSize)
                obj.sum = obj.sum - obj.sensorValues(1);
                obj.sensorValues(1) = [];
            end
            result = obj.sum/length(obj.sensorValues);
        end
    end
end