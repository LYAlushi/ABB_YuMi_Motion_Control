%%  PID function for ABB IRB14000 (YuMi) joint regulation
% Author      : L.Y.Alushi
% Date        : 08/2024
% Title       : PID computation for joint angle regulation ABB YuMi
% Research    : Motion control with collision avoidance for kinematically redundant manipulator
% Institution : Coventry University
% Supervisor  : K.Al Khudir
%% Begin Function----------------------------------------------------------
function controlOutput = pidControl(currentPositions, desiredPositions, Kp, Ki, Kd, dt)
    % Persistent variables to store the integral and derivative error terms
    persistent integralError previousError;
    if isempty(integralError)
        integralError = zeros(1, 14);
    end
    if isempty(previousError)
        previousError = zeros(1, 14);
    end
    % Compute the error terms
    positionError = desiredPositions - currentPositions;
    integralError = integralError + positionError * dt;
    derivativeError = (positionError - previousError) / dt;
    % Compute the PID control output
    controlOutput = Kp * positionError + Ki * integralError + Kd * derivativeError;
    % Update the previous error
    previousError = positionError;
end
%% End Function----------------------------------------------------------
