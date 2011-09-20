% Takes four parameters:
%   df: derivative function
%   dt: time step
%   T:  total time
%   initialState:  initial state
function [timeVec, stateVec] = ForwardIntegrate(df, dt, T, initialState)
    timeSteps = T/dt;                               
    stateVec = linspace(0, T, timeSteps);    
    timeVec = linspace(0, T, timeSteps);         % discrete time vector
    
    state = initialState;
    
    for i = 1:timeSteps
        state = IntegrateStep(df, dt, state);
        stateVec(i) = state;
    end
end