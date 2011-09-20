% This function takes three parameters:
%   f:  which calculates the derivative of each variable given the state
%       i.e. from a vector U to a vector U' of same dimension
%   dt: time step variable
%   U:  state vector
function V = IntegrateStep(f, dt, U)
    V = U + dt * f(U);
end