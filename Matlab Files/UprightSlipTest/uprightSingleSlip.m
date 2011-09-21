% This file contains: uprightSingleSlip
% uprightSlip takes as parameters:
%   the mass of the slip model
%   the natural length of the leg
%   the spring constant for the leg
%   the initial x velocity
%   the initial y velocity
%   the landing angle the leg will make at touchdown
%   an interval for an initial time, and a maximum final time
%
% This function will simulate one complete step, going through one ground
% stance phase, and one complete aerial phase, ending at the moment of the
% next touchdown. The simulation will not run longer than for the given
% time interval, although it may run for less time.
%
% The model assumes that we have a point mass on a weightless, linear
% spring, making a fixed angle at touchdown. Most initial conditions will
% not produce stable gaits, there is a helper function which can determine
% a good gait.
function [tPlot, statePlots] = uprightSingleSlip(mass, natLegLength, springConst, initXv, initYv, landingAngle, timeInterval)
    % Before anything else, we will define gravity.
    % The value is the standard global average of -9.80665 m/s^2
    
    global GRAVITY;
    GRAVITY = -9.80665;
    
    % First we calculate the initial state of the system. Since the initial
    % velocity is given, we need to find the initial position. We are
    % choosing to have the foot rest at coordinates (0,0), therefore we can
    % calculate the initial x and y coordinates using the landing angle and
    % trigometry.
    
    y0 = natLegLength * sin(landingAngle);
    x0 = -sqrt(natLegLength^2 - y0^2);
     
    % We save the state in a vector. This is the format in general for the
    % state vector of this sytem, it will be used repeatedly.
    
    initialState = [x0, y0, initXv, initYv]
    
    % Next, we create a pair of vectors to store the results of integrating
    % this system. These are the values that will be returned by the
    % function.
    %
    % The two vectors will always have the same length. tPlot
    % will be a vector recording each simulated time step (notice ode45
    % uses adaptive time stepping, so this is not known in advance).
    % statePlots is a vector of state vectors, one for each time step, in
    % the same order. Therefore statePlots(i) is the state of the system at
    % the time tPlot(i).
    
    tPlot = [];
    statePlots = [];
    
    % Here we run the ground phase, and save it in a pair of temporary
    % vectors.
    [tP, sP] = singleStancePhase(mass, natLegLength, springConst, initialState, timeInterval);
    
    % We append the results of the single stance phase simulation into tPlot
    % and statePlots
    tPlot = [tPlot; tP];
    statePlots = [statePlots; sP];
    
    % We next save the final state of the simulated ground phase, to reuse
    % as initial conditions for the aerial phase
    
    finalTime = tP(end);
    finalState = sP(end, :)
    
    % Our simulation promises to run only during the provided time
    % interval. If we used up all the time in the ground phase, we probably
    % do not have a good, periodic running solution anyway. In this case,
    % simply return what we have now, without starting an aerial phase.
    if (finalTime >= timeInterval(2))
        return;
    end
    
    % Assuming now that we have not bailed out due to time overrun, we will
    % now assume that we have completed a ground phase and lifted off into
    % a balistic phase. We will use the saved final state of the ground
    % phase as the initial conditions for the aerial phase.
    
    [tP, sP] = aerialPhase(mass, natLegLength, landingAngle, finalState, [finalTime, timeInterval(2)]);
    
    % Now append the time and state vectors to tPlot and statePlots, before
    % returning
    
    tPlot = [tPlot; tP];
    statePlots = [statePlots; sP];
end