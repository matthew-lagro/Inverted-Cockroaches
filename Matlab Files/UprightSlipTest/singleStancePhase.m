% This file contains singleStancePhase
% 
% This function integrates the slip model from the beginning to the end of
% a single leg stance phase. The initialState is given, i.e.
%   [x0, y0, xv0, yv0]
% so the model does not need to start at the beginning of a stance phase.
% However, it will continue until either time runs out, or we lift off the
% ground, i.e. when the spring force goes to zero.
function [timeVec, stateVec] = singleStancePhase(mass, natLegLength, springConst, initialState, timeInterval)
    global GRAVITY; % we use the same value for gravity as in uprightSingleSlip
    
    % First, we set up the options for the ode solver. We will be
    % triggering an event, see below for the defintion of this function.
    options = odeset('Events', stanceEvent);
    
    % Here is the bulk of the work. ode45 does an adaptive Runge-Kutta
    % integration of our system of odes, with the special option of
    % terminating when our spring force vector drops to zero.
    [timeVec, stateVec] = ode45(deriv, timeInterval, initialState, options);
    
    % Now, let us begin defining the arguments to ode45.
    %
    % First, we create a function that given a time and a state vector,
    % will return the derivatives of each component of the state vector.
    % Remember, the state vectors are of the form [x, y, xv, yv] where xv
    % and yv are the x and y components of the velocity. The time is
    % unused, so in the signature, replaced with ~
    function d = deriv(~, state)
        % For readability, we split the state vector into its (named) parts
        
        x = state(1);
        y = state(2);
        xv = state(3);
        yv = state(4);
        
        % The accelerations are given by the spring force, coaxial with the
        % spring. The magnitude of the force is given by Hooke's law, the
        % direction by decomposing the vector of the leg.
        %
        % legLength is the current length of the leg
        
        legLength = sqrt(x^2+y^2);
        
        % legExtension is the quantity we will use to calculate Hooke's
        % law. Notice it is negative when the leg is compressed.
        
        legExtension = natLegLength - legLength;
        
        % Hooke's law: the magnitude of the force from a spring is
        % proportional to the extension, in the opposite direction.
        
        F = - (springConst * legExtension);
        
        % Now we can calculate xa and ya, the accelerations. Notice that
        % the x component of the unit direction vector is x/legLength, etc.
        % We are using F=ma, or rather, a = F/m.
        
        xa = F * (x/legLength) * (1/mass);
        ya = F * (y/legLength) * (1/mass);
        
        % Finally, we set up the derivative
        d = [xv, yv, xa, ya];
    end

    % Here we define the event function, stanceEvent. The event is
    % triggered when value, as returned by stanceEvent, goes to zero, from
    % above.
    function [value, isterm, direction] = stanceEvent(~, state)
        
        % Value is just our (signed) spring force. We calculate it as above
        % in the derivative function.
        
        legLength = sqrt(x^2+y^2);
        legExtension = natLegLength - legLength;    
        value = - (springConst * legExtension);
        
        isterm = 1; % halt integration when event occurs
        direction = -1; % we only care about when the spring force has decreased to zero, i.e. was positive
    end
end