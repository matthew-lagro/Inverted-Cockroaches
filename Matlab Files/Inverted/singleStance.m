function [tf, sf, Ts, Sts, TEs, SEs, IEs] = singleStance(fixedParameters, initialState, timeInterval)
    % singleStance takes as parameters:
    %   fixedParameters  [mass, neutralLength, springConst]
    %   initialState     [r, theta, v, alpha]
    %   timeInterval     [initialTime, maximumFinalTime]
    %
    % returns:
    %   tf   final time
    %   sf   final state
    %   Ts   column vector of times
    %   Sts  column vector of states
    %   TEs  vector of times of events
    %   SEs  vector of state (vectors) at events
    %   IEs  vector of indicies of corresponding events
    %
    % Representing the evolution of a single stance phase forward in time.
    
    m = fixedParameters(1);
    l0 = fixedParameters(2);
    k = fixedParameters(3);
    
    % The work happens here
    options = odeset('Events', @events);
    [Ts, Sts, TEs, SEs, IEs] = ode45(@deriv, timeInterval, initialState, options);
    tf = Ts(end);
    sf = Sts(:, end);
    
    plot(Sts(:, 1) .* cos(Sts(:, 2)), Sts(:, 1) .* sin(Sts(:, 2)))
    
    function D = deriv(~, state)
        r = state(1);
        theta = state(2);
        v = state(3);
        alpha = state(4);
        
        Dr = v * cos(alpha - theta);
        Dtheta = v/r * sin(alpha - theta);
        
        F = springForce(r, theta);
        Dv = F/m * cos(theta - alpha)/v;
        Dalpha = F/m  * sin(theta - alpha);
        %Dv = F/m * cos(alpha - theta) + r * Dtheta * Dtheta;
        %Dalph
        
        D = [Dr; Dtheta; Dv; Dalpha];
    end
    
    function F = springForce(r, ~)
        F = k * (r - l0);
    end

    function [vals, terms, dirs] = events(~, states)
        r = states(1);
        legVal = r - l0;
        legTerm = 1;
        legDir = +1; %+1
        
        vals = [legVal];
        terms = [legTerm];
        dirs = [legDir];
    end
end