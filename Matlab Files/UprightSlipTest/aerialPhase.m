function [times, states] = aerialPhase(mass, natLegLength, landingAngle, initialState, timeInterval)
    global GRAVITY;
    
    options = odeset('Events', @airEvent);
    
    yFinal = natLegLength * sin(landingAngle);
    
    [times, states] = ode45(@deriv, timeInterval, initialState, options);
    
    function d = deriv(~, state)
        x = state(1);
        y = state(2);
        xv = state(3);
        yv = state(4);
        
        d = [xv; yv; 0; (mass * GRAVITY)];
    end

    function [value, isterm, direction] = airEvent(~, state)
        y = state(2);
        
        value = y - yFinal;
        
        isterm = 1;
        direction = -1; % only halt if height is decreasing to 0
    end
end