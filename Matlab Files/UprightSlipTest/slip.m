function slip(initXv, initYv, landingAngle)
    x0 = cos(landingAngle);
    y0 = sin(landingAngle);
    m = 1;
    g = -1;
    k = 10;
    
    initialState = [x0, y0, initXv, initYv];
    tPlot = [];
    statePlots = [];
    
    
    airOptions = odeset('Events', @airEvents, 'OutputSel', [2]);
    stanceOptions = odeset('Events', @stanceEvents, 'OutputSel', [2]);
    
    [tP, sP] = ode45(@stanceDeriv, [0,50], [0, 0.5, 1, 0], stanceOptions);
    
    % Save results from first ground phase
    finalTime = tP(end);
    finalState = sP(end, :);

    % Save the plotted values from first ground phase
    tPlot = [tPlot; tP];
    statePlots = [statePlots; sP];
    
    % set initial conditions for next part to match final conditions of
    % this part
    landingHeight = finalState(2);
    
    
    [tP, sP] = ode45(@airDeriv, [finalTime,50], finalState, airOptions);
    
    % save plotted values from air phase
    tPlot = [tPlot; tP];
    statePlots = [statePlots; sP];
    
    % save state from air phase
    finalTime = tP(end);
    finalState = sP(end, :)
    
    % reset x
    newX = -sqrt(1-(finalState(2))^2);
    finalState(1) = newX;
    % second ground phase
    [tP, sP] = ode45(@stanceDeriv, [finalTime,50], finalState, stanceOptions);
    
    % save plotted values from second ground phase
    tPlot = [tPlot; tP];
    statePlots = [statePlots; sP];
    

    plot(tPlot, statePlots(:, 2))
    %plot(statePlots(:,1), statePlots(:,2))
    
    
    
    function derivs = stanceDeriv(~, state)
        x = state(1);
        y = state(2);
        F = springForce(x, y);
        
        xv = state(3);
        yv = state(4);
        xa = F * x;
        ya = F * y;
        
        %[x, y, xv, yv, xa, ya]
        
        derivs = [xv; yv; xa; ya];
    end

    function [F, isterminal, direction] = stanceEvents(~, state)
        F = springForce(state(1), state(2));
        
        isterminal = 1;
        direction = 0;
    end
    
    function derivs = airDeriv(~, state)
        xv = state(3);
        yv = state(4);
        xa = 0;
        ya = m*g;
    
        derivs = [xv; yv; xa; ya];
    end

    function [ydisp, isterminal, direction] = airEvents(~, state)
        ydisp = state(2) - landingHeight;
        
        direction = 0;
        isterminal = 1;
    end

    function disp = displacement(x, y)
        disp = x^2 + y^2 - 1;
    end

    function F = springForce(x, y)
       F = -k * displacement(x,y); 
    end
end