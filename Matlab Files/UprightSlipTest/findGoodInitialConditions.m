function [alpha, initialState] = findGoodInitialConditions(mass, natLegLength, springConst, compression, xvCenter)
    [~, states] = singleStancePhase(mass, natLegLength, springConst, [0, (compression * natLegLength), xvCenter, 0], [0, 10]);
    
    initialState = states(end, :);
    alpha = asin(initialState(2));
    
    [~, states] = aerialPhase(mass, natLegLength, alpha, initialState, [0, 10]);
    
    initialState = states(end, :);
    
    % at this point, the y component and velocities are fine, but the x
    % coordinate needs to be reset so that the foot can be placed at (0,0)
    % intellegently
    newX = -sqrt(natLegLength^2 - initialState(2)^2);
    initialState(1) = newX;
end