function test
    [alpha, initialState] = findGoodInitialConditions(70, 1, 10000, 0.8, 3);
    
    [times, states] = uprightSingleSlip(1, 1, 100, initialState(3), initialState(4), alpha, [0, 50]);
    
    % plot x coordinate vs y coordinate
    plot(states(:,1), states(:, 2))
    %plot(times, states(:,2))
end