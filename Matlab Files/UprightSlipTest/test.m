function test
    [alpha, iS] = findGoodInitialConditions(70, 1, 10000, 0.8, 3);
    
    times = [];
    states = [];
    
    K = 100;
    
    finalState = [iS(1), iS(2), iS(3), iS(4)]
    %initialState = [iS(1), iS(2), iS(3)+0.3, iS(4)+0.1];
    %initialState = [-1, 1, 2, -1];
    %finalState = [1.4444, 0.9539, 3.3218, -1.6886];
    %finalState = iS;
    
    for i = 1:50
    
        [tP, sP, mF] = uprightSingleSlip(1, 1, 100, finalState(3), finalState(4), alpha, [0, 50]);
        times = [times; tP];
        states = [states; sP];
        finalState = sP(end,:);
        K = K * (1 + 10 * mF); % adjust spring constant on symmetry
        
        finalState = finalState + 0.1 .* randn(1, 4);
    end
    
%     %times = [];
%     %states = [];
%     for i = 1:100
%     
%         [tP, sP] = uprightSingleSlip(1, 1, 100, finalState(3), finalState(4), alpha, [0, 50]);
%         times = [times; tP];
%         states = [states; sP];
%         finalState = sP(end,:);
%     
%     end
    
    fS = finalState
    
    % plot x coordinate vs y coordinate
    plot(states(:,1), states(:, 2))
    %plot(times, states(:,2))
end