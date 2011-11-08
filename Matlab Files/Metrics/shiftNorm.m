function shiftNorm
    A = importdata('FootfallData2.csv');
    A = A.data;
    
  
    i = 1;
    padding = 50;
    timeLength = 250;
    trials = 20;
    % V is indexed as V(foot, time, trial)
    V = zeros(6,timeLength+2*padding);
    V(:,:,20) = zeros(6,timeLength+2*padding);

    for trial = 1:trials
        while i<=length(A) && A(i, 2) == trial            
            for foot=1:6
                if isnan(A(i,4+2*foot)) || isnan(A(i,4+2*foot+1))
                    break;
                end
                for time=A(i, 4+2*foot):A(i, 4+2*foot+1)
                   V(foot,time+padding,trial) = 1;
                end
            end
            i = i+1;
        end
    end
    
    Dist = zeros(20, 6);
    
    for trial=1:trials
       % 1,2
       a=1;
       b=2;
       
       W = V(a,:,trial) - V(b,:,trial);
       Dist(trial, 1) = sqrt(W*W');
       
       % 1,3
       b=3;
       W = V(a,:,trial) - V(b,:,trial);
       Dist(trial, 2) = sqrt(W*W');
    
       % 2,3
       a=2;
       W = V(a,:,trial) - V(b,:,trial);
       Dist(trial, 3) = sqrt(W*W');
       
       % 4,5
       a=4;
       b=5;
       
       W = V(a,:,trial) - V(b,:,trial);
       Dist(trial, 4) = sqrt(W*W');
       
       % 4,6
       b=6;
       W = V(a,:,trial) - V(b,:,trial);
       Dist(trial, 5) = sqrt(W*W');
    
       % 5,6
       a=5;
       W = V(a,:,trial) - V(b,:,trial);
       Dist(trial, 6) = sqrt(W*W');
    end
    
    Dist
    
    function dist = shiftDist(U, V)
        bestShift = 0;
        W = U - V;
        bestDist = sqrt(W*W');
        
        for shift=(-padding):padding
            
        end
    end
end