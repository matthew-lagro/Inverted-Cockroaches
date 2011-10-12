function MidStance
%This is code of the SLIP model of running. The code takes an angle of
%attack as the only parameter. We set the initial position at midstance
%when velocity in the y direction is zero. The ground phase is then run
%till aerial phase. A full aerial phase is completed followed by a complete
%stance phase.

k=200; %Spring constant
m=1; %Mass
g=9.8; %Acceleration due to gravity
w=sqrt(k/m);%Natural Frequency
l=1; %Natural Leg Length

tPlot=[];
sPlot=[];

airOptions=odeset('Events', @airEvents,'Refine',50);
stanceOptions = odeset('Events',@stanceEvents,'Refine',50);

%First ground phase started from midstance
%[t,S]=ode45(@groundPhase,[0 50],[0,0.5,1,0],stanceOptions);
[t,S]=ode45(@groundPhase,[0 50],[0,.5,3,0],stanceOptions);

%Save results from half ground phase
Tf= t(end);
Sf= S(end,:)

landingHeight= Sf(2)

tPlot=[tPlot;t];
sPlot=[sPlot;S];

%Complete aerial phase
[t,S]=ode45(@airPhase, [Tf, 50],Sf, airOptions);

%Save results from aerial phase
Tf=t(end);
Sf=S(end,:)


%Reset x
newX= -sqrt(l - Sf(2)^2);
Sf(1)=newX;

tPlot=[tPlot;t];
sPlot=[sPlot;S];

%Second ground phase
[t,S]=ode45(@groundPhase,[Tf,Tf+100],Sf,stanceOptions);

S(end,:)

tPlot=[tPlot;t];
sPlot=[sPlot;S];



    function U=groundPhase(~,state)
    F= springForce(state(1),state(2));
    LL= (state(1)^2 + state(2)^2)^(1/2);
    Fx= F*(state(1)/LL)*w^2;
    Fy= F*(state(2)/LL)*w^2 - g;
    U=[state(3);state(4);Fx;Fy];
    end

    function U=airPhase(~,state)
    U=[state(3);state(4);0;-g];
    end

% subplot(3,1,1), 
plot(sPlot(:,1),sPlot(:,2),'b',sPlot(end,1),sPlot(end,2),'r-o')
xlabel('Hortizontal Displacement')
ylabel('Vertical Displacement')
% subplot(3,1,2), plot(tPlot,sPlot(:,1),'r')
% subplot(3,1,3), plot(tPlot,sPlot(:,2),'g')
        
   

    function [F, isterminal, direction] = stanceEvents(~,state)
    F= springForce(state(1),state(2));
    isterminal=1;
    direction=-1;
    end

    function [ydisp,isterminal,direction]= airEvents(~,state)
    ydisp=state(2) - landingHeight ;       
    isterminal=1;
    direction=-1;
    end
    

    function L = disp(x,y)
    L= (x^2 + y^2)^(1/2) - l;
    end

    function F= springForce(x,y)
    F=-k*disp(x,y);
    end

end