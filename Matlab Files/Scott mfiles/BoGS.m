function [v0,vF,normV]=BoGS(xV0,yV0,alpha)

%alpha=pi/3-.1; %Fixed alpha

%Necessary Parameters
m=1;            %Mass
k=200;          %Spring Constant
l=1;            %Natural leg length
g=9.8;          %Acceleration due to gravity
w=sqrt(k/m);    %Natural Frequency

x0=-l*cos(alpha);
y0=l*sin(alpha);
v0=[xV0;yV0];
u0=[x0;y0;xV0;yV0];

airOptions=odeset('Events', @airEvents,'Refine',100);
stanceOptions = odeset('Events',@stanceEvents,'Refine',100);

% tPlot=[];
% sPlot=[];
%tspan1=0:.001:100;
[t,S]=ode45(@groundPhase,[0 100],u0,stanceOptions);

Tf= t(end);
Sf= S(end,:);
landingHeight= y0;

% tPlot=[tPlot;t];
% sPlot=[sPlot;S];

[t,S]=ode45(@aerialPhase,[Tf Tf+100],Sf, airOptions);

%Save results from aerial phase
% Tf=t(end);
Sf=S(end,:);
vF=[Sf(3);Sf(4)];

% %Reset x
% newX= -sqrt(l - Sf(2)^2);
% Sf(1)=newX;
% 
% tPlot=[tPlot;t];
% sPlot=[sPlot;S];
% 
% %Second ground phase
% [t,S]=ode45(@groundPhase,[Tf,Tf+100],Sf,stanceOptions);
% 
% tPlot=[tPlot;t];
% sPlot=[sPlot;S];
normV=norm(vF-v0);

% plot(sPlot(:,1),sPlot(:,2))
% xlabel('Horizontal Displacement')
% ylabel('Vertical Displacement')


    %Ground Phase eqns
    function U=groundPhase(~,u)
    LL= (u(1)^2 + u(2)^2)^(1/2);
    Fx= u(1)*(l/LL - 1)*w^2;
    Fy= u(2)*(l/LL - 1)*w^2 - g;
    U=[u(3);u(4);Fx;Fy];
    end
    
    %Aerial Phase eqns
    function U=aerialPhase(~,u)
    U=[u(3);u(4);0;-g];    
    end


    function [F, isterminal, direction] = stanceEvents(~,state)
    F= springForce(state(1),state(2));
    isterminal=1;
    direction=-1;
    end

    function [ydisp,isterminal,direction]= airEvents(~,state)
    ydisp=state(2) - landingHeight ;       
    isterminal=1;
    direction=-1; %only want to stop if we are _decending_ to landing
    end

    function L = disp(x,y)
    L= (x^2 + y^2)^(1/2) - l;
    end

    function F= springForce(x,y)
    F=-k*disp(x,y);
    end
end