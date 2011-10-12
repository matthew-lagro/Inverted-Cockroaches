function GroundDS
%Code is the Upright SLIP model with double stance.
%Simulation starts with both legs on the gournd. Transitions from double
%stance phase to single stance phase.
%
%Input Parameters: 
%(a)Mass
%(b)Spring constant of legs
%(c)Foot position of leg 2
%(d)Initial Velocity
%(e)Initial angle of attack alpha
%%%%%%%%%%%%%%%%%%%%%%%%%%

alpha= pi/3-.1;
xv0=5.3;
yv0=3;


m=1;   %Mass of critter
lg1=3; %Natural length of leg 1
lg2=1; %Natural length of leg 2
k=1;  %Spring constant of critter's leg
xc2=1; %Foot position of leg 2
g=9.8; %Gravitational acceleration

x0=xc2-lg1*cos(alpha);
y0=lg2*sin(alpha);


u0=[x0,y0,xv0,yv0];

% SSoption=odeset('Events',@Sevents);
DSoptions=odeset('Events',@DSevents);


[t,Y]=ode45(@DS,[0 1],u0,DSoptions);

for i=1:length(t)
clf
plot(Y(i,1),Y(i,2),'r-o',Y(:,1),Y(:,2))
drawnow
end

    %Single Stance Function
    function U=S(~,u)
    z1=F2(u(1),u(2))*(u(1)-xc2)/L2(u(1),u(2))/m;
    z2=F2(u(1),u(2))*u(2)/L2(u(1),u(2))/m -g ;
    U=[u(3);u(4);z1;z2];
    end
    
    %Double Stance Function
    function U=DS(~,u)
    z1= (F1(u(1),u(2))*u(1)/L1(u(1),u(2)) +...
        F2(u(1),u(2))*(u(1)-xc2)/L2(u(1),u(2)))/m;
    z2=(F1(u(1),u(2))*u(2)/L2(u(1),u(2)) +...
        F2(u(1),u(2))*u(2)/L2(u(1),u(2)))/m -g ;
    U=[u(3);u(4);z1;z2];  
    end

    %Leg 1 length function
    function L=L1(x,y)
    L=(x^2 + y^2)^(1/2);
    end
    
    %Leg 2 length function
    function L=L2(x,y)
    L=((x-xc2)^2 + y^2)^(1/2);
    end

    %Force function for leg 1
    function F=F1(x,y) 
    F=-k*(( x^2 + y^2)^(1/2) -lg1 );  
    end

    %Force function for leg 2
    function G=F2(x,y)
    G=-k*( ((x-xc2)^2 + y^2)^(1/2) - lg2 );    
    end

    function [LL,isterminal,direction]=DSevents(~,u)
    LL= (lg1- L1(u(1),u(2)));
    isterminal=1;
    direction=-1;
    end
% 
%     function [ydisp,isterminal,direction]=Sevents(~,u)
%     ydisp=    
    


end