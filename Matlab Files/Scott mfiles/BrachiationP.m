function BrachiationP

L=1;
g=9.8;
theta0=-pi/4; dtheta0=3;
p0=[theta0;dtheta0];

Poptions=odeset('Events',@Pevents,'Refine',50);
Aoptions=odeset('Events',@Aevents,'Refine',50);

[t,Y]=ode45(@P,[0 10],p0,Poptions); 
tf=t(end); Yend=Y(end,:); X2=L*sin(Yend(1));

xPlot1=L*sin(Y(:,1));
yPlot1=-L*cos(Y(:,1));

x0=L*sin(Yend(1)); y0=-L*cos(Yend(1));
xv0=L*Yend(2)*cos(Yend(1)); yv0=L*Yend(2)*sin(Yend(1));
a0=[x0;y0;xv0;yv0];

[t,Y]=ode45(@B,[tf tf+10],a0,Aoptions);
tf=t(end); Yend=Y(end,:); X3=Yend(1);

xPlot2=Y(:,1);
yPlot2=Y(:,2);

theta0=-acos(-Yend(2)/L);
dtheta0=(Yend(3)*cos(theta0) + Yend(4)*sin(theta0))/L;
p0=[theta0;dtheta0];

[~,Y]=ode45(@P,[tf tf+10],p0,Poptions); 

xPlot3=L*sin(Y(:,1))+X2+X3;
yPlot3=-L*cos(Y(:,1));

BPlotx=[xPlot1;xPlot2;xPlot3];
BPloty=[yPlot1;yPlot2;yPlot3];
N=length([xPlot1;xPlot2;xPlot3]);

for i=1:N
clf
plot(xPlot1,yPlot1,'r',xPlot2,yPlot2,'b',xPlot3,yPlot3,'r',BPlotx(i),BPloty(i),'k-o')
axis([-1 4 -1.1 0])
drawnow
end

% for i=1:length(t)
% clf
% plot(L*sin(Y(i,1)), -L*cos(Y(i,1)), 'r-o')
% drawnow
% end

function Z=P(~,u)
u1= u(2);
u2= (-g/L)*sin(u(1));
Z=[u1;u2];
end

function Z=B(~,u)
Z=[u(3);u(4);0;-g];
end

function [T,isterminal,direction]=Pevents(~,u)
T= theta0 + u(1);
isterminal=1;
direction=0;
end

function [H,isterminal,direction]=Aevents(~,u)
H=u(2)-y0;
isterminal=1;
direction=0;
end

end