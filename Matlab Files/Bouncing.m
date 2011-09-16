function Bouncing(T,dt,method)
t1=linspace(0,T,150);%Time plot vector
N=T/dt; %Total time steps
t=0:dt:T ; %Discrete time vector
k=1;%Spring constant
m=1;%Mass
g=1;%Acceleration due to gravity
%w=sqrt(k/m);%Natural Frequency

y0=1;dy0=0;%Initial Conditions
U=[y0;dy0];

A=[0 1; -k/m 0]; b=[0;-g] ;
B=[0 1;0 0];
Y=zeros(2,length(t)); Y(:,1)=U;

switch method

%FE    
case 1
for i= 1:N
    U= U + dt*f(U);
    Y(:,i+1)=U;
    clf
    plot(i*dt,U(1),'r-o',t1,true(t1),'b')
    drawnow    
end

%RK4
case 2
for i=1:N
if -k*U(1) <= 0 
s1= f(U);
s2= f(U + (dt/2)*s1);
s3= f(U + (dt/2)*s2);
s4= f(U + dt*s3);
U = U + dt*(s1 + 2*s2 + 2*s3 + s4)/6;
elseif -k*U(1) >0
p1= G(U);
p2= G(U + (dt/2)*p1);
p3= G(U + (dt/2)*p2);
p4= G(U + dt*p3);
U = U + dt*(p1 + 2*p2 + 2*p3 + p4)/6;
end
Y(:,i+1)=U;

% clf
% subplot(3,1,1), plot(i*dt,U(1),'r-o')
% title('Vertical Displacement vs Time')
% xlabel('Time')
% ylabel('Vertical Displacement')
% subplot(3,1,2), plot(0,U(1),'g-o')
% title('Center of Mass Animation')
% subplot(3,1,3), plot(i*dt,-k*U(1),'r-o')
% title('Force vs Time')
% xlabel('Time')
% ylabel('Force')
% drawnow 
end
end
plot(t,Y(1,:),'r-o')
title('Vertical Displacement vs Time')
xlabel('Time')
ylabel('Vertical Displacement')

function z=f(u)
z= A*u +b;
end

function z=G(u)
z=B*u + b;
end

end
