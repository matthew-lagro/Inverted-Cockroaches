function Spring(T,dt,method)
t1=linspace(0,T,1000);%Time plot vector
N=T/dt; %Total time steps
t=0:dt:T ; %Discrete time vector
k=1;%Spring constant
m=1;%Mass
g=1;%Acceleration due to gravity
w=sqrt(k/m);%Natural Frequency

y0=0;dy0=1;%Initial Conditions
U=[y0;dy0];

A=[0 1; -k/m 0]; b=[0;g] ;
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
s1= f(U);
s2= f(U + (dt/2)*s1);
s3= f(U + (dt/2)*s2);
s4= f(U + dt*s3);
U = U + dt*(s1 + 2*s2 + 2*s3 + s4)/6;
clf
plot(i*dt,U(1),'r-o',t1,true(t1),'b')
drawnow 
end

end

function z=f(u)
z= A*u +b;
end

function y= true(t)
y= dy0*sin(w*t)/w + (-g)*cos(w*t)/w^2 + g/w^2;
end
end