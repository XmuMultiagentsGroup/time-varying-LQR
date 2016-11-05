%Test the time-varying LQR with a simple system
%
%System dynamics
%Simple pendulum (nonlinear)
%
m=1; %kg
g=9.81; %m/s^2
l=0.5; %m
%constant u for now (zero)
u=0;
g=9.8;
L=0.2;
%damping
c=0.5; %kg*s/m
f=sqrt(g/4/L/(pi^2)); %This is not actually the frequency anymore since I added damping
dt=0.01/f;
t=[0:dt:10];
Q=cell(length(t),1); A=cell(length(t),1); B=cell(length(t),1);
R=cell(length(t),1); C=cell(length(t),1); D=cell(length(t),1);
%
for i = 1:length(t)
    A{i}=[0 1; -g/l -c/m];
    B{i}=[0 1/m]';
    C{i}=[1 0];
    D{i}=[0];
    %Q is identity for now
    %Could also try as C'*C?
    Q{i}=[1 0; 0 1];
    %Note that K values get GIANT if R is small. This isn't good because
    %I think Patrick is using R=0.01 right now.
    R{i}=1;
end

K=LQR_timevarying(A,B,Q,R,t,dt);

%Now to see if K(t) is really working

%Solve the linearized problem
%IC for x1, x2
x1(1)=pi/4; %position
x2(1)=0; %velocity
for i=1:length(t)-1
    x1_intermed=x1(i)+dt*x1_dot(A{i},B{i},x1(i),x2(i),u);
    x2_intermed=x2(i)+dt*x2_dot(A{i},B{i},x1(i),x2(i),u);
    x1(i+1)=x1(i)+dt/2*(x1_dot(A{i},B{i},x1(i),x2(i),u)+...
        x1_dot(A{i+1},B{i+1},x1_intermed,x2_intermed,u));
    x2(i+1)=x2(i)+dt/2*(x2_dot(A{i},B{i},x1(i),x2(i),u)+...
        x2_dot(A{i+1},B{i+1},x1_intermed,x2_intermed,u));
end

%Solve nonlinear problem
x1_nl(1)=pi/4;
x2_nl(1)=0;
for i=1:length(t)-1
    x1_nl_int=x1_nl(i)+dt*x2_nl(i);
    x2_nl_int=x2_nl(i)+dt*((-g/l)*sin(x1_nl(i))+(-c/m)*x2_nl(i)+1/m*u);
    x1_nl(i+1)=x1_nl(i)+dt/2*(x2_nl(i)+x2_nl_int);
    x2_nl(i+1)=x2_nl(i)+dt/2*((-g/l)*sin(x1_nl(i))+(-c/m)*x2_nl(i)+1/m*u+...
        (-g/l)*sin(x1_nl_int)+(-c/m)*x2_nl_int+1/m*u);
end

%Now try the same thing but with (A-B*K)x

x1c(1)=pi/4;
x2c(1)=0;

for i=1:length(t)-1
    x1c_intermed=x1c(i)+dt*x1_feedback(A{i},B{i},K{i},x1c(i),x2c(i));
    x2c_intermed=x2c(i)+dt*x2_feedback(A{i},B{i},K{i},x1c(i),x2c(i));
    x1c(i+1)=x1c(i)+dt/2*(x1_feedback(A{i},B{i},K{i},x1c(i),x2c(i))+...
        x1_feedback(A{i+1},B{i+1},K{i+1},x1c_intermed,x2c_intermed));
    x2c(i+1)=x2c(i)+dt/2*(x2_feedback(A{i},B{i},K{i},x1c(i),x2c(i))+...
        x2_feedback(A{i+1},B{i+1},K{i+1},x1c_intermed,x2c_intermed));
end

%feedback control for nonlinear response

x1f_nl(1)=pi/4;
x2f_nl(1)=0;
for i = 1:length(t)-1
    u1=K{i}(1);
    u2=K{i}(2);
        x1f_nl_int=x1f_nl(i)+dt*x2f_nl(i);
    x2f_nl_int=x2f_nl(i)+dt*((-g/l)*sin(x1f_nl(i))+(-c/m)*x2f_nl(i)-1/m*u1*x1f_nl(i)-1/m*u2*x2f_nl(i));
    x1f_nl(i+1)=x1f_nl(i)+dt/2*(x2f_nl(i)+x2f_nl_int);
    x2f_nl(i+1)=x2f_nl(i)+dt/2*((-g/l)*sin(x1f_nl(i))+(-c/m)*x2f_nl(i)-1/m*u1*x1f_nl(i)-1/m*u2*x2f_nl(i)+...
        (-g/l)*sin(x1f_nl_int)+(-c/m)*x2f_nl_int-1/m*u1*x1f_nl(i)-1/m*u2*x2f_nl(i));
end

figure;
plot(t,x1,t,x1_nl,t,x1c,t,x1f_nl,'LineWidth',2);
xlabel('Time (sec)');
ylabel('Angular Displacement (rad)');
legend('Linearized Response','Nonlinear Response','Linear Response w/ Feedback','Nonlinear Response w/ Feedback');
title('Damped Pendulum')
set(gca,'fontsize',16)