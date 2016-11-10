%Test the time-varying LQR
%
%System dynamics
%swing-up inverted pendulum (nonlinear)
%
m=1; %kg
g=9.81; %m/s^2
l=0.5; %m
I = m*l^2; %kg-m^2

f=sqrt(g/4/l/(pi^2));
dt=0.01/f;
t=[0:dt:10];
Q=cell(length(t),1); A=cell(length(t),1); B=cell(length(t),1);
R=cell(length(t),1); C=cell(length(t),1); D=cell(length(t),1);

for i = 1:length(t)
    %Linearized about the theta = pi point (swung up)
    A{i}=[0 1; -m*g/l 0];
    B{i}=[0 m*l/I]';
    C{i}=[1 0];
    D{i}=[0];
    %Q is identity for now
    %Could also try as C'*C?
    Q{i}=[1 0; 0 1];
    R{i}=1;
end

K=LQR_timevarying(A,B,Q,R,t,dt);

%Solve the linearized problem with no input
%IC for x1, x2
x1(1)=0; %position
x2(1)=0; %velocity
u = 0;
for i=1:length(t)-1
    x1_intermed=x1(i)+dt*x1_dot(A{i},B{i},x1(i),x2(i),u);
    x2_intermed=x2(i)+dt*x2_dot(A{i},B{i},x1(i),x2(i),u);
    x1(i+1)=x1(i)+dt/2*(x1_dot(A{i},B{i},x1(i),x2(i),u)+...
        x1_dot(A{i+1},B{i+1},x1_intermed,x2_intermed,u));
    x2(i+1)=x2(i)+dt/2*(x2_dot(A{i},B{i},x1(i),x2(i),u)+...
        x2_dot(A{i+1},B{i+1},x1_intermed,x2_intermed,u));
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

%Nonlinear feedback control

x1f_nl(1)=.1;
x2f_nl(1)=0;
E0=0;
for i = 1:length(t)-1
    E=0.5*I*(x2f_nl(i))^2+m*g*l*(cos(x1f_nl(i))-1);
    u2=K{i}(2)*(E-E0)*x2f_nl(i)*cos(x1f_nl(i));
    x1f_nl_int=x1f_nl(i)+dt*x2f_nl(i);
    x2f_nl_int=x2f_nl(i)+dt*((m*g*l/I)*sin(x1f_nl(i))+(m*u2*l/I)*cos(x1f_nl(i)));
    x1f_nl(i+1)=x1f_nl(i)+dt/2*(x2f_nl(i)+x2f_nl_int);
    x2f_nl(i+1)=x2f_nl(i)+dt/2*((m*g*l/I)*sin(x1f_nl(i))+(m*u2*l/I)*cos(x1f_nl(i))+...
        (m*g*l/I)*sin(x1f_nl_int)+(m*u2*l/I)*cos(x1f_nl_int));
end

figure(1);
plot(t,x1f_nl,'LineWidth',2);
xlabel('Time (sec)');
ylabel('Angular Displacement (rad)');
%legend('Linearized Response, Zero Input','Linear Response w/ Feedback','Nonlinear Feedback Control');
title('Swing-Up Pendulum with Feedback')
set(gca,'fontsize',16)