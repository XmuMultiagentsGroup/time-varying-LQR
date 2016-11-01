%Test the time-varying LQR with a simple system
%
%System dynamics
%Pick a linear system
%mass-spring-damper example given in UM controls tutorial
%http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=SystemModeling
%
m = 1; %kg
k = 1; %N/m
b = 0.2; %Ns/m (damping constant)
F = 1; %N (input force)
t=0:.1:10;
Q=cell(length(t),1); A=cell(length(t),1); B=cell(length(t),1);
R=cell(length(t),1); C=cell(length(t),1); D=cell(length(t),1);
%
for i = 1:length(t)
    A{i}=[0 1; -k/m -b/m];
    B{i}=[0 1/m]';
    C{i}=[1 0];
    D{i}=[0];
    %Q is identity for now
    Q{i}=[1 0; 0 1];
    %Note that K values get GIANT if R is small. This isn't good because
    %I think Patrick is using R=0.01 right now
    R{i}=0.1;
end

K=LQR_timevarying(A,B,Q,R,t);

%Now to see if K(t) is really working
%Say we have a nonlinear system that we linearize and put in state-space
%form. Then x_tilde is the coordinates (x, x_dot) of the linearized system
%and x_ol is the response of the system with no feedback control
%optimal solution

%Solve the linearized problem
%IC for x1, x2
x1(1)=0.2; %position
x2(1)=0; %velocity
dt=t(2)-t(1);
for i=1:length(t)-1
    x1_intermed=x1(i)+dt/2*x1_dot(A{i},B{i},x1(i),x2(i),F);
    x2_intermed=x2(i)+dt/2*x2_dot(A{i},B{i},x1(i),x2(i),F);
    x1(i+1)=x1(i)+dt/2*(x1_dot(A{i},B{i},x1(i),x2(i),F)+...
        x1_dot(A{i+1},B{i+1},x1_intermed,x2_intermed,F));
    x2(i+1)=x2(i)+dt/2*(x2_dot(A{i},B{i},x1(i),x2(i),F)+...
        x2_dot(A{i+1},B{i+1},x1_intermed,x2_intermed,F));
end

%Now try the same thing but with (A-B*K)x+BKrr? What is Krr? (I'm using F(t) for now)
x1c(1)=0.2;
x2c(1)=0;
for i=1:length(t)-1
    x1c_intermed=x1c(i)+dt/2*x1_feedback(A{i},B{i},K{i},x1c(i),x2c(i),F);
    x2c_intermed=x2c(i)+dt/2*x2_feedback(A{i},B{i},K{i},x1c(i),x2c(i),F);
    x1c(i+1)=x1c(i)+dt/2*(x1_feedback(A{i},B{i},K{i},x1c(i),x2c(i),F)+...
        x1_feedback(A{i+1},B{i+1},K{i+1},x1c_intermed,x2c_intermed,F));
    x2c(i+1)=x2c(i)+dt/2*(x2_feedback(A{i},B{i},K{i},x1c(i),x2c(i),F)+...
        x2_feedback(A{i+1},B{i+1},K{i+1},x1c_intermed,x2c_intermed,F));
end

figure;
plot(t,x1,t,x1c);
xlabel('Time (sec)');
ylabel('Displacement (m)');
legend('No Feedback Control','Feedback Control');