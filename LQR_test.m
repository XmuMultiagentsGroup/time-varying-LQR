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

%Now plot some things
%open loop (Need to plot this response for time-varying matrices. Like, 
%define a tiny time step and apply the dynamics at each time because the
%matrices could be time-varying
%Currently it's just plotting the same time response for each 
%matrix in the time step on top of itself)
u = zeros(size(t));
x0 = [0.2 0];
figure;
for i = 1:length(t)-1
    sys = ss(A{i},B{i},C{i},D{i});
    [y, t_int, x] = lsim(sys,u,t,x0);
    x0(1)=x0(1)+(y(2)-x0(1));
    plot(t_int,y, 'b');
    hold on;
    title('Open-Loop Response to Non-Zero Initial Condition')
    xlabel('Time (sec)')
    ylabel('Position (m)')
end

%closed loop (I am not doing this right)
figure;
for i = 1:length(t)-1
    sys_cl = ss(A{i}-B{i}*K{i},B{i},C{i},D{i});
    [y2, t_int2, x2] = lsim(sys_cl,u,t,x0);
    plot(t_int2, y2);
    hold on;
    title('Linear Simulation Results')
    xlabel('Time (sec)')
    ylabel('Position (m)')
end