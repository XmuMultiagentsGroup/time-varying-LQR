%Test out LQR function on a nonlinear time-varying system
%unicycle model
%Px, Py are spacial coordinates
%theta is heading
%v, w are inputs (forward velocity and steering speed, respectively)
%
t=0:.01:20;
dt=t(2)-t(1);
Q=cell(length(t),1); A=cell(length(t),1); B=cell(length(t),1);
R=cell(length(t),1); C=cell(length(t),1); D=cell(length(t),1);
%After linearizing the system about a solution trajectory 
%x_sol=[sin(t); 1-cos(t); t] and u_sol=[w_sol v_sol]
%We obtain the following time-varying matrices (A,B are TV, C,D are not)
for i = 1:length(t)
    A{i}=[0 0 -sin(t(i)); 0 0 cos(t(i)); 0 0 0];
    B{i}=[cos(t(i)) 0; sin(t(i)) 0; 0 1];
    C{i}=eye(3,3);
    D{i}=zeros(3,2);
    %Q is identity for now
    %Could also try as C'*C?
    Q{i}=[1 0 0; 0 1 0; 0 0 1];
    %Note that K values get GIANT if R is small. This isn't good because
    %I think Patrick is using R=0.01 right now.
    R{i}=10;
end

K=LQR_timevarying(A,B,Q,R,t,dt);

%Solve the linearized problem
%IC for x1, x2
Px(1)=5; %spacial x
Py(1)=5; %spacial y
theta(1)=1;
v=1; %make these change w/time!!!
w=1;
for i=1:length(t)-1
    Px_int=Px(i)+dt*Px_dot(A{i},B{i},Px(i),Py(i),theta(i),v,w);
    Py_int=Py(i)+dt*Py_dot(A{i},B{i},Px(i),Py(i),theta(i),v,w);
    theta_int=theta(i)+theta_dot(A{i},B{i},Px(i),Py(i),theta(i),v,w);
    Px(i+1)=Px(i)+dt/2*(Px_dot(A{i},B{i},Px(i),Py(i),theta(i),v,w)+...
        Px_dot(A{i+1},B{i+1},Px_int,Py_int,theta_int,v,w));
    Py(i+1)=Py(i)+dt/2*(Py_dot(A{i},B{i},Px(i),Py(i),theta(i),v,w)+...
        Py_dot(A{i+1},B{i+1},Px_int,Py_int,theta_int,v,w));
    theta(i+1)=theta(i)+dt/2*(theta_dot(A{i},B{i},Px(i),Py(i),theta(i),v,w)+...
        theta_dot(A{i+1},B{i+1},Px_int,Py_int,theta_int,v,w));
end

%Now try the same thing but with (A-B*K)x

Pxc(1)=5;
Pyc(1)=5;
thetac(1)=1;
for i=1:length(t)-1
    Px_int=Pxc(i)+dt*Px_feedback(A{i},B{i},K{i},Pxc(i),Pyc(i),thetac(i));
    Py_int=Pyc(i)+dt*Py_feedback(A{i},B{i},K{i},Pxc(i),Pyc(i),thetac(i));
    theta_int=thetac(i)+theta_feedback(A{i},B{i},K{i},Pxc(i),Pyc(i),thetac(i));
    Pxc(i+1)=Pxc(i)+dt/2*(Px_feedback(A{i},B{i},K{i},Pxc(i),Pyc(i),thetac(i))+...
        Px_feedback(A{i+1},B{i+1},K{i+1},Px_int,Py_int,theta_int));
    Pyc(i+1)=Pyc(i)+dt/2*(Py_feedback(A{i},B{i},K{i},Pxc(i),Pyc(i),thetac(i))+...
        Py_feedback(A{i+1},B{i+1},K{i+1},Px_int,Py_int,theta_int));
    thetac(i+1)=thetac(i)+dt/2*(theta_feedback(A{i},B{i},K{i+1},Pxc(i),Pyc(i),thetac(i))+...
        theta_feedback(A{i+1},B{i+1},K{i+1},Px_int,Py_int,theta_int));
end

figure;
subplot(2,1,1);
plot(t,Px,t,Pxc);
xlabel('Px');
ylabel('Time (sec)');
title('x Spacial Coordinate');
subplot(2,1,2);
plot(t,Py,t,Pyc);
xlabel('Py');
ylabel('Time (sec)');
title('y Spacial Coordinate');