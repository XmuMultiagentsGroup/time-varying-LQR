function f = theta_dot(A,B,Px,Py,theta,u1,u2)

f = A(3,1)*Px+A(3,2)*Py+A(3,3)*theta+...
    B(3,1)*u1+B(3,2)*u2;

end