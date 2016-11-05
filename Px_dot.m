function f = Px_dot(A,B,Px,Py,theta,u1,u2)

f = A(1,1)*Px+A(1,2)*Py+A(1,3)*theta+...
    B(1,1)*u1+B(1,2)*u2;

end