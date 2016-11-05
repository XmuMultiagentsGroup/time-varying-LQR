function f = Py_dot(A,B,Px,Py,theta,u1,u2)

f = A(2,1)*Px+A(2,2)*Py+A(2,3)*theta+...
    B(2,1)*u1+B(2,2)*u2;

end