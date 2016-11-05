function f = Py_feedback(A,B,K,Px,Py,theta)

G=A-B*K;
f = G(2,1)*Px+G(2,2)*Py+G(2,3)*theta;

end