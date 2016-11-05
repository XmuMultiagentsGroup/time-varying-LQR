function f = Px_feedback(A,B,K,Px,Py,theta)

G=A-B*K;
f = G(1,1)*Px+G(1,2)*Py+G(1,3)*theta;

end