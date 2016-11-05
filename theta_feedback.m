function f = theta_feedback(A,B,K,Px,Py,theta)

G=A-B*K;
f = G(3,1)*Px+G(3,2)*Py+G(3,3)*theta;

end