function f = x2_feedback(A,B,K,x1,x2,F)

G=A-B*K;
f = G(2,1)*x1+G(2,2)*x2+B(2,1)*F;

end