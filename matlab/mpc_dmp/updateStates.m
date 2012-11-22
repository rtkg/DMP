function xk1=updateStates(A_,lmbd,Pk,xk)
%xk=(q[k],dq[k],q_1[k],dq_1[k], ... ,q_D[k],dq_D[k])
%xk1=(q[k+1],dq[k+1],q_1[k+1],dq_1[k+1], ... ,q_D[k+1],dq_D[k+1])

lmbd=lmbd(:);
xk=xk(:);
D=size(Pk,2);
xk1=zeros(2+2*D,1);

xk1(1:2)=A_*xk(1:2)+Pk*lmbd;
for d=1:D
   ind=2+2*d-1:2*d+2;
   xk1(ind)=A_*xk(ind)+Pk(:,d);
end    

