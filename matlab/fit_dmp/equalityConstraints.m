function [Aeq beq]=equalityConstraints(nBF,nD,e,norm)
%equality constraints for
%p=[a_1,b_1,w_11, ..., w_1n,...,a_nD,b_nD,w_nD1, ..., w_nDn, r_1,c_1, ... , r_n,c_n]
%enforcing a & b to set values

Aeq=zeros(2*nD+nBF*(2+nD)); beq=zeros(2*nD+nBF*(2+nD),1);
for i=1:nD
    Aeq((2+nBF)*(i-1)+1,(2+nBF)*(i-1)+1)=1;
    Aeq((2+nBF)*(i-1)+2,(2+nBF)*(i-1)+2)=1;
    beq((2+nBF)*(i-1)+1)=-log(e)^2;
    beq((2+nBF)*(i-1)+2)=-sqrt(4*log(e)^2);
end

if norm==2
    Aeq(:,end+1)=0; 
end    