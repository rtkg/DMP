function Sm=scaleMatrix(nBF,nD,pI,norm)
%Construct a scale matrix for the parameters a,b,w,r and c based on the respective maximal
%values of the initial guess


S=zeros(2*nD+nBF*(2+nD),1);
Sa=0; Sb=0; Sw=0; Sr=0; Sc=0; 

%find the maximal values for a,b,w,r and c over all demos
for i=1:nD
    ind_a{i}=(2+nBF)*(i-1)+1;
    ind_b{i}=(2+nBF)*(i-1)+2;
    ind_w{i}=i*(2+nBF)-nBF+1:1:i*(2+nBF);
    ind_r{i}=nD*(nBF+2)+1:2:2*nD+nBF*(2+nD)-1;
    ind_c{i}=nD*(nBF+2)+2:2:2*nD+nBF*(2+nD);
    
    if (max(abs(pI(ind_a{i}))) > Sa  )
        Sa=max(abs(pI(ind_a{i})));
    end
    if (max(abs(pI(ind_b{i}))) > Sb  )
        Sb=max(abs(pI(ind_b{i})));
    end
    if (max(abs(pI(ind_w{i}))) > Sw  )
        Sw=max(abs(pI(ind_w{i})));
    end
     if (max(abs(pI(ind_r{i}))) > Sr  )
        Sr=max(abs(pI(ind_r{i})));
     end 
    if (max(abs(pI(ind_c{i}))) > Sc  )
        Sc=max(abs(pI(ind_c{i})));
    end 
end

for i=1:nD
    S(ind_a{i})=Sa;
    S(ind_b{i})=Sb;
    S(ind_w{i})=Sw;
    S(ind_r{i})=Sr;
    S(ind_c{i})=Sc;
end

if norm==2
    S(end+1)=abs(pI(end));
end

Sm=diag(S);
