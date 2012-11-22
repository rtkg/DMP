function pI = initialGuess(nBF,demos,r_min,g_min,e,norm)
%Computes the initial guess via fixing the basis functions equidistantly and setting maximal BF
%widths s.t. r < r_max. The parameter norm determines whether the BF weights and dynamic
%parameters a & b are computed via l2, l1 or linf norm.

r_max=sqrt(-1/2/log(g_min)); %maximum width s.t. a gaussian at c=0: g(1)=g_min 
c=1/nBF/2:1/nBF:1-1/nBF/2; %equidistant spacing 
r=r_max*(1-c); %linear function for r s.t. 0<= r <= r_max on the intervall c =[0,1]

pBF=reshape([r(:) c(:)]',1, [])'; %combined gaussian parameters with alternating witdth and
                                  %position entries
pI=[];
ab=[-log(e)^2; -sqrt(4*log(e)^2)];
for i=1:length(demos)
    if norm==1
        w=find_weights_L2(demos{i}.D,pBF,nBF,ab);
    elseif norm == 2
        error('linf norm not implemented yet');
    elseif norm == 3
        error('l1 norm not implemented yet');
    else
        error('Invalid norm: 1->l2, 2->linf,3->l1');
    end
    
    pI=[pI;w];
end
pI=[pI;pBF];


if norm==2
    %find inital guess for the additional parameter z as the maximum residual
    r=residual(pI,D{1});
    pI(end+1)=max(abs(min(r)),max(r));
end