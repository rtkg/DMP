function cc=mpcCC(DMP,xk,P,options)

xk=xk(:);
W=options.W;
D=size(P{1},2);
U=zeros(2*W,D*W);

Rk=[];
for i=1:D
    Rk=[Rk xk(2*i+1:2*i+2)]; %Arrange matrix holding the current reference states as column vectors
end    

U_=[];
E=[];
for i=1:W
    ind_r=2*i-1:2*i;
    for j=1:i
        ind_c=D*(j-1)+1:D*j;
        ind_powA_=2*(i-j)+1:2*(i-j)+2;
        U(ind_r,ind_c)=DMP.powA_(ind_powA_,:)*P{j};    
    end
    E=[E; eye(D)];
    U_=blkdiag(U_,DMP.powA_(2*i+1:2*i+2,:)*Rk+U(ind_r,1:size(E,1))*E);
end

Atd=DMP.powA_(3:end,:)*xk(1:2);
Btd=U-U_;

%compute distances
l=zeros(1,D);
for d=1:D
    l(d)=norm(options.P*xk(1:2)-options.P*xk(2+2*d-1:2*d+2));
end    

%Objective function
H=Btd'*diag(repmat(diag(options.P),W,1))*Btd; H=(H+H')/2;
f=(Atd'*diag(repmat(diag(options.P),W,1))*Btd -options.kappa*[l zeros(1,D*(W-1))])';

%Constraints
%Aeq=cell(1,W); [Aeq{:}]=deal(ones(1,D)); Aeq=blkdiag(Aeq{:});
Aeq=zeros(W,W*D);
for i=1:W
  Aeq(i,i*D-D+1:i*D)=ones(1,D);
end
beq=ones(W,1);
lb=zeros(D*W,1);
ub=ones(D*W,1);

%QPR_options = optimset('Algorithm','active-set','Display','off','LargeScale','off','MaxIter',10000);
%[cc, fval, exitflag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],QPR_options);

%evalc is just to get rid of the f***ing qpOASES copyright statements in the command window
[T,cc,fval,exitflag,iter,lambda] = evalc('qpOASES(H,f,Aeq,lb,ub,beq,beq)');
cc=cc(1:D)';

if exitflag ~= 0 %exitflag 1 for quadprog
    error(strcat('QP finished with exitflag: ',num2str(exitflag)));
end    
