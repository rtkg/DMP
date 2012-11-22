function w = find_weights_L2(d,p,nBF,ab)
%
% find linear BF weights and dynamic parameters by solving a QP, enforcing critical damping w.r.t
% the dynamics parameters a and b
% single demonstration only!

s=d(:,1);
x=d(:,2);
dx=d(:,3);
ddx=d(:,4);

nS=length(s);
B = [x, dx, zeros(nS,nBF)];
y = ddx;

for i=1:nBF
    ind = 2*i-1:2*i; 
    B(:,i+2)=gaussmf(s,p(ind));
end


UL = 1e5;
options = optimset('Display','off','Algorithm','active-set','LargeScale','off');

Q = eye(length(ddx));

H =  B'*Q*B;
g = -B'*Q*y;
H=(H+H')/2;

%Fix a & b
Aeq=zeros(2,2+nBF);beq=zeros(2,1);
Aeq(1,1)=1; Aeq(2,2)=1;
beq(1)=ab(1); beq(2)=ab(2);

lb = -UL*ones(nBF+2,1);  
ub =  UL*ones(nBF+2,1);  

w = quadprog(H,g,[],[],Aeq,beq,lb,ub,[],options);

%%%EOF