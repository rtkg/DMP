function [H,f,A,lb,ub,lbA,nE]=optimizationProblem(W,Ph,Bt,Ocon,options)

%%%%%%%% GENERAL FORMULATION %%%%%%%%

L=length(W{1}.z)/2; 
ws=options.ws;
nD=size(W{1}.D,2); %number of Demos (assuming it's the same for each DMP)
nDv=(nD+1)*L*(ws+1); %number of decision variables 

ph=[];
Xi=zeros(2*L*(ws+1),nD(+1)*L*(ws+1));
for i=1:ws+1
    c_ind=(i-1)*(nD+1)*L+1:i*(nD+1)*L; %column indices

    ph=[ph; Ph^(i-1)];
    %Put -Om[k+i=1] in the Diagonal of Xi
    Om=[];
    for l=1:L
        Om=blkdiag(Om,[W{i}.D(2*l-1:2*l,:) zeros(2,1)]);    
    end    
    Xi((i-1)*2*L+1:i*2*L,c_ind)=(-1)*Om;

    for j=i:ws
        r_ind=j*2*L+1:(j+1)*2*L;  %row indices

        Kp=[];
        for l=1:L
            Kp=blkdiag(Kp,[W{i}.U(l,:) 1]);
        end     
        Xi(r_ind,c_ind)=Ph^(j-i)*Bt*Kp;
    end
end
ph=ph*W{1}.z; %phi=[Phi^0, ..., Phi^ws]^T*zeta[k]

%compute distances between current state and demo states
v=zeros(nDv,1);

for i=1:L
    v((i-1)*(nD+1)+1:i*(nD+1))=[sum((repmat(W{1}.z(2*i-1:2*i),1,nD)-W{1}.D(2*i-1:2*i,:)).^2) 0];
end  

%State prioritizing matrix
Ps=diag(repmat(diag(options.Ps),L*(ws+1),1));

%Weight matrix for the auxiliary control inputs
Po=kron(eye(L*(ws+1)),blkdiag(diag(ones(nD,1)),options.Po));

%Hessian
H=Po'*Xi'*Ps'*Ps*Xi*Po; H=(H+H')/2;

%linear part of the objective
f=(ph'*Ps'*Ps*Xi*Po)'+options.epsilon*v;

%Bounds
minb=-1e6; maxb=1e6;
lb=repmat([zeros(nD,1); minb],L*(ws+1),1); ub=maxb*ones((nD+1)*L*(ws+1),1); 

%Equality constraints
A=kron(eye(L*(ws+1)),[ones(1,nD) 0]); lbA=-ones(L*(ws+1),1); 
nE=L*(ws+1);

%%Keep the auxiliary control inputs to zero
% A=[kron(eye(L*(ws+1)),[zeros(1,nD) 1]); A]; lbA=[ zeros(L*(ws+1),1); lbA];
% nE=nE+L*(ws+1);
%%%%%%%% ADDITIONAL CONSTRAINTS FOR OBSTACLE AVOIDANCE %%%%%%%%

if ws < 1
    warning('preview window size has to be at least 1 to perform obstacle avoidance');
    return;
end  

%Form appropriate selection matrices to pick position/velocity vector from the state vector zeta
Sp=kron(eye(L),[1 0]);
Sv=kron(eye(L),[0 1]);

dlt=ph(2*L+1:end);
Y=zeros(2*L*ws,(nD+1)*L*(ws+1));
for j=1:ws
    Y((j-1)*2*L+1:2*L*ws,(j-1)*(nD+1)*L+1:j*(nD+1)*L)=Xi(j*2*L+1:2*L*(ws+1),(j-1)*(nD+1)*L+1:j*(nD+1)*L);  
end      

for i=1:length(options.Obstacles)
    %Augment the constraint matrices
    if isempty(Ocon{i}.ba)
        continue;
    end
    HSp=[];
    for j=1:ws
        HSp=blkdiag(HSp,Ocon{i}.Ha(j,:)*Sp);
    end
    keyboard
    lbA=[lbA; -HSp*dlt-Ocon{i}.ba];
    A=[A; -HSp*Y];
end    