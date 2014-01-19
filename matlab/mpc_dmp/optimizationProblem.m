function [H,f,A,lb,ub,lbA,ubA]=optimizationProblem(W,Ph,Bt,options)

%%%%%%%% GENERAL FORMULATION %%%%%%%%

L=length(W{1}.z)/2; 
ws=options.ws;
nD=size(W{1}.D,2); %number of Demos (assuming it's the same for each DMP)
nDv=nD*L*(ws+1); %number of decision variables 

ph=[];
Xi=zeros(2*L*(ws+1),nD*L*(ws+1));
for i=1:ws+1
    c_ind=(i-1)*nD*L+1:i*nD*L; %column indices

    ph=[ph; Ph^(i-1)];
    %Put -Om[k+i=1] in the Diagonal of Xi
    Om=[];
    for l=1:L
        Om=blkdiag(Om,W{i}.D(2*l-1:2*l,:));    
    end    
    Xi((i-1)*2*L+1:i*2*L,c_ind)=(-1)*Om;

    for j=i:ws
        r_ind=j*2*L+1:(j+1)*2*L;  %row indices

        Kp=[];
        for l=1:L
            Kp=blkdiag(Kp,W{i}.U(l,:));
        end     
        Xi(r_ind,c_ind)=Ph^(j-i)*Bt*Kp;
    end
end
ph=ph*W{1}.z; %phi=[Phi^0, ..., Phi^ws]^T*zeta[k]

%compute distances between current state and demo states
v=zeros(nDv,1);
for i=1:L
    v((i-1)*nD+1:i*nD)=sum((repmat(W{1}.z(2*i-1:2*i),1,nD)-W{1}.D(2*i-1:2*i,:)).^2);
end  

%Prioritizing matrix
P=diag(repmat(diag(options.P),L*(ws+1),1));

%Hessian
H=Xi'*P*Xi; H=(H+H')/2;

%linear part of the objective
f=(ph'*P*Xi)'+options.epsilon*v;

%Bounds
lb=zeros(nD*L*(ws+1),1); ub=[];

%Equality constraints
A=kron(eye(L*(ws+1)),ones(1,nD)); lbA=ones(L*(ws+1),1); ubA=lbA;

%%%%%%%% ADDITIONAL CONSTRAINTS FOR OBSTACLE AVOIDANCE %%%%%%%%

for i=1:length(options.Constraints)
    if ws < 1
       warning('preview window size has to be at least 1 to perform obstacle avoidance');
       break;
    end     
    
        %check if the constraint is active
        %if .....
        %else
        %  continue;
        %end
    
    
        N=options.Constraints{i}.N;
        b=options.Constraints{i}.b;
    %Form an appropriate selection matrix depending on the constraint type
    S=zeros(L,2*L);
        if options.Constraints{i}.type=='p'
          for l=1:L
              S(l,2*l-1)=1;
           end   
        elseif options.Constraints{i}.type=='v'
          for l=1:L
              S(l,2*l)=1;
           end  
        else
            error('Unknown constraint type!');
        end

        dlt=ph(2*L+1:end);
            
        Y=zeros(2*L*ws,nD*L*(ws+1));
        for j=1:ws
           Y((j-1)*2*L+1:2*L*ws,(j-1)*nD*L+1:j*nD*L)=Xi(j*2*L+1:2*L*(ws+1),(j-1)*nD*L+1:j*nD*L);  
        end    
        NS=kron(eye(ws),N*S);

        %Augment the constraint matrices
        ubA=[ubA; -b*ones(ws,1)-NS*dlt];
        lbA=[lbA; -1e6*ones(ws,1)];
        A=[A; NS*Y];
end    