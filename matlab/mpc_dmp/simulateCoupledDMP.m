function S=simulateCoupledDMP(DOFs,options)

if options.ws < 0
    error('Preview window size has to be larger than 0!');
end
if options.ws > options.Tau/options.Td
 error('Preview window has to be smaller than Tau/Td');    
end

ws=options.ws;
Tau=options.Tau;
Td=options.Td;
L=length(DOFs);


%%%%%%%%%%%%%%%%%%DISCRETE COUMPUND STATE TRANSITION MATRICEX Phi (Ph) & Beta (Bt) %%%%%%%%%

Ph=[];
Bt=[];

for i=1:L
    %continous state matrix & control vector
   A=[0 1;DOFs{i}.param{1}.w(1)/Tau^2 DOFs{i}.param{1}.w(2)/Tau];
   B=[0;1/Tau^2];
   %discrete state matrix & control vector   
   DOFs{i}.A_=expm(A*Td);
   DOFs{i}.B_=inv(A)*(DOFs{i}.A_-eye(2))*B;
   
   Ph=blkdiag(Ph,DOFs{i}.A_);
   Bt=blkdiag(Bt,DOFs{i}.B_);

  
  
end 
%%%%%%%%%%%%%%%%%% INITIAL STATES %%%%%%%%%
W{1}.z=[]; W{1}.D=[];
for i=1:L
    W{1}.z=[W{1}.z; DOFs{i}.x0];
    Xd0=[];
    for j=1:length(DOFs{i}.param)
        Xd0=[Xd0 [DOFs{i}.param{j}.q0_ref ;0]];
    end
    W{1}.D= [W{1}.D;Xd0]; 
end
W{1}.s=0;
W{1}.t=0;


%make sure to integrate at least 1 step forward, even if the preview window size = 0
if ws==0
    nS=1;
else
    nS=ws;
end

for k=1:ceil(Tau/Td)+1

    W=forwardIntegration(W{1},nS,DOFs,options);

    %%%%%%%%%%%%%%%%%% FORMULATE THE OPTIMIZATION MATRICES %%%%%%%%%
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
    Aeq=kron(eye(L*(ws+1)),ones(1,nD)); beq=ones(L*(ws+1),1);


    %%%%%%%% SOLVE THE OPTIMIZATION PROBLEM%%%%%%%%

    [Mu,fval,exitflag,iter,lmbd] = qpOASES(H,f,Aeq,lb,ub,beq,beq);

    %%%%%%%% COMPUTE THE PREDICTED STATES%%%%%%%%

    for i=1:ws
        mu=Mu((i-1)*nD*L+1:i*nD*L); 
        W{i}.mu=mu;
        W{i+1}.t=W{i}.t+Td; %increase the time
        Kp=[];
        for l=1:L
            Kp=blkdiag(Kp,W{i}.U(l,:));
        end     
        W{i+1}.z=Ph*W{i}.z+Bt*Kp*mu;
    end    
    
S{k}=W; %save all the states and computed values for the current step k
W{1}=W{2};%iterate the window for the next step
    
end %Loop end



