function S=simulateCoupledDMP(DOFs,options)

if options.ws < 0
    error('Preview window size has to be larger than 0!');
end
if options.ws > options.T/options.Td
    error('Preview window has to be smaller than T/Td');    
end

ws=options.ws;
Tau=options.Tau;
Td=options.Td;
T=options.T;
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
W{1}.k=1;

%make sure to integrate at least 1 step forward, even if the preview window size = 0
if ws==0
    nS=1;
else
    nS=ws;
end

nD=size(W{1}.D,2); %number of Demos (assuming it's the same for each DMP)
nDv=nD*L*(ws+1); %number of decision variables 
Ocon{1}.Ha=[];Ocon{1}.ba=[];
for k=1:ceil(T/Td)+1
    W=forwardIntegration(W{1},nS,DOFs,options); %integrate the window forward in time

    %%%%%%%%%%%%%%%%%% FORMULATE AND SOLVE THE OPTIMIZATION PROBLEM %%%%%%%%%
    [H,f,A_Aeq,lb,ub,lbA,nE]=optimizationProblem(W,Ph,Bt,Ocon,options);  
    [Mu,lmbd,exitflag] =qld(H, A_Aeq, f, lbA,lb, ub, nE,1);

    if exitflag ~= 0
        warning('QP was not solved succesfully!');
        %    keyboard
    end    
    %%%%%%%% COMPUTE THE PREDICTED STATES%%%%%%%%

    for i=1:nS
        mu=Mu((i-1)*(nD+1)*L+1:i*(nD+1)*L); 
        W{i}.mu=mu;
        Kp=[];
        for l=1:L
            Kp=blkdiag(Kp,[W{i}.U(l,:) 1]);
        end     
        W{i+1}.z=Ph*W{i}.z+Bt*Kp*mu;
    end    

    if k==1
        aC=obstacleAvoidance({W{1:ws+1}},options);%fill the active constraint window in the first step
    else
        aC=obstacleAvoidance({W{ws+1}},options); %only evaluate the active constraints for k+P
    end  

    %Concatenate the previous and current active constraints and pop the one at k (the first one)
    %from the list
    for i=1:length(Ocon)
        Ocon{i}.Ha=[Ocon{i}.Ha; aC{i}.Ha];
        Ocon{i}.ba=[Ocon{i}.ba; aC{i}.ba];
        Ocon{i}.Ha(1,:)=[]; Ocon{i}.ba(1)=[];
    end

    S{k}=W; %save all the states and computed values for the current step k
    W{1}=W{2};%iterate the window for the next step
    
end %Loop end



