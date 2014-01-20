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
W{1}.k=1;

%make sure to integrate at least 1 step forward, even if the preview window size = 0
if ws==0
    nS=1;
else
    nS=ws;
end

nD=size(W{1}.D,2); %number of Demos (assuming it's the same for each DMP)
nDv=nD*L*(ws+1); %number of decision variables 
for k=1:ceil(Tau/Td)+1
    W=forwardIntegration(W{1},nS,DOFs,options); %integrate the window forward in time

    %%%%%%%%%%%%%%%%%% FORMULATE AND SOLVE THE OPTIMIZATION PROBLEM %%%%%%%%%
    qpO_options=qpOASES_options;
    qpO_options.maxIter=1000;
     enableRegularisation=1;
    [H,f,A_Aeq,lb,ub,lbA,ubA]=optimizationProblem(W,Ph,Bt,options);

    if k==1
        [Mu,fval,exitflag,iter,lmbd] = qpOASES(H,f,A_Aeq,lb,ub,lbA,ubA,[],qpO_options);
    else
        [Mu,fval,exitflag,iter,lmbd] = qpOASES(H,f,A_Aeq,lb,ub,lbA,ubA,Mu,qpO_options); %start QP from previous
                                                                              %Solution if applicable
    end
    if exitflag ~= 0
        %    keyboard
    end    
    %%%%%%%% COMPUTE THE PREDICTED STATES%%%%%%%%

    for i=1:nS
        mu=Mu((i-1)*(nD+1)*L+1:i*(nD+1)*L); 
        W{i}.mu=mu;
        W{i+1}.t=W{i}.t+Td; %increase the time
        W{i+1}.k=k+1;
        Kp=[];
        for l=1:L
            Kp=blkdiag(Kp,[W{i}.U(l,:) 1]);
        end     
        W{i+1}.z=Ph*W{i}.z+Bt*Kp*mu;
    end    

    S{k}=W; %save all the states and computed values for the current step k
    W{1}=W{2};%iterate the window for the next step
    
end %Loop end



