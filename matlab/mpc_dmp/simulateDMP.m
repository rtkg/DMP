function [t,s,X,Lmbd, ps_ref]=simulateDMP(DMP,x0,options)

if options.W < 1
    error('Preview window size has to be at least 1!');
end
if options.W > options.Tau/options.Td
 error('Preview window has to be smaller than Tau/Td');    
end

W=options.W;
Tau=options.Tau;
Td=options.Td;
x0=x0(:);

if ~isempty(options.PauseSimTime)
    t_pause=options.PauseSimTime;
else
    t_pause=Tau;
end
if ~isempty(options.PauseSimStep)
    step_pause=options.PauseSimStep;
else
    step_pause=Td;
end

ode_options=odeset('RelTol',options.RelTol,'AbsTol',options.AbsTol);


%%%%%%%%INITIAL STATE VECTOR [q, dq, q_1,dq_1, ... , q_D,dq_D]%%%%%%%%
nD=length(DMP.param);
for i=1:nD
   x0=[x0; DMP.param{i}.q0_ref;0]; 
end

%%%%%%%%%%%%%%%%%%CONTINOUS/DISCRETE STATE TRANSITION MATRIX A/A_%%%%%%%%
%abuse the DMP struct to store the duration adjusted state space parameters
%it is assumed that all DMP share the same matrices A and B!!!!
DMP.A=[0 1;DMP.param{1}.w(1)/Tau^2 DMP.param{1}.w(2)/Tau];
DMP.invA=inv(DMP.A); %store the inverse to avoid recomputation at every timestep
DMP.A_=expm(DMP.A*Td); %discrete state transition matrix
DMP.B=[0;1/Tau^2];
DMP.powA_=eye(2);
for i=1:W
   DMP.powA_(end+1:end+2,:)=DMP.A_^i; %precompute and store the powers of A_
end



%%%%%%%%%%%%%%%%%%%%%INITIALIZE MPC PREVIEW WINDOW%%%%%%%%%%%%%%%%%%%%%%%%%%%
%the first W particular solutions P[k+1] ... P[k+W] in the preview window
s=0;
t=0;
for i=1:W
t(i+1,1)=i*Td;
[s(i+1,1) P{i}]=integrateStep(DMP,s(i),t(i:i+1),options);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%SIMULATE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X=x0'; 
for i=1:Tau/Td; 
    ps_ref{i}=P{1};
    
    %compute the cc factors for the next step
    if options.mpc
        Lmbd(i,:)=mpcCC(DMP,X(i,:),P,options);
    else
        S=[];
        for j=1:length(DMP.param)
            S=[S X(i,2*j+1:2*j+2)'];
        end
        c=CC(S,X(i,1:2)',options.P,5,options.kappa);
        Lmbd(i,:)=c';
    end
    
  %Update the states for one step
  X(i+1,:)=updateStates(DMP.A_,Lmbd(i,:),P{1},X(i,:));

  %Update preview window, phase variable and time
  t(end+1)=t(end)+Td;
  [s(end+1) P{end+1}]=integrateStep(DMP,s(end),t(end-1:end),options);
  P(1)=[];
end

%remove the additional entries for the preview window from the time and phase variable vector
t(end-W+1:end)=[];
s(end-W+1:end)=[];

%add the last set of cc coefficients to make the sizes comparable
Lmbd(end+1,:)=Lmbd(end,:);
%EOF


