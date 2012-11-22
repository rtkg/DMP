function [s_k1 P_k]=integrateStep(DMP,s_k,t,options)

ode_options=odeset('RelTol',options.RelTol,'AbsTol',options.AbsTol);
P_k=[];
if ~options.ExactDisc
    %integrate the canonical system
    [dummy,s_] = ode45(@canonicalSystem,t,s_k,ode_options,options.Tau);  
    s_k1=s_(end);

    for i=1:length(DMP.param)
        nBF=length(DMP.param{i}.pBF)/2;
        w=DMP.param{i}.w(3:end); %basis function weights (without a & b)
        pBF=DMP.param{i}.pBF; %nonlinear basis function parameters
        ud_=[];

        for j=1:nBF
            ind=2*j-1:2*j;
            ud_=[ud_ gaussmf(s_k,pBF(ind))];
        end    
        ud=ud_*w;
        
        %Integral assuming that ud is constant for t in [k*Td,(k+1)*Td]
        P_k=[P_k DMP.invA*(DMP.A_-eye(2))*DMP.B*ud];
    end    
else
     [dummy,p_] = ode45(@particularSolutions,t,[s_k; zeros(length(DMP.param)*2,1)],ode_options,t(end),DMP,options); 
     s_k1=p_(end,1);
     for i=1:length(DMP.param)
        P_k=[P_k p_(end,2*i:2*i+1)']; 
     end    
end
%EOF
