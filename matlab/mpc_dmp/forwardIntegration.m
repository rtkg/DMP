function W=forwardIntegration(Wk,nS,DOFs,options)
W{1}=Wk;
Td=options.Td;
Tau=options.Tau;
nDofs=length(DOFs);

for k=1:nS
    W{k}.U=[];W{k+1}.D=[];

    for i=1:nDofs    
        Ui=[]; 
        for j=1:length(DOFs{i}.param)
            nBF=length(DOFs{i}.param{j}.pBF)/2;
            w=DOFs{i}.param{j}.w(3:end); %basis function weights (without a & b)
            pBF=DOFs{i}.param{j}.pBF; %nonlinear basis function parameters

            u=[];
            for b=1:nBF
                ind=2*b-1:2*b;
                u=[u gaussmf(W{k}.s,pBF(ind))];
            end    
            Ui=[Ui u*w];   
        end
       W{k}.U=[W{k}.U; Ui]; %DMP forcing terms
       W{k+1}.D=[W{k+1}.D; DOFs{i}.A_*W{k}.D(2*i-1:2*i,:)+DOFs{i}.B_*Ui]; %Encoded demonstrated trajectories
    end    

    %integrate the canonical system   
    W{k+1}.s=W{k}.s+Td/Tau; %ds=1/Tau
    
end    


%EOF
