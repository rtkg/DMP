function DMP = computeDMPs(motion,joint,param)
% ACADO nonlinear least squares norm version with distance constraints on the gaussian
%BF centers and fixed a & b

demos=motion.demos; %assumes that demo positions are shifted to 0!
nBF=param.nBF;
r_min=param.r_min;
w_max=param.w_max;
g_min=param.g_min;
cd_min=param.cd_min;
Tau=param.Tau;
norm=param.norm;
Td=param.Td;
e=param.e;
nD=length(demos);

% --------------------------------------------------------------------
% Nonlinear least square solution for the gaussian BF parameters
% --------------------------------------------------------------------

pI = initialGuess(nBF,demos,r_min,g_min,e,norm);
S=scaleMatrix(nBF,nD,pI,norm); 

[A, b]=inequalityConstraints(nBF,nD,cd_min,g_min,norm);
[lb, ub]=bounds(nBF,nD,w_max,r_min,g_min,norm);
[Aeq, beq]=equalityConstraints(nBF,nD,e,norm);

%make sure initial solution is feasible
if (max(A*pI-b) > 0 || max(lb-pI) > 0 || min(ub-pI) < 0 || max(abs(Aeq*pI-beq)) >= 1e-12)
    disp('Warning: Infeasible initial solution');
end


pS = nlsq_ACADO(demos,pI,A,b,Aeq,beq,lb,ub,S);
p=S*pS;

if (~isempty(find(isnan(p),1,'first')))
    disp('Warning: solution vector contains NaN - setting weights to zero!');

    p=zeros(size(p));
    ab=pI(1:2);
   for i=1:nD
       p((2+nBF)*(i-1)+1:(2+nBF)*(i-1)+2)=ab; %set ab
   end    

   %set the nonlinear parameters to very small values 
   p(end-2*nBF+1:end)=1e-6;
end    

pBF=p(1+nD*(2+nBF):nD*(2+nBF)+2*nBF);
pBFI=pI(1+nD*(2+nBF):nD*(2+nBF)+2*nBF);
cmap = hsv(nBF);
figure; hold on; grid on;
for i=1:nBF
    ind = 2*i-1:2*i;
    s=0:0.00001:1;
    g=gaussmf(s,pBF(ind));
    plot(s,g,'color',cmap(i,:)); 
end
plot(0:1/nBF:1,ones(1+nBF,1),'r.','MarkerSize',20);

% --------------------------------------------------------------------
% simulate
% --------------------------------------------------------------------

DMP.id=motion.id;
DMP.name=motion.name;

euler_int=0;
figure; 
for i=1:length(demos)     
    t=demos{i}.D(:,1);
    w=p((2+nBF)*(i-1)+1:(2+nBF)*i);
    
    ind_e=length(t);
    t=0:Td:t(end)*Tau*1.5;
    x0=[0;demos{i}.D(1,2:3)'];
    
    DMP.param{i}.w=w;
    DMP.param{i}.pBF=pBF;
    DMP.param{i}.q0_ref=x0(2);
    DMP.param{i}.D=demos{i}.D;

    if (~euler_int)
        [T,X] = ode45(@dyn_sys,t,x0,[],DMP.param{i},Tau);
        ddx=diff(X(:,3))/Td; ddx(end+1)=ddx(end);   
        
        subplot(3,1,1);  hold on; grid on;
        plot(T(end),0,'b.','MarkerSize',20);
        plot(demos{i}.D(:,1),demos{i}.D(:,2),'r');
        plot(T,X(:,2),'k--');
        title(strcat(DMP.name,'-',joint,' Position'));

        subplot(3,1,2);  hold on; grid on;
        plot(T(end),0,'b.','MarkerSize',20);
        plot(demos{i}.D(:,1),demos{i}.D(:,3),'r');
        plot(T,X(:,3),'k--');
        title(strcat(DMP.name,'-',joint,' Velocity'));

        subplot(3,1,3);  hold on; grid on;
        plot(T(end),0,'b.','MarkerSize',20);
        plot(demos{i}.D(:,1),demos{i}.D(:,4),'r');
        plot(T,ddx,'k--');
        title(strcat(DMP.name,'-',joint,' Acceleration'));
    else
        [T,X,f] = eulerIntegrator([t(1);t(end)],Td,x0,DMP.param{i},Tau);      
        subplot(4,1,1);  hold on; grid on;
        plot(T(end),0,'b.','MarkerSize',20);
        plot(demos{i}.D(:,1),demos{i}.D(:,2),'r');
        plot(T,X(:,2),'k--');
        title(strcat(DMP.name,'-',joint,' Position'));

        subplot(4,1,2);  hold on; grid on;
        plot(T(end),0,'b.','MarkerSize',20);
        plot(demos{i}.D(:,1),demos{i}.D(:,3),'r');
        plot(T,X(:,3),'k--');
        title(strcat(DMP.name,'-',joint,' Velocity'));

        subplot(4,1,3);  hold on; grid on;
        plot(T(end),0,'b.','MarkerSize',20);
        plot(demos{i}.D(:,1),demos{i}.D(:,4),'r');
        plot(T,X(:,4),'k--');
        title(strcat(DMP.name,'-',joint,' Acceleration'));
        
        subplot(4,1,4);  hold on; grid on;
        plot(T(end),0,'b.','MarkerSize',20);
        plot(T,f,'b');
        title(strcat(DMP.name,'-',joint,' Forcing term'));
    end
end   

%%%EOF

