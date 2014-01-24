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
tic
pI = initialGuess(nBF,demos,r_min,g_min,e,norm);
t_init=toc;
S=scaleMatrix(nBF,nD,pI,norm); 

[A, b]=inequalityConstraints(nBF,nD,cd_min,g_min,norm);
[lb, ub]=bounds(nBF,nD,w_max,r_min,g_min,norm);
[Aeq, beq]=equalityConstraints(nBF,nD,e,norm);

%make sure initial solution is feasible
if (max(A*pI-b) > 0 || max(lb-pI) > 0 || min(ub-pI) < 0 || max(abs(Aeq*pI-beq)) >= 1e-12)
    % disp('Warning: Infeasible initial solution');
end

if strcmp(param.kernel,'optimized')
    tic
    pS = nlsq_ACADO(demos,pI,A,b,Aeq,beq,lb,ub,S);
    t_fit=toc;
    p=S*pS;
elseif strcmp(param.kernel,'fixed')
    p=pI;
    t_fit=0;
else
    error('Unknown kernel specification');
end

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

% cmap = hsv(nBF);
% figure; hold on; grid on;
% for i=1:nBF
%     ind = 2*i-1:2*i;
%     s=0:0.00001:1;
%     g=gaussmf(s,pBF(ind));
%     plot(s,g,'color',cmap(i,:)); 
% end
% plot(0:1/nBF:1,ones(1+nBF,1),'r.','MarkerSize',20);
%figure;

DMP.id=motion.id;
DMP.name=motion.name;
DMP.t_learn=t_init+t_fit;

% --------------------------------------------------------------------
% simulate
% --------------------------------------------------------------------


Tend=1;
euler_int=1;
%figure;   
for i=1:length(demos)     
    t=demos{i}.D(:,1);
    w=p((2+nBF)*(i-1)+1:(2+nBF)*i);
    wI=pI((2+nBF)*(i-1)+1:(2+nBF)*i);

    ind_e=length(t);
    t=0:Td:t(end)*Tau*Tend;
    x0=[0;demos{i}.D(1,2:3)'];
    
    DMP.param{i}.w=w;
    DMP.param{i}.wI=wI;
   DMP.param{i}.pBFI=pBFI;
    DMP.param{i}.pBF=pBF;
    DMP.param{i}.q0_ref=x0(2);
    DMP.param{i}.D=demos{i}.D;

    if (~euler_int)
        [T,X] = ode45(@dyn_sys,t,x0,[],DMP.param{i},Tau);
        ddx=diff(X(:,3))/Td; ddx(end+1)=ddx(end);   
        
        %subplot(3,1,1);  hold on; grid on;
        %plot(T(end),0,'b.','MarkerSize',20);
        %plot(demos{i}.D(:,1),demos{i}.D(:,2),'r');
        %plot(T,X(:,2),'k--');
        %title(strcat(DMP.name,'-',joint,' Position'));

        %subplot(3,1,2);  hold on; grid on;
        %plot(T(end),0,'b.','MarkerSize',20);
        %plot(demos{i}.D(:,1),demos{i}.D(:,3),'r');
        %plot(T,X(:,3),'k--');
        %title(strcat(DMP.name,'-',joint,' Velocity'));

        
        %subplot(3,1,3);  hold on; grid on;
        %plot(T(end),0,'b.','MarkerSize',20);
        %plot(demos{i}.D(:,1),demos{i}.D(:,4),'r');
        %plot(T,ddx,'k--');
        %title(strcat(DMP.name,'-',joint,' Acceleration'));
    else
        [T,X,f] = eulerIntegrator([t(1);t(end)],Td,x0,DMP.param{i},Tau);   
       if (abs(X(end,2)) > 1e-1)
            %HHHAAAACCKKK to detect unsuccesful solutions
DMP.param{i}.w=DMP.param{i}.wI;
DMP.param{i}.pBF=DMP.param{i}.pBFI;
disp('Detected unsuccesfull ACADO solution ...');

             [T,X,f] = eulerIntegrator([t(1);t(end)],Td,x0,DMP.param{i},Tau); 
         end   

        % subplot(4,1,1);  hold on; grid on;
        % plot(T(end),0,'b.','MarkerSize',20);
        % plot(demos{i}.D(:,1),demos{i}.D(:,2),'r');
        % plot(T,X(:,2),'k--');
        % title(strcat(DMP.name,'-',joint,' Position'));

        % subplot(4,1,2);  hold on; grid on;
        % plot(T(end),0,'b.','MarkerSize',20);
        % plot(demos{i}.D(:,1),demos{i}.D(:,3),'r');
        % plot(T,X(:,3),'k--');
        % title(strcat(DMP.name,'-',joint,' Velocity'));

        % subplot(4,1,3);  hold on; grid on;
        % plot(T(end),0,'b.','MarkerSize',20);
        % plot(demos{i}.D(:,1),demos{i}.D(:,4),'r');
        % plot(T,X(:,4),'k--');
        % title(strcat(DMP.name,'-',joint,' Acceleration'));
        
        % subplot(4,1,4);  hold on; grid on;
        % plot(T(end),0,'b.','MarkerSize',20);
        % plot(T,f,'b');
        % title(strcat(DMP.name,'-',joint,' Forcing term'));

        E=[demos{i}.D(:,2)-X(:,2) demos{i}.D(:,3)-X(:,3)];
        DMP.param{i}.rep_err.sq_sum=sum(E.^2);
        DMP.param{i}.rep_err.E=E;
 
    end
end   

%%%EOF
