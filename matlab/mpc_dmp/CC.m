function c = CC(S,z,varargin)

if nargin > 2
    P=varargin{1};
else
    P=eye(length(z));
end
if nargin > 3
    CvxCmb=varargin{2};
else
    CvxCmb=1;
end
if nargin > 4
    lmbd=varargin{3};
else
    lmbd=1;
end

nci=size(S,2);
QPR_options = optimset('Algorithm','active-set','Display','off','LargeScale','off','MaxIter',10000);
QPO_options=qpOASES_options('maxIter',100,'epsRegularisation',1e-8 ,'printLevel',0);

if CvxCmb==1 
    
    %UNIQUE CONVEX COMBINATION CONSIDERING ONLY POSITIONS
    S=S(1,:); z=z(1);
    c=zeros(nci,1);
    if (z<=min(S))
        [dummy ind]=min(S);
        c(ind)=1;
    elseif (z>=max(S))
        [dummy ind]=max(S);
        c(ind)=1;
    else   
        [S_sort ind_orig]=sort(S);
        [dummy ind_l ]=find(S_sort <=z);
        [dummy ind_u ]=find(S_sort >z);
        ind=ind_orig([ind_l(end) ind_u(1)]);
        
        Aeq=[S(ind); ones(1,2)];
        beq=[z; 1];
        lb=zeros(2,1);
        ub=ones(2,1);
        [c_, fval,exitflag,output,lambda]=linprog([],[],[],Aeq,beq,lb,ub,[],QPR_options);
        
        if (exitflag ~= 1)
             error('No valid solution for LP found');
        end
        
        c(ind)=c_;
    end
    
elseif CvxCmb==2
    
    %L2 DISTANCE WEIGHTING INCLUDING PRIORIZATION MATRIX P  
    D= P*S-repmat(P*z,1,nci); %Prioritized distance matrix between template states and current state
    n=[];
    for i=1:nci
        n=[n norm(D(:,i))];
    end    
    eps=1e-12; %regularization constant
    c=(1./(n+eps)/sum(1./(n+eps)))'; 

elseif CvxCmb==3
    
    %LI DISTANCE WEIGHTING INCLUDING PRIORIZATION MATRIX P  
    D= P*S-repmat(P*z,1,nci); %Prioritized distance matrix between template states and current state
    n=[];
    for i=1:nci
        n=[n max(abs((D(:,i))))];
    end    
    eps=1e-12; %regularization constant
    c=(1./(n+eps)/sum(1./(n+eps)))'; 
    
elseif CvxCmb==4
    
    %CONVEX COMBINATON VIA PROJECTON ON THE CONVEX HULL OF THE TEMPLATE STATES INCLUDING PRIORIZATION MATRIX P  
    H=S'*P'*P*S; H=(H+H')/2;
    g=-z'*P'*P*S;
    lb=zeros(nci,1);
    ub=ones(nci,1);
    Aeq=ones(1,nci);
    beq=1;
    [cQP, fval, exitflag] = quadprog(H,g,[],[],Aeq,beq,lb,ub,[],QPR_options);
    
    if (exitflag ~= 1)
        if (exitflag == 0)
            disp('Warning maximum number of iterations exceeded in QP');
            else
        error('No valid solution for QP found');
        end
    end

    v=P*S*cQP; %point projected onto the scaled convex hull

    D= P*S-repmat(P*z,1,nci); %Prioritized distance matrix between template states and current state
    n=[];
    for i=1:nci
        n=[n norm(D(:,i))];
    end    
    
    eps=1e-12; %regularization constant
    f= n/(sum(n)+eps);  %priorization vector
    Aeq=[P*S; ones(1,nci)];
    beq=[v; 1];
    lb=zeros(nci,1);
    ub=ones(nci,1);
    [c, fval, exitflag] = linprog(f,[],[],Aeq,beq,lb,ub,[],QPR_options);
 
    if (exitflag ~= 1)
        error('No valid solution for LP found');
    end
    
elseif CvxCmb==5    
    %convex combination of the template state solving a single QP only, including
%               minimization -f'*c of a distance weighted vector f  
    
    D= P*S-repmat(P*z,1,nci); %Prioritized distance matrix between template states and current state
    n=[];
    for i=1:nci
        n=[n norm(D(:,i))];
    end    
    f=n*lmbd; %priorization vector    

    %f=n/min(n);
    H=S'*P*S; H=(H+H')/2;
    g=(-z'*P*S+f)';
    lb=zeros(nci,1);
    ub=ones(nci,1);
    Aeq=ones(1,nci);
    beq=1;

    %[c, fval, exitflag] = quadprog(H,g',[],[],Aeq,beq,lb,ub,[],QPR_options);
    [T,c,fval,exitflag,iter,lambda] = evalc('qpOASES(H,g,Aeq,lb,ub,beq,beq,[],QPO_options)');
    
    if (exitflag ~= 0)
        if (exitflag == 1)
            disp('Warning maximum number of iterations exceeded in QP');
            else
        error('No valid solution for QP found');
        end
    end
else
    error('Invalid convex combination method specified!');
end

return

%QPO_options = qpOASES_options( 'printLevel',0);

%PROJECT ON THE CONVEX HULL
% %QPOASES
% H=S'*P*S;
% g=-(z'*P*S)';
% A=ones(1,nci);
% lbA=1;
% ubA=1;
% lb=zeros(nci,1);
% ub=ones(nci,1);
% [cQP,fval,exitflag,iter,lambda] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],QPO_options);
% v=S*cQP;

% %maximize l2 norm of the coefficient vector
% H=-eye(nci);
% g=zeros(nci,1);
% A=[S; ones(1,nci)];
% lbA=[-v; -1];
% ubA=[-v; -1];
% lb=-ones(nci,1);
% ub=zeros(nci,1);
% [cQP,fval,exitflag,iter,lambda] = qpOASES( H,g,A,lb,ub,lbA,ubA );
% cl2=-c;

%FIND THE CLOSEST POINT TO v IN S according to a weighted norm


%RESOLVE THE REDUNDANCY FOR THE COEFFICIENTS VIA PRIORITIZING THE CLOSEST ONE

%LINPROG
%f=zeros(nci,1); f(ind_min)=-1; %only prioritize the closest point       







%%QPOASES
% H=zeros(nci,nci);
% g=zeros(nci,1); f(ind_min)=-1; %priorization vector       
% A=[P*S; ones(1,nci)];
% lbA=[v; 1];
% ubA=[v; 1];
% lb=zeros(nci,1);
% ub=ones(nci,1);
% [c,fval,exitflag,iter,lambda] = qpOASES( H,g,A,lb,ub,lbA,ubA,[],options);



%%%HAAAAAAAAAAAAAAAACK

% %MINIMIZE L2 OF c
% H=eye(nci);
% %g=zeros(nci,1);

% Aeq=[P*S; ones(1,nci)];
% beq=[v; 1];
% lb=zeros(nci,1);
% ub=ones(nci,1);



% [cl2, fval, exitflag] = quadprog(H,[],[],[],Aeq,beq,lb,ub,[],QPR_options);

% if max(abs(c-cl2)) > 0.1
%     c'
%     cl2'
% end    
    
% c=cl2;

%ONLY POSITION




%%%END HAAAAAAAAAAAAACK 