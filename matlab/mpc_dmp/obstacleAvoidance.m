function aC=obstacleAvoidance(W,options)

aC{1}.Ha=[]; aC{1}.ba=[];
L=length(W{1}.z)/2;
nS=length(W);
eps=1e-5; 

%Form appropriate selection matrices to pick position/velocity vector from the state vector zeta
Sp=kron(eye(L),[1 0]);
Sv=kron(eye(L),[0 1]);

for i=1:length(options.Obstacles)
          H=-options.Obstacles{i}.O.H(:,1:2); %Obstacle normal vectors HARDCODED FOR 2D!!!!
        b=options.Obstacles{i}.O.H(:,end); %Obstacle distances 

    for j=1:nS
        %solve linear program to see if the current velocity ray intersects the obstacle
        xk=Sp*W{j}.z; %position at j
        xdk=Sv*W{j}.z; %velocity at j

        % xk=[-.9;-.9];
        % xdk=[0.15;0.28];
 
        [gma,lmbd,exitflag] =qld(0, H*xdk, 1,b+H*xk, 0, 1e6, 0, 1);
        
        % if exitflag==0
        %     dir=gma;
        % else
        %     dir=1
        % end    
        % plot(options.Obstacles{i}.O); axis equal; hold on;axis([-1.5 1.5 -1.5 1.5]);
        % plot(xk(1),xk(2),'ms');
        % plot([xk(1) xk(1)+dir*xdk(1)], [xk(2) xk(2)+dir*xdk(2)],'g');
        % keyboard
        % close all;
    
        if (exitflag==0)
            %velocity ray intersects the obstacle -> add active constraint 
            ac_ind=find(lmbd); ac_ind=ac_ind(1);
            if ac_ind > size(H,1) 

                 error('Wtf: state inside the obstacle - this should not happen!');
            end
            aC{i}.Ha=[aC{i}.Ha; H(ac_ind,:)];
            aC{i}.ba=[aC{i}.ba; b(ac_ind)*(1+eps)]; 
        else
            %velocity ray doesn't intersect obstacle -> add zeros
            aC{i}.Ha=[aC{i}.Ha;zeros(1,2)];
            aC{i}.ba=[aC{i}.ba; 0];
        end

    end
end

%EOF
