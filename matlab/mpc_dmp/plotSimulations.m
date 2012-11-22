function h=plotSimulations(DMP,t,s,X,Lmbd,options,varargin)

if nargin==7
    h=varargin{1};
else
    h=figure;
end

if ~isempty(options.PauseSimTime)
    t_pause=options.PauseSimTime;
else
    t_pause=t(end);
end
if ~isempty(options.PauseSimStep)
    step_pause=options.PauseSimStep;
else
    step_pause=options.Td;
end

D=length(DMP.param);

%FIND AXIS LIMITS
q_min=Inf;
q_max=-Inf;
dq_min=Inf;
dq_max=-Inf;
for i=1:D+1
 q_min=min(q_min,min(X(:,2*i-1)));    
 q_max=max(q_max,max(X(:,2*i-1)));    
 dq_min=min(dq_min,min(X(:,2*i)));    
 dq_max=max(dq_max,max(X(:,2*i)));    
end    

%SET TITLES, LABELS & AXIS LIMITS
figure(h); scrsz = get(0,'ScreenSize'); set(h,'position',scrsz*0.76);
subplot(2,2,1);
xlabel('t');ylabel('q');
title('Position vs time');
axis([t(1) t(end) q_min*1.1 q_max*1.1]);
subplot(2,2,2);
xlabel('t');ylabel('dq');
title('Velocity vs time');
axis([t(1) t(end) dq_min*1.1 dq_max*1.1]);
subplot(2,2,3);
xlabel('s'); ylabel('q');zlabel('dq');
title('Phase space');
axis([min(s) max(s) q_min*1.1 q_max*1.1 dq_min*1.1 dq_max*1.1]);
subplot(2,2,4); grid on;
xlabel('q'); ylabel('dq');
axis([q_min*1.1 q_max*1.1 dq_min*1.1 dq_max*1.1]);

for j=1:D
    q_j=X(1,1+2*j);
    dq_j=X(1,2+2*j);
    subplot(2,2,1);
    text(t(1),q_j,num2str(j)); grid on; hold on;
    subplot(2,2,2);
    text(t(1),dq_j,num2str(j)); grid on; hold on;
    subplot(2,2,3);
    text(t(1),q_j,dq_j,num2str(j)); grid on; hold on; 
end

%PLOTTING
h_tq=[]; h_tdq=[]; h_sqdq=[];
for i=2:length(t)
    %clean up
    subplot(2,2,4);  cla; grid on; hold on; 
    delete(h_sqdq); h_sqdq=[];
    delete(h_tdq); h_tdq=[];
    delete(h_tq); h_tq=[];
    Qd=[];

    for j=1:D
        q_t_j=X(i-1:i,1+2*j);
        dq_t_j=X(i-1:i,2+2*j);
        Qd=[Qd [q_t_j(2);dq_t_j(2)]]; %current  q, dq states of the template DMPs
        
        %plot template DMPs in t, q
        subplot(2,2,1);
        plot(t(i-1:i),q_t_j,'Color',[1 .6 .6],'LineWidth',2);
        
        %plot template DMPs in t, q
        subplot(2,2,2);
        plot(t(i-1:i),dq_t_j,'Color',[1 .6 .6],'LineWidth',2);
        
        %plot template DMPs in s, q, dq
        subplot(2,2,3);
        plot3(s(i-1:i),q_t_j,dq_t_j,'Color',[1 .6 .6],'LineWidth',2);
        
        % indicate the number of the template DMPs in the q, dq plot
        subplot(2,2,4);
        text(q_t_j(2),dq_t_j(2),num2str(j));
 
        %indicate the number of the template DMPs in the t, q plot
        subplot(2,2,1);
        h_tq(j)=text(t(i),q_t_j(2),num2str(j));
        
        %indicate the number of the template DMPs in the t, dq plot
        subplot(2,2,2);
        h_tdq(j)=text(t(i),dq_t_j(2),num2str(j));
        
        %indicate the number of the template DMPs in the s, q, dq plot
        subplot(2,2,3);
        h_sqdq(j)=text(s(i),q_t_j(2),dq_t_j(2),num2str(j));
    end

    %plot convex hull of the current template states + controller state &
    %projection of the controller state in q, dq 
    subplot(2,2,4);
    if D < 2
        K=[1 1];
    else
        K=convhulln(Qd');
    end
    v=Qd*Lmbd(i,:)'; %projection
    % if i>1
    %     vf=Qd*Lmbd(i+1,:)';
    %     vp=Qd*Lmbd(i-1,:)';
    %      plot(vf(1),vf(2),'g+','MarkerSize',10);
    %       plot(vp(1),vp(2),'m+','MarkerSize',10);
    % end
    
    plot(X(i,1),X(i,2),'ko','MarkerFaceColor','k','MarkerSize',6);
    plot(v(1),v(2),'b+','MarkerSize',10);
     text(q_min,dq_max,strcat('c: ',num2str(Lmbd(i,:))),'EdgeColor','r','FontSize',12,'FontWeight','demi');
   if size(Qd,2) < 2
        tri=[1 1 1];
    else
        tri=delaunay(Qd(1,:),Qd(2,:));
    end
    trisurf(tri,Qd(1,:),Qd(2,:),zeros(1,D),'FaceAlpha',.5,'EdgeAlpha',0,'FaceColor',[1 .6 .6]); hold on; 
     for j=1:size(K,1)
      plot3(Qd(1,K(j,:)), Qd(2,K(j,:)),[0 0],'Color',[1 .6 .6],'LineWidth',2,'LineWidth',1);  
    end 
 
   %plot controller state in t, q
    subplot(2,2,1);
    plot(t(i-1:i),X(i-1:i,1) ,'k');
    
    %plot controller state in t, dq
    subplot(2,2,2);
    plot(t(i-1:i),X(i-1:i,2) ,'k');
    
    %plot controller state in s, q, dq
    subplot(2,2,3);
    plot3(s(i-1:i),X(i-1:i,1),X(i-1:i,2) ,'k');

    %plot the convex hull of the template controllers in s, q, dq
    % subplot(2,2,3);
    % h_sqdq(end+1)=trisurf(tri,repmat(s(i),1,nD),S(1,:),S(2,:),'FaceAlpha',.5,'EdgeAlpha',0,'FaceColor','Color',[1 .6 .6],'LineWidth',2);
     drawnow('expose'); %force plot update
     
    if(t(i)>t_pause)
        keyboard
        t_pause=t_pause+step_pause;
    end  

end
%EOF