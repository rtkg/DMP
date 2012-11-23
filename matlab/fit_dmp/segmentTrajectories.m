function demos=segmentTrajectories(file,joints,options)
traj=importdata(file);
data=traj.data;

nJ=size(data,2)-1; %number of joints (first column is time)

%convert from deg to rad
if (options.units == 1)
    data(:,2:end)=data(:,2:end)*pi/180;
end    

%demo time vector
t_=data(:,1)*1e-9; t_=t_-t_(1);
Td_=mean(diff(t_));
t_=0:Td_:t_(end);%average the sample time to ensure distinct values of t

%%%%%FIND ONSET/END TIME OF THE MOTION
cmap = hsv(nJ);
cut_ind=[Inf -Inf];
figure;
for i=1:nJ
    ind_l=find((abs(diff(data(:,i+1))) > options.p_thresh),1,'first');
    ind_u=find((abs(diff(data(:,i+1))) > options.p_thresh),1,'last');
    if(ind_l < cut_ind(1)), cut_ind(1)=ind_l; end
    if(ind_u > cut_ind(2)), cut_ind(2)=ind_u; end
    plot(t_,data(:,i+1),'Color',cmap(i,:)); grid on; hold on;
    text(t_(end),data(end,i+1),joints{i},'FontSize',6);
end
h1=plot(t_(cut_ind(1)),data(cut_ind(1),2:end),'r.','MarkerSize',20);
h2=plot(t_(cut_ind(2)),data(cut_ind(2),2:end),'r.','MarkerSize',20);
title(file);
xlabel('deg'); ylabel('t');

while true
    t_o = input('Specify onset time (enter to continue): ');
    if isempty(t_o), break; end

    delete(h1);
    cut_ind(1)=find(t_ > t_o,1,'first');
    h1=plot(t_(cut_ind(1)),data(cut_ind(1),2:end),'r.','MarkerSize',20);
end
while true
    t_e = input('Specify end time (enter to continue): ');
    if isempty(t_e), break; end

    delete(h2);
    cut_ind(2)=find(t_ > t_e,1,'first');
    h2=plot(t_(cut_ind(2)),data(cut_ind(2),2:end),'r.','MarkerSize',20);
end
close;
%cut data prepend/append one line to ensure velocity is zero at beginning and end & normalize the
%time vector


data=data(cut_ind(1):cut_ind(2),:); data=[data; repmat(data(end,:),options.app,1)];
t_=t_(cut_ind(1):cut_ind(2)+options.app); t_=t_-t_(1); t_=t_/t_(end);

% data=data(cut_ind(1):cut_ind(2),:); data=[data(1,:); data(1,:); data; data(end,:); data(end,:)];
% t_=[t_(cut_ind(1))-2*Td_ t_(cut_ind(1))-Td_ t_(cut_ind(1):cut_ind(2)) t_(cut_ind(2))+Td_ ];   t_=t_-t_(1); t_=t_/t_(end);

%resampled normalized time vector
t=0:options.Td:1;
t=t(:);

for i=1:nJ
    %differentiate to get velocity and smooth it
    q=interp1(t_,data(:,i+1),t,'linear');
    dq=diff(q)/options.Td; dq(end+1)=dq(end);
    dq=[ones(options.smooth_window,1)*dq(1);dq;ones(options.smooth_window,1)*dq(end)]; %append enough values at front and end to compensate for the filter
    dq=smooth(dq,options.smooth_window,'lowess');
    dq=dq(options.smooth_window+1:options.smooth_window+length(t)); %cutoff the dummy values

    %integrate to get position
    for j=2:length(t)
        q(j)=q(j-1)+dq(j)*options.Td;
    end

    %differentiate to get acceleration
    ddq=diff(dq)/options.Td; ddq(end+1)=ddq(end);
    
    if (options.plot_smoothing )
        figure;
        subplot(3,1,1);
        plot(t_,data(:,i+1),'r'); grid on; hold on;
        plot(t,q,'k'); 
        legend('raw','smoothened');
        title(strcat(joints{i},' position'));
        
        subplot(3,1,2);
        plot(t,dq,'k'); 
        title(strcat(joints{i},' velocity'));

        subplot(3,1,3);
        plot(t,ddq,'k'); 
        title(strcat(joints{i},' acceleration'));

        keyboard;
        close;
    end
    %shift to 0
    demos{i}.D=[t, q-q(end),dq,ddq];
    demos{i}.raw=[t_', data(:,i+1)];
    demos{i}.file=file;
end


