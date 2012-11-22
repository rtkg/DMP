function data=processData(raw_data,options)

nM=length(raw_data); %number of motions

%find joints: it is assumed that all files contain the same joints!!
traj=importdata(raw_data{1}.file{1});
remain=traj.textdata{1};
count=1;
while true
    [str, remain] = strtok(remain, '#');
    if strcmp(str,'time'), continue; end
    if isempty(str),  break;  end
    data{count}.joint=str;
    for i=1:nM
        data{count}.motion{i}.id=[];
        data{count}.motion{i}.name=[];
        data{count}.motion{i}.demos=[];
    end
    joints{count}=str;
    count=count+1;
end
nJ=length(data);



for i=1:nM
    nD=length(raw_data{i}.file); %number of demonstrations for the current motion
    
    %load the demos from the files and segment them
    for j=1:nD

        demos=segmentTrajectories(raw_data{i}.file{j},joints,options);
        for k=1:nJ
            data{k}.motion{i}.id=raw_data{i}.id;
            data{k}.motion{i}.name=raw_data{i}.name;
            data{k}.motion{i}.demos{end+1}=demos{k};
        end    
    end      
       
    % traj=importdata(file);

    % remain=traj.textdata{1};
    % count=1;
    % while true
    %     [str, remain] = strtok(remain, '#');
    %     if isempty(str),  break;  end
    %     count=count+1;
        
        
    % end
end    





% % loads demonstration data from the given file, resamples it with the
% % sampling time given in Td and performs a two-stepped linear reggression
% % smoothing (first on the position, then on the velocity)

% %data = loadData(file,joint_names);
% N=numel(joint_names);
% data=importdata(file);

% t_=data.data(:,1)*1e-9; t_=t_-t_(1);
% t_=0:mean(diff(t_)):t_(end);%average the sample time to ensure distinct values of t

% t=t_(1):Td:t_(end);

% y=zeros(length(t),N); yd=y; ydd=y;
% for i=2:N+1
%     %   %Interpolate data at Td timesteps and do a gaussian smoothing
%     %   y_=interp1(t_,data.data(:,i),t,'linear');
%     %   y_=[ones(1,smooth_window)*y_(1),y_,ones(1,smooth_window)*y_(end)]; %append enough values at front and end to compensate for the filter
%     %   y_=smoothts(y_,'g',smooth_window,s_dev);
%     %   y(:,i-1)=y_(smooth_window+1:smooth_window+length(t) ); %cutoff the dummy values

%     %LINEAR REGRESSION FILTERING
%     %smooth the position values and differentiate to get velocity
%     y(:,i-1)=smooth(interp1(t_,data.data(:,i),t,'linear'),smooth_window,'lowess');
%     yd(1:end-1,i-1)=diff(y(:,i-1))/Td; yd(end,i-1)=yd(end-1,i-1);
   
%     %make sure that the velocity is 0 at the beginning and the end
%     t_vel =t_(find(abs(diff(data.data(:,i))) >= 5e-2*max(diff(data.data(:,i)))));%check velocity on/offset on the raw data to avoid errors due to smoothing
%     id_vel = find((t >= t_vel(1)) & (t<= t_vel(end)));
%     yd(1:id_vel(1),i-1)=0;
%     yd(id_vel(end):end,i-1)=0;   
    
%     %smooth the velocity with a larger window 
%     yd(:,i-1)=smooth(yd(:,i-1),smooth_window*2,'lowess');
    
%     %integrate to get the final postion
%     for j=2:length(t)
%         y(j,i-1)=y(j-1,i-1)+yd(j-1,i-1)*Td;
%     end

%     %differentiate to get the acceleration
%     ydd(1:end-1,i-1)=diff(yd(:,i-1))/Td; ydd(end,i-1)=ydd(end-1,i-1);
% end

% %parse the file path to get the file name
% remains=file;
% while ~isempty(remains)
%     [name remains] = strtok(remains, '/');
% end
% [name remains] = strtok(name, '.');

% for i=1:N
%     joint.name=joint_names(i);
%     joint.y=y(:,i);
%     joint.yd=yd(:,i);
%     joint.ydd=ydd(:,i);
%     joint.y_input=data.data(:,i+1);
%     joints(i)=joint;
% end

% trajectories.name=name;
% trajectories.Td=Td;
% trajectories.t=t;
% trajectories.t_input=t_;
% trajectories.joints=joints;
% trajectories.B='C'; %basis is set to configurations space

% if plot_flag
%     for i=1:N
%         figure(1);
%         plot(trajectories.t,trajectories.joints(i).y,'b');
%         grid on; hold on;
%         plot(t_,data(:,i+1),'r');
%         title(strcat(trajectories.joints(i).name,' position'));
%         legend('smoothened data','input data');
%         figure(2);
%         plot(trajectories.t,trajectories.joints(i).yd,'m');
%         grid on;
%         title(strcat(trajectories.joints(i).name,' velocity'));
%         figure(3);
%         plot(trajectories.t,trajectories.joints(i).ydd,'g');
%         grid on;
%         title(strcat(trajectories.joints(i).name,' acceleration'));
%         keyboard;
%         close all;
%     end
% end









