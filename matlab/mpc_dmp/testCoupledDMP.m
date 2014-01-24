clear all;clc;cla; close all;

% --------------------------------------------------------------------
% load DMPs computed with following parameters:
% --------------------------------------------------------------------

load '../controllers/minjerk_traj.mat';
load '../controllers/tripod_grasp.mat';

% DOFs{1}=tripod_grasp{1}.DMP{1};
% DOFs{2}=tripod_grasp{2}.DMP{1};
DOFs{1}=minjerk_traj{1}.DMP{1};
DOFs{2}=minjerk_traj{2}.DMP{1};

%initial values
DOFs{1}.x0=[-0.45;0];
DOFs{2}.x0=[-1;0];


% --------------------------------------------------------------------
% Parameters
% --------------------------------------------------------------------

options.Tau=1;  %DMP time scaling
options.Td=.01; %sample time
options.T=1.1; %simulation duration
options.Ps=diag([1; 1]); %state prioization matrix in the QP
options.Po=1e4; %auxiliary control input weight in the QP
options.epsilon=0.000001; %weighting coefficient for the penalty term in the QP
options.ws=10; %mpc preview window size
options.plot_step=[]; %plot step size
options.Qplot_window_size=[-1.5 1.5];
options.dQplot_window_size=[-2 2];
options.Constraints{1}.N=[-sqrt(2)/2 sqrt(2)/2]; 
%options.Constraints{1}.N=[-1; 0]; 
options.Constraints{1}.b=.2; 
options.Constraints{1}.active=[1:70]; 
options.Constraints{1}.type='p'; 

S=simulateCoupledDMP(DOFs,options);

%ind_DMP=1;
%h1=plotSimulations(S,ind_DMP,options);
%h2=plot2DSimulations(S,options);

% %%%%%%%%%%%%%%%PLOT STUFF FOR MPC JINT PAPER%%%%%%%%%%%%%%%%%%%%%
for i=1:2
h(i)=plotSimulations(S,i,options);
set(0, 'currentfigure', h(i));
delete(subplot(1,3,1)); delete(subplot(1,3,3));
set( gca, 'Units', 'normalized', 'Position', [0.05 0.1 0.8 .8] );
xlabel('');ylabel('');title('');
axis([0 1 -1.01 1.01]);
ax(i)= get(h(i), 'Children');
end
ax2Children = get(ax(2),'Children');
copyobj(ax2Children, ax(1));
close(h(2));
return


for i=1:2
h(i)=plotSimulations(S,i,options);
set(0, 'currentfigure', h(i));
delete(subplot(1,3,2)); delete(subplot(1,3,3));
set( gca, 'Units', 'normalized', 'Position', [0.05 0.1 0.8 .8] );
xlabel('');ylabel('');title('');
axis([0 1 -1.01 1.01]);
ax(i)= get(h(i), 'Children');
end
ax2Children = get(ax(2),'Children');
copyobj(ax2Children, ax(1));
close(h(2));


h2=plot2DSimulations(S,options);
set(0, 'currentfigure', h2);
delete(subplot(1,3,2)); delete(subplot(1,3,3));
set( gca, 'Units', 'normalized', 'Position', [0 0.1 1 .8] );
xlabel('');ylabel('');title('');
axis equal;
axis([-1 1 -1 1]);
scrsz = get(0,'ScreenSize'); set(h2,'position',scrsz*0.8);
% print(gcf,'/home/rkg/Data/Documents/MPC_JINT/figs/oa','-dpng','-r450');