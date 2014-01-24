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
DOFs{1}.x0=[-0.65;0];
DOFs{2}.x0=[-1;0];

% [gma,fval,exitflag,output,lmbd] = linprog(1,-Ao,bo);
% --------------------------------------------------------------------
% Parameters
% --------------------------------------------------------------------

options.Tau=1;  %DMP time scaling
options.Td=.01; %sample time
options.T=1.1; %simulation duration
options.Ps=diag([1; 1]); %state prioization matrix in the QP
options.Po=1e3; %auxiliary control input weight in the QP
options.epsilon=0.000001; %weighting coefficient for the penalty term in the QP
options.ws=5; %mpc preview window size
options.plot_step=[]; %plot step size
options.Qplot_window_size=[-1 1];
options.dQplot_window_size=[-2 2];

Obst.type='circle';
Obst.r0=[-0.5;-0.5];
Obst.r=.2;
Obst.nP=10;
options.Obstacles{1}.O=createObstacle(Obst);

%ind_DMP=2;
%h1=plotSimulations(S,ind_DMP,options);
%h2=plot2DSimulations(S,options,h2);

%%%%%%%%%%%%%%%PLOT STUFF FOR MPC JINT PAPER%%%%%%%%%%%%%%%%%%%%%
I{1}{1}.x0=[-0.55;0];
I{1}{2}.x0=[-1;0];
I{2}{1}.x0=[-0.65;0];
I{2}{2}.x0=[-1;0];
I{3}{1}.x0=[-0.75;0];
I{3}{2}.x0=[-1;0];
I{4}{1}.x0=[-0.85;0];
I{4}{2}.x0=[-1;0];

I{5}{1}.x0=[-1;0];
I{5}{2}.x0=[-0.55;0];
I{6}{1}.x0=[-1;0];
I{6}{2}.x0=[-0.65;0];
I{7}{1}.x0=[-1;0];
I{7}{2}.x0=[-0.75;0];
I{8}{1}.x0=[-1;0];
I{8}{2}.x0=[-0.85;0];
h2=figure;
for i=1:length(I)
    DOFs{1}.x0=I{i}{1}.x0;
    DOFs{2}.x0=I{i}{2}.x0;
    S=simulateCoupledDMP(DOFs,options);
    h2=plot2DSimulations(S,options,h2);
end

delete(subplot(1,3,2)); delete(subplot(1,3,3));
set( gca, 'Units', 'normalized', 'Position', [0 0.1 1 .8] );
xlabel('');ylabel('');title('');
axis([-1 1 -1 1]);
scrsz = get(0,'ScreenSize'); set(h2,'position',scrsz*0.8);
print(gcf,'/home/rkg/Data/Documents/MPC_JINT/figs/oa','-dpng','-r450');

