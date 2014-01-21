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
options.T=1.5; %simulation duration
options.Ps=diag([1; 1]); %state prioization matrix in the QP
options.Po=1e4; %auxiliary control input weight in the QP
options.epsilon=0.000001; %weighting coefficient for the penalty term in the QP
options.ws=10; %mpc preview window size
options.plot_step=1; %plot step size
options.Qplot_window_size=[-1.5 1.5];
options.dQplot_window_size=[-2 2];
options.Constraints{1}.N=[-sqrt(2)/2 sqrt(2)/2]; 
%options.Constraints{1}.N=[-1; 0]; 
options.Constraints{1}.b=.2; 
options.Constraints{1}.active=[1:70]; 
options.Constraints{1}.type='p'; 

S=simulateCoupledDMP(DOFs,options);
ind_DMP=2;

%h1=plotSimulations(S,ind_DMP,options);
h2=plot2DSimulations(S,options);

