clear all;clc;cla; close all;

% --------------------------------------------------------------------
% load DMPs computed with following parameters:
% --------------------------------------------------------------------

load '../controllers/tripod_grasp.mat';
DMP=tripod_grasp{7}.DMP{1};


% --------------------------------------------------------------------
% Parameters
% --------------------------------------------------------------------

options.AbsTol=1e-6;
options.RelTol=1e-4;
options.PauseSimTime=[]; %time at which to stop the simulation plot
options.PauseSimStep=[]; %timestep used to plot after stopping
options.Tau=1;  %duration of a motion
options.Td=.01; %sample time
options.P=eye(2); P(1,1)=100; %prioization matrix in the QP
options.kappa=1e-6; %weighting coefficient for the penalty term in the QP
options.W=10; %mpc preview window size
options.ExactDisc=0; %if one, exact discretization is used, otherwise the integral evaluates to
                     %x[k+1]=A_*x[k] + inv(A)*(A_-eye(2))*B*u[k] assuming constant u[k] between timesteps
options.mpc=1; %if one, use mpc controller, otherwise the cc coefficients are formed only
              %considering the current projection according to the ICRA13 paper


x0=[-.5;0]; %initial position and velocity
[t,s,X,Lmbd]=simulateDMP(DMP,x0,options);
h=plotSimulations(DMP,t,s,X,Lmbd,options);
