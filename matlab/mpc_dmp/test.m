clear all;clc;cla; close all;

% --------------------------------------------------------------------
% load DMPs computed with following parameters:
% --------------------------------------------------------------------

load '../controllers/tripod_grasp.mat';
DMP=tripod_grasp{10}.DMP{1};


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
options.kappa=0.000001; %weighting coefficient for the penalty term in the QP
options.W=1; %mpc preview window size
options.ExactDisc=0; %if one, exact discretization is used, otherwise the integral evaluates to
                     %x[k+1]=A_*x[k] + inv(A)*(A_-eye(2))*B*u[k] assuming constant u[k] between timesteps
options.mpc=0; %if one, use mpc controller, otherwise the cc coefficients are formed only
              %considering the current projection according to the ICRA13 paper


x0=[.85;0]; %initial position and velocity
[t,s,X,Lmbd]=simulateDMP(DMP,x0,options);
h=plotSimulations(DMP,t,s,X,Lmbd,options);

for i=1:size(X,2)
   dX(:,i)=diff(X(:,i))/options.Td;
end 
dX(end+1,:)=dX(end,:);


figure;
plot3(dX(:,1),dX(:,2),t,'k--'); grid on; hold on;
plot3(dX(:,3),dX(:,4),t,'r');
plot3(dX(:,5),dX(:,6),t,'r');
plot3(dX(:,7),dX(:,8),t,'r');
plot3(dX(:,9),dX(:,10),t,'r');






plot(t,dq);