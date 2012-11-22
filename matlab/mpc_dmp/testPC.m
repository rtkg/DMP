clear all;clc;cla; close all;

% --------------------------------------------------------------------
% load PC computed with following parameters:
% --------------------------------------------------------------------

load PC;
load x_.txt; 
load x_ref_.txt;
load s_.txt; 
load ps_ref_.txt;
load lmbd_.txt; 

% --------------------------------------------------------------------
% Parameters
% --------------------------------------------------------------------

options.AbsTol=1e-6;
options.RelTol=1e-4;
options.PauseSimTime=[]; %time at which to stop the simulation plot
options.PauseSimStep=[]; %timestep used to plot after stopping
options.Tau=5;  %duration of a motion
options.Td=.01; %sample time
options.P=eye(2); options.P(1,1)=100; %prioization matrix in the QP
options.kappa=1e-6; %weighting coefficient for the penalty term in the QP
options.W=10; %mpc preview window size
options.ExactDisc=0; %if one, exact discretization is used, otherwise the integral evaluates to
                     %x[k+1]=A_*x[k] + inv(A)*(A_-eye(2))*B*u[k] assuming constant u[k] between timesteps
options.mpc=0; %if one, use mpc controller, otherwise the cc coefficients are formed only
              %considering the current projection according to the ICRA13 paper

x_(:,1)=x_(:,1)-x_(1,1); x_(:,1)=x_(:,1)*1e-9; dummy=find(x_(:,1) > options.Tau); x_(dummy,:)=[];
x_ref_(:,1)=x_ref_(:,1)-x_ref_(1,1); x_ref_(:,1)=x_ref_(:,1)*1e-9; x_ref_(dummy,:)=[];
s_(dummy)=[]; ps_ref_(dummy,:)=[]; lmbd_(dummy,:)=[];
DMP=PC{1}.DMP{1};

[t,s,X,Lmbd,ps_ref]=simulateDMP(DMP,x_(1,2:3)',options);
%h=plotSimulations(DMP,t,s,X,Lmbd,options);

nD=length(DMP.param);
for i=1:nD
    plot(s_,x_ref_(:,2*i),'k--','LineWidth',2); grid on; hold on;
    plot(s,X(:,2*i+1),'r');
end    
plot(s_,x_(:,2),'b--','LineWidth',2);
plot(s,X(:,1),'m');

