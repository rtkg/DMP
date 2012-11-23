clear all;clc;cla; close all;

load '../controllers/tripod_grasp.mat'

PC=tripod_grasp;

kappa=1e-6;
P=eye(2);P(1,1)=100;
t_tol= 0.05; %tolerance for open loop control
writeROSYaml(PC,'dof_config.yaml',kappa,P,t_tol);

