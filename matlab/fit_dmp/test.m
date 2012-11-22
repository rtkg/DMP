clear;clc;cla; close all;
% --------------------------------------------------------------------
% parameters
% --------------------------------------------------------------------

param.nBF = 5; % number of basis functions
param.r_min=1e-1/2; %minimum gaussian width
param.w_max=1e5; %maximum weight value
param.g_min=1e-4; %maximum value of a gaussian at 1
param.cd_min=1/(param.nBF-1)/5;  %minimum distance between gaussian centers
param.Tau=1;
param.norm=1; %use L2 norm
param.Td=.01;
param.e=1e-5; %a=-log(e)^2; b=-sqrt(4*log(e)^2))

% --------------------------------------------------------------------
% load data
% --------------------------------------------------------------------

dir = '../../demonstrations/';
data{1}.joint='FFJ3';
data{1}.motion{1}.id=14;
data{1}.motion{1}.name='tripod_grasp';
data{1}.motion{1}.demos{1}.path=strcat(dir,'FFJ3_tripod_large_closed.txt');
data{1}.motion{1}.demos{2}.path=strcat(dir,'FFJ3_tripod_large_open.txt');
data{1}.motion{1}.demos{3}.path=strcat(dir,'FFJ3_tripod_small_closed.txt');
data{1}.motion{1}.demos{4}.path=strcat(dir,'FFJ3_tripod_small_open.txt');

data{2}.joint='MFJ3';
data{2}.motion{1}.id=14;
data{2}.motion{1}.name='tripod_grasp';
data{2}.motion{1}.demos{1}.path=strcat(dir,'MFJ3_tripod_large_closed.txt');
data{2}.motion{1}.demos{2}.path=strcat(dir,'MFJ3_tripod_large_open.txt');
data{2}.motion{1}.demos{3}.path=strcat(dir,'MFJ3_tripod_small_closed.txt');
data{2}.motion{1}.demos{4}.path=strcat(dir,'MFJ3_tripod_small_open.txt');

units=1; %convert from deg to rad
data=load_data(data,param.Td,units);

for i=1:length(data)
    PC{i}.joint=data{i}.joint;
    for j=1:length(data{i}.motion)
        PC{i}.DMP{j}=computeDMPs(data{i}.motion{j},param);
    end
end

kappa=1e-6;
P=eye(2);P(1,1)=100;
writeROSYaml(PC,'dof_config.yaml',kappa,P);
save('../mpc_dmp/PC','PC');
