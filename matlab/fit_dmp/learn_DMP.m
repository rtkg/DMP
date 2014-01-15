clear all;clc;cla; close all;
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
param.Td=1e-2;
param.e=1e-5; 
%a=-log(e)^2 
%b=-sqrt(4*log(e)^2))

% --------------------------------------------------------------------
% load data
% --------------------------------------------------------------------

dir = '../../demonstrations/';

load(strcat(dir,'tripod.mat'));
load(strcat(dir,'parallel_extension.mat'));
load(strcat(dir,'palmar_pinch.mat'));
load(strcat(dir,'open_hand.mat'));
load(strcat(dir,'close_hand.mat'));
load(strcat(dir,'large_diameter.mat'));
load(strcat(dir,'small_diameter.mat'));
load(strcat(dir,'power_sphere.mat'));
load(strcat(dir,'precision_sphere.mat'));
load(strcat(dir,'lateral.mat'));
load(strcat(dir,'inferior_pincer.mat'));
load(strcat(dir,'minjerk.mat'));

data=minjerk;

for i=1:length(data)
    PC{i}.joint=data{i}.joint;
    for j=1:length(data{i}.motion)
        PC{i}.DMP{j}=computeDMPs(data{i}.motion{j},data{i}.joint,param);
    end
end

minjerk_traj=PC;
save('../controllers/minjerk_traj','minjerk_traj');
