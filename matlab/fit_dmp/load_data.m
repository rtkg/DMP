clear;clc;cla; close all;

% --------------------------------------------------------------------
% load data
% --------------------------------------------------------------------
Td = 1e-2; %Trajectories are resampled with Td (after normalizing w.r.t time)

data_path = '/home/rkg/ros/aass_teleop/demonstration_logger/data/';

options.Td=1e-2;
options.smooth_window=25;
options.app=3; %number of values appended to q in order to ensure 0 vel/acc at the end of the motion
options.plot_smoothing=0;
options.p_thresh=0.05;
options.units=1; %converts from deg to rad if set to 1

raw_data{1}.id=14;
raw_data{1}.name='tripod_grasp';
raw_data{1}.file{1}=strcat(data_path,'tripod_small_open.txt');
raw_data{1}.file{2}=strcat(data_path,'tripod_small_closed.txt');
raw_data{1}.file{3}=strcat(data_path,'tripod_large_open.txt');
raw_data{1}.file{4}=strcat(data_path,'tripod_large_closed.txt');

 tripod=processData(raw_data,options);
 % save('../../demonstrations/tripod.mat','tripod');

%trajectories=processData(joint_names,file,Td,smooth_window,0);
%trajectories=truncateTrajectories(trajectories,v_thresh,alpha_T,0);
%trajectories=shiftToOrigin(trajectories);
% write_ACADO_measurements(trajectories.t,trajectories.joints([7]),'demonstrations/FFJ3_tripod_small_closed.txt');

% file= strcat(data_path,'tripod_small_open.txt');
% trajectories=processData(joint_names,file,Td,smooth_window,0);
% trajectories=truncateTrajectories(trajectories,v_thresh,alpha_T,0);
% trajectories=shiftToOrigin(trajectories);
% write_ACADO_measurements(trajectories.t,trajectories.joints([7]),'demonstrations/FFJ3_tripod_small_open.txt');

% file= strcat(data_path,'tripod_large_open.txt');
% trajectories=processData(joint_names,file,Td,smooth_window,0);
% trajectories=truncateTrajectories(trajectories,v_thresh,alpha_T,0);
% trajectories=shiftToOrigin(trajectories);
% write_ACADO_measurements(trajectories.t,trajectories.joints([7]),'demonstrations/FFJ3_tripod_large_open.txt');

% file= strcat(data_path,'tripod_large_closed.txt');
% trajectories=processData(joint_names,file,Td,smooth_window,0);
% trajectories=truncateTrajectories(trajectories,v_thresh,alpha_T,0);
% trajectories=shiftToOrigin(trajectories);
% write_ACADO_measurements(trajectories.t,trajectories.joints([7]),'demonstrations/FFJ3_tripod_large_closed.txt');

