clear;clc;cla; close all;

% --------------------------------------------------------------------
% load data
% --------------------------------------------------------------------
Td = 1e-2; %Trajectories are resampled with Td (after normalizing w.r.t time)

data_path = '/home/rkg/ros/aass_teleop/demonstration_logger/data/';

options.Td=1e-2;
options.smooth_window=25;
options.app=3; %number of values appended to q in order to ensure 0 vel/acc at the end of the motion
options.plot_smoothing=1;
options.p_thresh=0.05;
options.units=1; %converts from deg to rad if set to 1

% raw_data{1}.id=14;
% raw_data{1}.name='tripod_grasp';
% raw_data{1}.file{1}=strcat(data_path,'tripod_small_open.txt');
% raw_data{1}.file{2}=strcat(data_path,'tripod_small_closed.txt');
% raw_data{1}.file{3}=strcat(data_path,'tripod_large_open.txt');
% raw_data{1}.file{4}=strcat(data_path,'tripod_large_closed.txt');
% tripod=processData(raw_data,options);

% raw_data{1}.id=22;
% raw_data{1}.name='parallell_extension_grasp';
% raw_data{1}.file{1}=strcat(data_path,'parallel_extension_3_closed.txt');
% raw_data{1}.file{2}=strcat(data_path,'parallel_extension_3_open.txt');
% raw_data{1}.file{3}=strcat(data_path,'parallel_extension_6_open.txt');
% raw_data{1}.file{4}=strcat(data_path,'parallel_extension_6_closed.txt');
% parallel_extension=processData(raw_data,options);

% raw_data{1}.id=9;
% raw_data{1}.name='palmar_pinch_grasp';
% raw_data{1}.file{1}=strcat(data_path,'palmar_pinch_3_closed.txt');
% raw_data{1}.file{2}=strcat(data_path,'palmar_pinch_3_open.txt');
% raw_data{1}.file{3}=strcat(data_path,'palmar_pinch_6_open.txt');
% raw_data{1}.file{4}=strcat(data_path,'palmar_pinch_6_closed.txt');
% palmar_pinch=processData(raw_data,options);

% raw_data{1}.id=900;
% raw_data{1}.name='open_hand_motion';
% raw_data{1}.file{1}=strcat(data_path,'open_hand.txt');
% open_hand=processData(raw_data,options);

% raw_data{1}.id=901;
% raw_data{1}.name='close_hand_motion';
% raw_data{1}.file{1}=strcat(data_path,'close_hand.txt');
% close_hand=processData(raw_data,options);

% raw_data{1}.id=1;
% raw_data{1}.name='large_diameter_grasp';
% raw_data{1}.file{1}=strcat(data_path,'large_diameter_8_closed.txt');
% raw_data{1}.file{2}=strcat(data_path,'large_diameter_8_open.txt');
% large_diameter=processData(raw_data,options);

% raw_data{1}.id=2;
% raw_data{1}.name='small_diameter_grasp';
% raw_data{1}.file{1}=strcat(data_path,'small_diameter_3_closed.txt');
% raw_data{1}.file{2}=strcat(data_path,'small_diameter_3_open.txt');
% small_diameter=processData(raw_data,options);

% raw_data{1}.id=11;
% raw_data{1}.name='power_sphere_grasp';
% raw_data{1}.file{1}=strcat(data_path,'power_sphere_9_closed.txt');
% raw_data{1}.file{2}=strcat(data_path,'power_sphere_9_open.txt');
% power_sphere=processData(raw_data,options);

% raw_data{1}.id=13;
% raw_data{1}.name='precision_sphere_grasp';
% raw_data{1}.file{1}=strcat(data_path,'precision_sphere_5_closed.txt');
% raw_data{1}.file{2}=strcat(data_path,'precision_sphere_5_open.txt');
% precision_sphere=processData(raw_data,options);

% raw_data{1}.id=16;
% raw_data{1}.name='lateral_grasp';
% raw_data{1}.file{1}=strcat(data_path,'lateral_1_closed.txt');
% raw_data{1}.file{2}=strcat(data_path,'lateral_1_open.txt');
% lateral=processData(raw_data,options);

raw_data{1}.id=19;
raw_data{1}.name='inferior_pincer_grasp';
raw_data{1}.file{1}=strcat(data_path,'inferior_pincer_5_closed.txt');
raw_data{1}.file{2}=strcat(data_path,'inferior_pincer_5_open.txt');
inferior_pincer=processData(raw_data,options);

save('../../demonstrations/inferior_pincer.mat','inferior_pincer');

