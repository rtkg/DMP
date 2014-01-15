clear all; close all; clc;

%Raster definition
N=6;
x0=linspace(-1,1,N); dx0=0; ddx0=0;
xT=0; dxT=0; ddxT=0;

T=1;
Td=0.01;

minjerk{1}.joint='x'; minjerk{2}.joint='y';
   minjerk{1}.motion{1}.id=1; minjerk{2}.motion{1}.id=1;
      minjerk{1}.motion{1}.name='minimum_jerk'; minjerk{2}.motion{1}.name='minimum_jerk';
%generate minimum jerk trajectories
for i=1:N
   [X t]=minJerkTraj(x0(i),dx0,ddx0,xT,dxT,ddxT,T,Td);
   minjerk{1}.motion{1}.demos{i}.D=[t' X'];
   minjerk{2}.motion{1}.demos{i}.D=[t' X'];
end

save('../../demonstrations/minjerk','minjerk');

%%%EOF

