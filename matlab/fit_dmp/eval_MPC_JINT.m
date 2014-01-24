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
param.kernel='optimized'; % 'fixed' or 'optimized'
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


%first 3 -> 4 demos, other 6 -> 2demos
data{1}=tripod;
data{2}=parallel_extension;
data{3}=palmar_pinch;
data{4}=large_diameter;
data{5}=small_diameter;
data{6}=power_sphere;
data{7}=precision_sphere;
data{8}=lateral;
data{9}=inferior_pincer;
nD=length(data);

%BF=[3 5 7 9 11 13 15];
BF=7;
for k=1:length(BF)
    param.nBF = BF(k);
    K{k}.nBF = BF(k);
    for d=1:nD
        for i=1:length(data{d})
            K{k}.C{d}.PC{i}.joint=data{d}{i}.joint;
            for j=1:length(data{d}{i}.motion)
                K{k}.C{d}.PC{i}.DMP{j}=computeDMPs(data{d}{i}.motion{j},data{d}{i}.joint,param);

            end
        end
    end
end

%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  load 'BF3_5_7_9_11_13_15_fixed.mat';
%  load 'BF3_5_7_9_11_13_15_optimized.mat';
%  K=BF3_5_7_9_11_13_15_optimized;


 [Essp Essv T]=segment_MPC_JINT(K);
% BF_v=[3 5 7 9 11 13 15];
% ind=1:length(K);

% subplot(1,2,1);
% plot(BF_v(ind),mean(Essp(:,ind)),'bo-','MarkerFaceColor','b'); grid on; hold on;
% set(gca,'XTick',BF_v(ind));
% title('Mean Summed Square Errors Position');
% xlabel('nBF');
% ylabel('mean(Essp)');
% legend('h_i, c_i fixed');
% subplot(1,2,2);
% plot(BF_v(ind),mean(Essv(:,ind)),'bo-','MarkerFaceColor','b'); grid on; hold on;
% set(gca,'XTick',BF_v(ind));
% title('Mean Summed Square Errors Velocities');
% xlabel('nBF');
% ylabel('mean(Essv)');
% legend('h_i, c_i fixed');



% % xlabel('$\mathcal{G}^P \hspace{25mm} \mathcal{G}^E \hspace{25mm} \mathcal{G}^M$','interpreter', ...
% %        'latex','fontsize',15);
% % ylabel('$|\mathcal{G}^*|$','interpreter','latex','fontsize',16);
% % print(gcf,'../figs/plan_results_single','-dpng','-r450');