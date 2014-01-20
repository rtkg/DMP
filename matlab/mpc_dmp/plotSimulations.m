function h=plotSimulations(S,ind_DMP,options)

plot_step=options.plot_step;
if (ind_DMP > length(S{1}{1}.z/2))
    error('Given DMP index to large - cannot plot!');
end
if options.plot_step > length(S)
   error('Plot step size has to be smaller than the number of state elements!');
elseif isempty(plot_step)
   plot_step=length(S); %if no step size is given plot the complete simulation at once
end

%set plot properties
 h=figure; figure(h); scrsz = get(0,'ScreenSize'); set(h,'position',scrsz*0.76);

subplot(1,3,1);
title('Position vs. time');
xlabel('t');ylabel('q');
axis([0 S{end}{end}.t options.Qplot_window_size]);
grid on; hold on;
subplot(1,3,2);
title('Velocity vs. time');
xlabel('t');ylabel('dq');
axis([0 S{end}{end}.t options.dQplot_window_size]);
grid on; hold on;
subplot(1,3,3);
title('Position vs. Velocity');
xlabel('q');ylabel('dq');
axis([options.Qplot_window_size options.dQplot_window_size]);
axis equal;
grid on; hold on;

ind=1:length(S); %index list
complete=0;
    t=[]; Q=[]; dQ=[]; D=[]; dD=[];  h_ptpQ=[];h_ptpdQ=[];h_pQpdQ=[];
while ~complete

    for i=1:plot_step
         %clean up
        subplot(1,3,1); delete(h_ptpQ); h_ptpQ=[];
        subplot(1,3,2); delete(h_ptpdQ); h_ptpdQ=[];
        subplot(1,3,3); delete(h_pQpdQ); h_pQpdQ=[];
        
        %Extract the stuff to be plotted
        [tk Qk dQk Dk dDk]=extractStates(S{ind(1)}{1},ind_DMP);
        t=[t;tk];
        Q=[Q; Qk]; dQ=[dQ; dQk];
        D=[D; Dk]; dD=[dD; dDk];


        
        %Extract the currently predicted states             
        pt=[]; pQ=[]; pdQ=[];
        for j=2:length(S{ind(1)})
            [ptk pQk pdQk]=extractStates(S{ind(1)}{j},ind_DMP);
            pt=[pt;ptk];
            pQ=[pQ; pQk]; pdQ=[pdQ; pdQk];
        end
        
        ind(1)=[]; %pop index list   
        if isempty(ind), complete=1; break; end
    end
figure(h);
%plot the encoded demonstrated states
subplot(1,3,1);
for i=1:size(D,2)
plot(t,D(:,i),'Color',[1 .6 .6],'LineWidth',2);
end
subplot(1,3,2);
for i=1:size(D,2)
plot(t,dD(:,i),'Color',[1 .6 .6],'LineWidth',2);
end
subplot(1,3,3);
for i=1:size(D,2)
plot(D(:,i),dD(:,i),'Color',[1 .6 .6],'LineWidth',2);
end

%plot the Controller states    
subplot(1,3,1);
plot(t,Q,'k'); 
subplot(1,3,2);
plot(t,dQ,'k'); 
subplot(1,3,3);
plot(Q,dQ,'k'); 

%plot the predicted controller states
subplot(1,3,1);
h_ptpQ=plot(pt,pQ,'ro','MarkerSize',3,'MarkerFaceColor','r'); 
subplot(1,3,2);
h_ptpdQ=plot(pt,pdQ,'ro','MarkerSize',3,'MarkerFaceColor','r'); 
subplot(1,3,3);
h_pQpdQ=plot(pQ,pdQ,'ro','MarkerSize',3,'MarkerFaceColor','r'); 

drawnow;
%if ~complete, keyboard; end

%clear all but the last states to guarantee overlap for plotting in the next step
        t(1:end-1)=[]; Q(1:end-1)=[]; dQ(1:end-1)=[];
        D(1:end-1,:)=[]; dD(1:end-1,:)=[]; 
end

% %EOF