function h=plot2DSimulations(S,options)

plot_step=options.plot_step;
L=length(S{1}{1}.z)/2;

if (L ~= 2)
    error('Only 2D coupling implemented right now - cannot plot!');
end
if options.plot_step > length(S)
    error('Plot step size has to be smaller than the number of state elements!');
elseif isempty(plot_step)
    plot_step=length(S); %if no step size is given plot the complete simulation at once
end

%set plot properties
h=figure; figure(h); scrsz = get(0,'ScreenSize'); set(h,'position',scrsz*0.76);
subplot(1,3,1);
title('Positions');
xlabel('q_1');ylabel('q_2');
axis([options.Qplot_window_size options.Qplot_window_size]);
%axis equal;
grid on; hold on;
subplot(1,3,2);
title('Velocities');
xlabel('dq_1');ylabel('dq_2');
axis([options.dQplot_window_size options.dQplot_window_size]);
%axis equal;
grid on; hold on;
subplot(1,3,3);
title('Auxiliary control inputs');
xlabel('t');ylabel('lmbd_oi');
%axis([options.dQplot_window_size options.dQplot_window_size]);
grid on; hold on;

ind=1:length(S); %index list
complete=0;
for l=1:L
    t{l}=[]; Q{l}=[]; dQ{l}=[]; D{l}=[]; dD{l}=[];
end    
h_pQ1pQ2=[];  h_pdQ1pdQ2=[]; ldo=[];

while ~complete
    for i=1:plot_step
        %clean up
        subplot(1,3,1); delete(h_pQ1pQ2);  h_pQ1pQ2=[];
        subplot(1,3,2); delete(h_pdQ1pdQ2); h_pdQ1pdQ2=[];
        
        %extract the auxiliary control inputs at time k
        ldo=[ldo [S{ind(1)}{1}.mu(size(D,2)+1); S{ind(1)}{1}.mu(end)]]; %HAAACKKK! works only for
                                                                        %L==2!!!! (should do it
                                                                        %in the loop below)
        
        for l=1:L
            %Extract the stuff to be plotted

            [tk Qk dQk Dk dDk]=extractStates(S{ind(1)}{1},l);

            t{l}=[t{l};tk];
            Q{l}=[Q{l}; Qk]; dQ{l}=[dQ{l}; dQk];
            D{l}=[D{l}; Dk]; dD{l}=[dD{l}; dDk];
            
            %Extract the currently predicted states             
            pt{l}=[]; pQ{l}=[]; pdQ{l}=[];
            for j=2:length(S{ind(1)})
                [ptk pQk pdQk]=extractStates(S{ind(1)}{j},l);
                pt{l}=[pt{l};ptk];
                pQ{l}=[pQ{l}; pQk]; pdQ{l}=[pdQ{l}; pdQk];
            end
        end  
        ind(1)=[]; %pop index list   
        if isempty(ind), complete=1; break; end
    end
    
    %plot the encoded demonstrated states
    N=size(D{1},2);
    c{1}=[ones(1,N) N*ones(1,N) 2:N-1 2:N-1];
    c{2}=[[1:N] [1:N] N*ones(1,N-2) ones(1,N-2)];
    for i=1:length(c{1})
        subplot(1,3,1);
        plot(D{1}(:,c{1}(i)),D{2}(:,c{2}(i)),'Color',[1 .6 .6],'LineWidth',2);
        subplot(1,3,2);
        plot(dD{1}(:,c{1}(i)),dD{2}(:,c{2}(i)),'Color',[1 .6 .6],'LineWidth',2);
    end
    
    %plot the Controller states    
    subplot(1,3,1);
    plot(Q{1},Q{2},'k'); 
    subplot(1,3,2);
    plot(dQ{1},dQ{2},'k');
    
    %plot the predicted controller states
    subplot(1,3,1);
    h_pQ1pQ2=plot(pQ{1},pQ{2},'ro','MarkerSize',3,'MarkerFaceColor','r'); 
    subplot(1,3,2);
    h_pdQ1pdQ2=plot(pdQ{1},pdQ{2},'ro','MarkerSize',3,'MarkerFaceColor','r'); 

    %plot constraints
    for j=1:length(options.Constraints)
        % keyboard
        % options.Constraints{j}.active
        %check if the constraint is active
        %if .....
        %else
        %  continue;
        %end
        
        N=options.Constraints{j}.N;
        b=options.Constraints{j}.b;
        
        %find 2 points outside the bounding box
        L=zeros(2,2);
        if abs(N(2))>0.01
            L(1,1)=max([options.Qplot_window_size(2) options.dQplot_window_size(2)]);
            L(1,2)=min([options.Qplot_window_size(1) options.dQplot_window_size(1)]);
            L(2,1:2)=(repmat(-b,1,2)-N(1)*L(1,1:2))/N(2);
        else    
            L(2,1)=max([options.Qplot_window_size(2) options.dQplot_window_size(2)]);
            L(2,2)=min([options.Qplot_window_size(1) options.dQplot_window_size(1)]);
            L(1,1:2)=(repmat(-b,1,2)-N(2)*L(1,1:2))/N(1);
        end

        if options.Constraints{j}.type=='p'
            subplot(1,3,1);
        elseif options.Constraints{j}.type=='v'
            subplot(1,3,2);
        else
            error('Unknown constraint type!');
        end

        plot(L(1,:),L(2,:),'b','LineWidth',2);

    end

 subplot(1,3,3);
 plot(t{1},ldo(1,:),'b'); hold on; grid on;
 plot(t{1},ldo(2,:),'r');
    
    drawnow;
    %   if ~complete, keyboard; end

    %clear all but the last states to guarantee overlap for plotting in the next step
    for l=1:L
        t{l}(1:end-1)=[]; Q{l}(1:end-1)=[]; dQ{l}(1:end-1)=[];
        D{l}(1:end-1,:)=[]; dD{l}(1:end-1,:)=[]; ldo(:,1:end-1)=[];
    end
end
