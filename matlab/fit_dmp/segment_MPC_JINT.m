function [Essp Essv T]=segment_MPC_JINT(K)

Essp=[]; Essv=[]; T=[];
for k=1:length(K)
    t_learn=[]; sq_sum=[]; E=[];
    for d=1:length(K{k}.C)
        for i=1:length(K{k}.C{d}.PC)  
            t_learn=[t_learn;K{k}.C{d}.PC{i}.DMP{1}.t_learn];

            for j=1:length(K{k}.C{d}.PC{i}.DMP{1}.param)  
                
                E=[E; K{k}.C{d}.PC{i}.DMP{1}.param{j}.rep_err.E];
                sq_sum=[sq_sum; K{k}.C{d}.PC{i}.DMP{1}.param{j}.rep_err.sq_sum];
 if sq_sum(end,1)>1e5
      K{k}.C{d}.PC{i}
     K{k}.C{d}.PC{i}.DMP{1}
     % keyboard
 end
            end
        end
    end
    % time pos vel
    res{k}.E=E;
    res{k}.mean=[mean(t_learn) mean(sq_sum)];
    res{k}.var=[var(t_learn) var(sq_sum)];
    res{k}.median=[median(t_learn) median(sq_sum)];
    res{k}.nBF=K{k}.nBF;
    Essp=[Essp sq_sum(:,1)]; Essv=[Essv sq_sum(:,2)]; T=[T t_learn];
end

