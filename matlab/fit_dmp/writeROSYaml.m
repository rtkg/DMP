function writeROSYaml(PC,path,kappa,P,t_tol)

nJ=length(PC);

%Map containing the ROS topics for publishing/subscribing
for i=1:nJ
   topics{i}={strcat('/sh_',lower(PC{i}.joint),'_inverse_dynamics_controller/state'), ...
              strcat('/sh_',lower(PC{i}.joint),'_inverse_dynamics_controller/command')}; 
   joints{i}=PC{i}.joint;
end    
topic_map=containers.Map(joints,topics);



fid=fopen(path,'w','n');
%dlmwrite(path,'dof_config:','-append','delimiter',' ','precision',12,'newline','pc');
fprintf(fid,'%s\n','dof_config:');

for i=1:nJ;
    fprintf(fid,' - joint: %s\n',PC{i}.joint);
    topics=topic_map(PC{i}.joint);
    fprintf(fid,'   state_topic: %s\n',topics{1});
    fprintf(fid,'   command_topic: %s\n',topics{2});
    fprintf(fid,'   tracking_tolerance: %d\n',t_tol);
    fprintf(fid,'   dmp:\n');
    
    for j=1:length(PC{i}.DMP)      
        fprintf(fid,'   - name: %s\n',PC{i}.DMP{j}.name);
        fprintf(fid,'     id: %d\n',PC{i}.DMP{j}.id);
        fprintf(fid,'     nD: %d\n',length(PC{i}.DMP{j}.param));
        %assuming same number of basis functions within one DMP
        fprintf(fid,'     nBF: %d\n',length(PC{i}.DMP{j}.param{1}.pBF)/2); 
        
        %write a & b assuming they are the same within one DMP
        fprintf(fid,'     ab: [');
        fprintf(fid,'%f,',PC{i}.DMP{j}.param{1}.w(1)); 
        fprintf(fid,'%f',PC{i}.DMP{j}.param{1}.w(2)); 
        fprintf(fid,']\n');
        
        %write the basis function parameters assuming they are the same within one DMP
         fprintf(fid,'     pBF: [');
         for k=1:length(PC{i}.DMP{j}.param{1}.pBF)-1
            fprintf(fid,'%f,',PC{i}.DMP{j}.param{1}.pBF(k)); 
         end    
         fprintf(fid,'%f',PC{i}.DMP{j}.param{1}.pBF(k+1)); 
         fprintf(fid,']\n');
         
         %write the basis function parameters assuming they are the same within one DMP
         w=[]; q0_ref=[];
         for k=1:length(PC{i}.DMP{j}.param)
            w=[w PC{i}.DMP{j}.param{k}.w(3:end)'];
            q0_ref=[q0_ref PC{i}.DMP{j}.param{k}.q0_ref];
         end
      
         fprintf(fid,'     w: [');
         for k=1:length(w)-1
            fprintf(fid,'%f,',w(k)); 
         end    
   

         fprintf(fid,'%f',w(k+1)); 
         fprintf(fid,']\n');         
         
         %write priorization matrix P
         fprintf(fid,'     P: [%f,%f,%f,%f]\n',P(1,1),P(1,2),P(2,1),P(2,2));
         
         %write initial positions q0
         fprintf(fid,'     q0_ref: [');
         for k=1:length(q0_ref)-1
            fprintf(fid,'%f,',q0_ref(k)); 
         end   
         
         if isempty(k)
             k=0;
         end
         
         fprintf(fid,'%f',q0_ref(k+1)); 
         fprintf(fid,']\n');

        fprintf(fid,'     kappa: %f\n\n',kappa);
    end
end

fclose(fid);
%keyboard
