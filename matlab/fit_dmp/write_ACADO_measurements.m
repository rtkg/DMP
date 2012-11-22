function write_ACADO_measurements(time,joints,filename)

fileID = fopen(filename{1},'w','n');


% [M N]=size(trajectories);
% 
% fprintf(fileID,'#Timestep\n%3.2f\n#Dimensions\n%d% d\n#Joint trajectories\n'...
%     ,Td,M,N);
% 
fclose(fileID);

measurements(:,1)=time(:); %write time in the first column

for i=1:numel(joints)
   measurements(:,end+1)=joints(i).y(:);
   measurements(:,end+1)=joints(i).yd(:);
   measurements(:,end+1)=joints(i).ydd(:);
end


dlmwrite(filename{1},measurements,'-append','delimiter',' ','precision',6,'newline','pc');

