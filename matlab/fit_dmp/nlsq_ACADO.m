function p = nlsq_ACADO(d,pI,A,b,Aeq,beq,lb,ub,S)
%writes the necessary files for communicating with ACADO, runs the ACADO binary and reads the result

% -----------------
path = '~/Data/Coding/dynamical_systems/';
% -----------------

nD=length(d);
nBF = (length(pI)-2*nD)/(2+nD);
demos=[]; args=num2str(nBF);
for j=1:nD
    demos=[demos; d{j}.D]; 
    args = [args,' ',num2str(size(d{j}.D,1))];
end

pI=pI(:)';

%Write the demos to a file to be read by ACADO
fileID = fopen(strcat(path,'data/demos.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'data/demos.dat'),demos,'-append','delimiter',' ','precision',12,'newline','pc');

%Write the inital guess to a file to be read by ACADO
fileID = fopen(strcat(path,'data/initial_guess.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'data/initial_guess.dat'),pI,'-append','delimiter',' ','precision',12,'newline','pc');

%Write the inequality constraint matrix to a file to be read by ACADO
fileID = fopen(strcat(path,'data/inequality_constraint_matrix.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'data/inequality_constraint_matrix.dat'),A,'-append','delimiter',' ','precision',12,'newline','pc');

%Write the inequality constraint vector to a file to be read by ACADO
fileID = fopen(strcat(path,'data/inequality_constraint_vector.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'data/inequality_constraint_vector.dat'),b,'-append','delimiter',' ','precision',12,'newline','pc');

%Write the equality constraint matrix to a file to be read by ACADO
fileID = fopen(strcat(path,'data/equality_constraint_matrix.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'data/equality_constraint_matrix.dat'),Aeq,'-append','delimiter',' ','precision',12,'newline','pc');

%Write the equality constraint vector to a file to be read by ACADO
fileID = fopen(strcat(path,'data/equality_constraint_vector.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'data/equality_constraint_vector.dat'),beq,'-append','delimiter',' ','precision',12,'newline','pc');

%Write the scale Matrix to a file to be read by ACADO
fileID = fopen(strcat(path,'data/scale_matrix.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'data/scale_matrix.dat'),S,'-append','delimiter',' ','precision',12, ...
         'newline','pc');

%Write the upper bounds to a file to be read by ACADO
fileID = fopen(strcat(path,'data/upper_bounds.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'data/upper_bounds.dat'),ub,'-append','delimiter',' ','precision',12,'newline','pc');

%Write the lower bounds to a file to be read by ACADO
fileID = fopen(strcat(path,'data/lower_bounds.dat'),'w','n');
fclose(fileID);
dlmwrite(strcat(path,'data/lower_bounds.dat'),lb,'-append','delimiter',' ','precision',12,'newline','pc');

executable = 'nlsq_full_bounds';

disp(' ');
disp([' Execute command:  ', executable, ' ', args])
disp(' ');

out = system([strcat(path,'bin/'), executable, ' ', args]);

p=load(strcat(path,'data/solution.dat'));


%%%EOF