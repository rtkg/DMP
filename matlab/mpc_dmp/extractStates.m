function [tk Qk dQk Dk dDk]=extractStates(S,ind_DMP)
tk=S.t;

Xk=S.z((ind_DMP-1)*2+1:ind_DMP*2);  
Qk=Xk(1);
dQk=Xk(2);

DXk=S.D((ind_DMP-1)*2+1:ind_DMP*2,:);
Dk=DXk(1,:);
dDk=DXk(2,:);

