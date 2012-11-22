function [lb ub]=bounds(nBF,nD,w_max,r_min,g_min,norm)
%bounds for
%p=[a_1,b_1,w_11, ..., w_1n,...,a_nD,b_nD,w_nD1, ..., w_nDn, r_1,c_1, ... , r_n,c_n]

r_max=sqrt(-1/2/log(g_min));

%RELEVANT INDEX SETS
for i=1:nD
    ind_a{i}=(2+nBF)*(i-1)+1;
    ind_b{i}=(2+nBF)*(i-1)+2;
    ind_w{i}=i*(2+nBF)-nBF+1:1:i*(2+nBF);
    ind_r{i}=nD*(nBF+2)+1:2:2*nD+nBF*(2+nD)-1;
    ind_c{i}=nD*(nBF+2)+2:2:2*nD+nBF*(2+nD);
end

%UPPER BOUNDS p <= ub
ub=zeros(2*nD+nD*nBF+2*nBF,1);
for i=1:nD
  ub(ind_a{i})=-2;
  ub(ind_b{i})=-2;
  ub(ind_w{i})=w_max;
  ub(ind_r{i})=r_max;
  ub(ind_c{i})=1;
end

%LOWER BOUNDS p >= lb
lb=zeros(2*nD+nD*nBF+2*nBF,1);
for i=1:nD
  lb(ind_a{i})=-w_max;
  lb(ind_b{i})=-w_max;
  lb(ind_w{i})=-w_max;
  lb(ind_r{i})=r_min;
  lb(ind_c{i})=0;
end

if norm==2
lb(end+1)=-Inf; ub(end+1)=Inf;    
end