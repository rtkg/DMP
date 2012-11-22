function [A b]=inequalityConstraints(nBF,nD,cd_min,g_min,norm)
%Inequality constraints (no box constraints) for
%p=[a_1,b_1,w_11, ..., w_1n,...,a_nD,b_nD,w_nD1, ..., w_nDn, r_1,c_1, ... , r_n,c_n]
%Enforces: r_n+r_max*c_n <= r_max (gaussian BF width constraint)
%          c_0 >= 0; -c_(n-1)+c_n >= cd_min (orders gaussian BF centers and enforces minimum
%          distance between centers)
%          

r_max=sqrt(-1/2/log(g_min));


%RELEVANT INDEX SETS
for i=1:nD
    ind_a{i}=(2+nBF)*(i-1)+1;
    ind_b{i}=(2+nBF)*(i-1)+2;
    ind_r{i}=nD*(nBF+2)+1:2:2*nD+nBF*(2+nD)-1;
    ind_c{i}=nD*(nBF+2)+2:2:2*nD+nBF*(2+nD);
end

%UPPER CONSTRAINTS uA*p <= u
uA=zeros(2*nD+nD*nBF+2*nBF);u=zeros(2*nD+nD*nBF+2*nBF,1);
for j=1:nBF  
    %r_n+r_max*c_n <= r_max    
    uA(ind_r{1}(j),ind_r{1}(j))=1;
    uA(ind_r{1}(j),ind_r{1}(j)+1)=r_max;
    u(ind_r{1}(j))=r_max;
end

%LOWER CONSTRAINTS lA*p >= l
lA=zeros(2*nD+nD*nBF+2*nBF);l=zeros(2*nD+nD*nBF+2*nBF,1);
for j=1:nBF
    %lower bound on c s.t. c_0 >= 0; -c_(n-1)+c_n >= cd_min
    lA(ind_c{1}(j),ind_c{i}(j))=1; 
    if j==1
       l(ind_c{1}(j))=0;
    else
        lA(ind_c{1}(j),ind_c{i}(j)-2)=-1;
        l(ind_c{1}(j))=cd_min;
    end    
end

A=[uA;-lA];
b=[u;-l];

if norm==2;
    A(:,end+1)=0;
end