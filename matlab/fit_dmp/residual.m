function r = residual(varargin)

p=varargin{1};
D=varargin{2};
if nargin==3
    S=varargin{3};
else
    S=eye(length(p));
end

nD=length(D);
nBF=(length(p)-2*nD)/(2+nD);
nS=0;
ddx=[];

for i=1:nD
    ind_d{i}=nS+1:1:nS+size(D{i},1);
    ind_a{i}=(2+nBF)*(i-1)+1;
    ind_b{i}=(2+nBF)*(i-1)+2;
    ind_w{i}=i*(2+nBF)-nBF+1:1:i*(2+nBF);
    ind_r{i}=nD*(nBF+2)+1:2:2*nD+nBF*(2+nD)-1;
    ind_c{i}=nD*(nBF+2)+2:2:2*nD+nBF*(2+nD);
    nS=nS+size(D{i},1);
    ddx=[ddx; D{i}(:,4)];
end

B=zeros(nS,2*nD+nBF*(2+nD));
p=S*p;

for i=1:nD
    B(ind_d{i},ind_a{i})=D{i}(:,2);
    B(ind_d{i},ind_b{i})=D{i}(:,3);
    for j=1:nBF
        B(ind_d{i},ind_w{i}(j))=gaussmf(D{i}(:,1),[p(ind_r{i}(j)), p(ind_c{i}(j))]);
    end
end

r = B*p-ddx;

%%%EOF
