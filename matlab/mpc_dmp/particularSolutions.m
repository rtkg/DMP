function dp=particularSolutions(t,p,t_k1,DMP,options)

dp(1)=canonicalSystem(t,p(1),options.Tau);

K=expm(DMP.A*(t_k1-t))*DMP.B;

for i=1:length(DMP.param)
   nBF=length(DMP.param{i}.pBF)/2;
   w=DMP.param{i}.w(3:end); %basis function weights (without a & b)
   pBF=DMP.param{i}.pBF; %nonlinear basis function parameters
   ud_=[];

   for j=1:nBF
       ind=2*j-1:2*j;
       ud_=[ud_ gaussmf(p(1),pBF(ind))];
   end    
   dp=[dp; K*ud_*w]; 
end    



% for i=1:nBF
%    ind = 2*i-1:2*i;  
%   B = [B, gaussmf(x(1),pBF(ind))/T^2];
% end

%EOF

