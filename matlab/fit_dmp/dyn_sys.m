function [dx f] = dyn_sys(t,x,DMPi,T)
%x=[s; q; dq]
% define ode xdd=a*x+b*xd+f; td=T;


w=DMPi.w;
pBF=DMPi.pBF;
nBF=length(pBF)/2;
if (T <= 0)
    error('Time constant has to be larger then zero');
end



dx(2) = x(3);

B = [x(2)/T^2, x(3)/T];


for i=1:nBF
   ind = 2*i-1:2*i;  
  B = [B, gaussmf(x(1),pBF(ind))/T^2];
end

dx(3) = B*w;
dx(1) = 1/T;

dx = dx(:);

if nargout==2
    f=B*[0;0;w(3:end)];
end


%%%EOF