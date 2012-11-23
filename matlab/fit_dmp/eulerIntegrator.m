function [T,X,f]=eulerIntegrator(tspan,Td,x0,param,Tau);

x0=x0(:);

X=x0';
T=tspan(1):Td:tspan(2);

for i=1:length(T)
   [dx f(i,1)]= dyn_sys(T(i),X(i,1:3),param,Tau);
   X(i,4)=dx(3);  
   
   X(i+1,3)=X(i,3)+dx(3)*Td;
   X(i+1,2)=X(i,2)+dx(2)*Td;
   X(i+1,1)=X(i,1)+dx(1)*Td;
end    
X(end,:)=[];
