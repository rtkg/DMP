function O=createObstacle(Obst)
O=[];
if strcmp(Obst.type,'circle')
    phi=linspace(0,2*pi-2*pi/Obst.nP,Obst.nP);
    O=Polyhedron([Obst.r*cos(phi)+Obst.r0(1); Obst.r*sin(phi)+Obst.r0(2)]');
elseif  strcmp(Obst.type,'square')
    r0=Obst.r0; r0=r0(:)';
    r=Obst.r;
    Po=[r0+[r r]; r0+[r -r]; r0+[-r r]; r0+[-r -r]];
    O=Polyhedron(Po);
elseif  strcmp(Obst.type,'rectangle')
    warning('Obstacle type rectangle not implemented yet!');    
    return
elseif strcmp(Obst.type,'random')
   a1=Obst.r0(1)-Obst.r; b1=Obst.r0(1)+Obst.r;
   a2=Obst.r0(2)-Obst.r; b2=Obst.r0(2)+Obst.r;
   O=Polyhedron([a1+(b1-a1).*rand(Obst.nP,1) a2+(b2-a2).*rand(Obst.nP,1)]);
else
    error('Unknown obstacle type!');    
end
O.computeHRep();
O.normalize();


