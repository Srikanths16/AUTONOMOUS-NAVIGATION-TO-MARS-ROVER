function feasible=feasiblePoint(point,terrain)
feasible=true;
if ~(point(1)>=1 && point(1)<=size(terrain,1) && point(2)>=1 && point(2)<=size(terrain ,2) && terrain(point(1),point(2))==1)
 feasible=false;
end