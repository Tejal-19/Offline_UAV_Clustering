
function y=Mutate(x,params)

nVar=params.nVar;
pmu=params.pmu;
mu=params.mu;
x_lb = params.x_lb;
x_ub = params.x_ub;
y_lb = params.y_lb;
y_ub = params.y_ub;
z_lb = params.z_lb;
z_ub = params.z_ub;

nMu=ceil(pmu*nVar);

j=randsample(nVar,nMu);

  
y=x;
for i=1:nVar

r(i)= rand(1);
   if r(i) < 0.5
       delta(i)= (2*r(i))^(1/mu+1);
   else
       delta(i)= 1-(2*(1-r(i)))^(1/mu+1);
   end
end
 


y.Position.X(j)= y.Position.X(j) + (x_ub-x_lb)*delta(j);
y.Position.Y(j)= y.Position.Y(j) + (y_ub-y_lb)*delta(j);
y.Position.Z(j)= y.Position.Z(j) + (z_ub-z_lb)*delta(j);
y.Rank=[];
y.DominationSet=[];
y.DominatedCount=[];
y.CrowdingDistance=[];
y.Cluster.Group = [];
y.Cluster.Role = [];
y.Cluster.Target = [];

    
        y.Position.X = min(max(y.Position.X, x_lb), x_ub);
        y.Position.Y = min(max(y.Position.Y, y_lb), y_ub);
        y.Position.Z = min(max(y.Position.Z, z_lb), z_ub);


end