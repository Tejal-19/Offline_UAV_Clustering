
function [y1, y2]=Crossover(x1,x2,params)


  mu= params.mu;
  nVar= params.nVar;

  x_lb = params.x_lb;
  x_ub = params.x_ub;
  y_lb = params.y_lb;
  y_ub = params.y_ub;
  z_lb = params.z_lb;
  z_ub = params.z_ub;

  P = [x1;x2];
  pl = 1:2;
  idx1 = pl(randperm(2, 1));
  idx2 = pl(pl~=idx1);
  y1 = P(idx1);
  y2 = P(idx2);
  
   for i=1:nVar
                 
u(i) = rand(1);
    if u(i) <= 0.5
        bq(i)= (2*u(i))^(1/(mu+1));
    else
        bq(i)= (1/(2*(1-u(i))))^(1/(mu+1));
    end

    
y1.Position.X(i)=0.5*(((1+bq(i))*P(idx1).Position.X(i))+(1-bq(i))*P(idx2).Position.X(i));
y1.Position.Y(i)=0.5*(((1+bq(i))*P(idx1).Position.Y(i))+(1-bq(i))*P(idx2).Position.Y(i));
y1.Position.Z(i)=0.5*(((1+bq(i))*P(idx1).Position.Z(i))+(1-bq(i))*P(idx2).Position.Z(i));
y1.Rank=[];
y1.DominationSet=[];
y1.DominatedCount=[];
y1.CrowdingDistance=[];
y1.Cluster.Group = [];
y1.Cluster.Role = [];
y1.Cluster.Target = [];

y2.Position.X(i)=0.5*(((1-bq(i))*P(idx1).Position.X(i))+(1+bq(i))*P(idx2).Position.X(i));
y2.Position.Y(i)=0.5*(((1-bq(i))*P(idx1).Position.Y(i))+(1+bq(i))*P(idx2).Position.Y(i));
y2.Position.Z(i)=0.5*(((1-bq(i))*P(idx1).Position.Z(i))+(1+bq(i))*P(idx2).Position.Z(i));
y2.Rank=[];
y2.DominationSet=[];
y2.DominatedCount=[];
y2.CrowdingDistance=[];
y2.Cluster.Group = [];
y2.Cluster.Role = [];
y2.Cluster.Target = [];
    
  end
 
        y1.Position.X = min(max(y1.Position.X, x_lb), x_ub);
        y1.Position.Y = min(max(y1.Position.Y, y_lb), y_ub);
        y1.Position.Z = min(max(y1.Position.Z, z_lb), z_ub);

        y2.Position.X = min(max(y2.Position.X, x_lb), x_ub);
        y2.Position.Y = min(max(y2.Position.Y, y_lb), y_ub);
        y2.Position.Z = min(max(y2.Position.Z, z_lb), z_ub);

        
end