function pop = pop_init(UAV_data,params)

x_lb = params.x_lb;
x_ub = params.x_ub;
y_lb = params.y_lb;
y_ub = params.y_ub;
z_lb = params.z_lb;
z_ub = params.z_ub;
nVar = params.nVar;
nPop = params.nPop;

pop_empty.Features.U_id = [];
pop_empty.Features.Speed = [];
pop_empty.Features.Transmit_range = [];
pop_empty.Features.B_capacity = [];
pop_empty.Features.B_charging = [];
pop_empty.Position.X = [];
pop_empty.Position.Y = [];
pop_empty.Position.Z = [];
pop_empty.Cluster.Group = [];
pop_empty.Cluster.Role = [];
pop_empty.Cluster.Target = [];

pop = repmat(pop_empty, nPop, 1);

for i=1:nPop
    p_idx = randsample(numel(UAV_data), nVar);
    for j=1:nVar
        
        pop(i).Features.U_id(j) = UAV_data(p_idx(j)).Features.U_id;
        pop(i).Features.Speed(j) = UAV_data(p_idx(j)).Features.Speed;
        pop(i).Features.Transmit_range(j) = UAV_data(p_idx(j)).Features.Transmit_range;
        pop(i).Features.B_capacity(j) = UAV_data(p_idx(j)).Features.B_capacity;
        pop(i).Features.B_charging(j) = UAV_data(p_idx(j)).Features.B_charging;


        pop(i).Position.X(j) = rand().*(x_ub-x_lb)+x_lb;
        pop(i).Position.Y(j) = rand().*(y_ub-y_lb)+y_lb;
        pop(i).Position.Z(j) = rand().*(z_ub-z_lb)+z_lb;
    end
end

end

