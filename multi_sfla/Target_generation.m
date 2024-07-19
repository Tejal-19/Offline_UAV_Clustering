function Target_data = Target_generation(params,varargin)

TP_mode = params.TP_mode;
Ntrgt = params.Ntrgt;

Target_data_empty.Features.T_id = [];
Target_data = repmat(Target_data_empty, Ntrgt, 1);

x_lb = params.x_lb;
x_ub = params.x_ub;
y_lb = params.y_lb;
y_ub = params.y_ub;
z_lb = params.z_lb;
z_ub = params.z_ub;

for i=1:Ntrgt
    Target_data(i).Features.T_id = i;
       
    if strcmp(TP_mode, 'Random')
        Target_data(i).Position.X = rand().*(x_ub-x_lb)+x_lb;
        Target_data(i).Position.Y = rand().*(y_ub-y_lb)+y_lb;
        Target_data(i).Position.Z = rand().*(z_ub-z_lb)+z_lb;
    else
        Target_data(i).Position.X = varargin{1}(i);
        Target_data(i).Position.Y = varargin{2}(i);
        Target_data(i).Position.Z = varargin{3}(i);
    end
end

end

