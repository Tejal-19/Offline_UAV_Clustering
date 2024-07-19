function [F3, F4] = CostFunc(UAVs,Target_data,Dist,params)

nVar = params.nVar;
Ntrgt = params.Ntrgt;

c = 3*10^8;
T_P_x = zeros(1,Ntrgt);
T_P_y = zeros(1,Ntrgt);
T_P_z = zeros(1,Ntrgt);
for i=1:Ntrgt
    T_P_x(i) = Target_data(i).Position.X;
    T_P_y(i) = Target_data(i).Position.Y;
    T_P_z(i) = Target_data(i).Position.Z;
end

U_P_x = UAVs.Position.X;
U_P_y = UAVs.Position.Y;
U_P_z = UAVs.Position.Z;
U_Speed = UAVs.Features.Speed;
U_Target = UAVs.Cluster.Target;
U_Group = UAVs.Cluster.Group;
U_Role = UAVs.Cluster.Role;

N_G_clusters = unique(U_Group);
U_list = 1:nVar;
trvc = 0;
rbc = 0;

for i=1:length(N_G_clusters)
    TestG = ismember(U_Group, N_G_clusters(i));
    Tackled_target = unique(U_Target(TestG));
    
    for ts=1:length(Tackled_target)
        N_u = U_list(U_Target==Tackled_target(ts) & TestG);
        R_list = U_Role(N_u);
        if Tackled_target(ts) ~= 0
         
            d = sqrt((T_P_x(Tackled_target(ts))-U_P_x(N_u)).^2+...
                (T_P_y(Tackled_target(ts))-U_P_y(N_u)).^2+...
                (T_P_z(Tackled_target(ts))-U_P_z(N_u)).^2);
            trvc = trvc + sum(d./U_Speed(N_u));

        else
            trvc = trvc + 0;

        end
    end
    if N_G_clusters(i) > 0
        L_idx = N_u(R_list == 1);
        F_idx = N_u(R_list == 0);
        rbc_temp = exp(-log10(mean(Dist(L_idx, F_idx))/c));
        if ~isinf(rbc_temp)
            rbc = rbc + rbc_temp;
        else
            rbc = rbc + 0;
        end
    else
        rbc = rbc + 0;
    end
end


F3 = trvc; % travel cost % to be minimized
F4 = -rbc; % reliability of connectivity % to be maximized

end
