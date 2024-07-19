function [UAVs, k] = UAV_clustering(UAVs, Target_data, params)

nVar = params.nVar;
Ntrgt = params.Ntrgt;

Nmin_clusters = min(Ntrgt, floor(nVar/Ntrgt));
Nmax_clusters = max(Ntrgt, ceil(nVar/Ntrgt));
if Nmax_clusters > nVar
    Nmax_clusters = floor(nVar/2);
end
K_clusters = Nmin_clusters:Nmax_clusters;
k = K_clusters(randperm(length(K_clusters),1));

U_list = 1:nVar;

T_P_x = zeros(1,Ntrgt);
T_P_y = zeros(1,Ntrgt);
T_P_z = zeros(1,Ntrgt);
T_id = zeros(1,Ntrgt);
for i=1:Ntrgt
    T_P_x(i) = Target_data(i).Position.X;
    T_P_y(i) = Target_data(i).Position.Y;
    T_P_z(i) = Target_data(i).Position.Z;
    T_id(i) = Target_data(i).Features.T_id;
end


U_P_x = UAVs.Position.X;
U_P_y = UAVs.Position.Y;
U_P_z = UAVs.Position.Z;

U_transmit_range = UAVs.Features.Transmit_range;
U_transmit_range = U_transmit_range';

U_Roles = -ones(1, nVar);
U_Targets = zeros(1, nVar);
U_Groups = zeros(1, nVar);


U_discharge_rate =  UAVs.Features.B_capacity ./ UAVs.Features.B_charging;
U_discharge_rate = U_discharge_rate';

D_u_t = zeros(nVar,Ntrgt);
for i=1:nVar
    for j=1:Ntrgt
        D_u_t(i,j) = sqrt((U_P_x(i)-T_P_x(j))^2+...
            (U_P_y(i)-T_P_y(j))^2+(U_P_z(i)-T_P_z(j))^2);
    end
end

D_u_u = zeros(nVar,nVar);
for i=1:nVar
    for j=1:nVar
        D_u_u(i,j) = sqrt((U_P_x(i)-U_P_x(j))^2+...
            (U_P_y(i)-U_P_y(j))^2+(U_P_z(i)-U_P_z(j))^2);
    end
end

Leaders = zeros(1, k);
TX = zeros(1, k);

for t = 1:k
    
    if t <= Ntrgt & ~all(all(isinf(D_u_t)))
        [Nearest, L_idx] = min(D_u_t(:, t));

%         if Nearest <= U_transmit_range(U_list(ismember(U_list, L_idx)))
            Leaders(t) = U_list(ismember(U_list, L_idx) & ~ismember(U_list, Leaders));
            TX(t) = T_id(t);
            D_u_t(:, t) = Inf;
            D_u_t(ismember(U_list, Leaders(t)), :) = Inf;
            U_discharge_rate(Leaders(t)) = -Inf;
%             U_transmit_range(Leaders(t)) = -Inf;

%         else
%             [~, L_idx] = max(U_discharge_rate(U_list));
%             Leaders(t) = U_list(ismember(U_list, L_idx) & ~ismember(U_list, Leaders));
%             TX(t) = 0;
%             U_discharge_rate(Leaders(t)) = -Inf;
%             U_transmit_range(Leaders(t)) = -Inf;
%         end

    else

        [~, L_idx] = max(U_discharge_rate(U_list));
        Leaders(t) = U_list(ismember(U_list, L_idx) & ~ismember(U_list, Leaders));
        TX(t) = 0;
        U_discharge_rate(Leaders(t)) = -Inf;
%         U_transmit_range(Leaders(t)) = -Inf;
    end

end

% U_transmit_range = UAVs.Features.Transmit_range;
% U_transmit_range = U_transmit_range';

counter = 1;

while ~isempty(U_list) & counter <= k

    Neighbors_list = U_list(~ismember(U_list, Leaders));
    Neighbors = D_u_u(Leaders(counter), Neighbors_list);

    Followers = Neighbors_list(Neighbors <= U_transmit_range(Leaders(counter)));

    if ~isempty(Followers)
        U_Roles(Leaders(counter)) = 1;
        U_Roles(Followers) = 0;
        U_Groups(Leaders(counter)) = counter;
        U_Groups(Followers) = counter;
        U_Targets(Leaders(counter)) = TX(counter);
        U_Targets(Followers) = U_Targets(Leaders(counter));
    end

    D_u_u(ismember(U_list, Leaders(counter)) | ismember(U_list, Followers), :) = Inf;
    U_list(ismember(U_list, Leaders(counter)) | ismember(U_list, Followers)) = [];

    counter = counter + 1;

end

UAVs.Cluster.Group = U_Groups;
UAVs.Cluster.Role = U_Roles;
UAVs.Cluster.Target = U_Targets;

U_G = unique([UAVs.Cluster.Group]);
U_G(U_G==0) = [];

K_list = 1:length(U_G);

K_test = all(ismember(U_G, K_list));

temp = ismember(K_list, U_G);

if length(U_G) < k && ~K_test
    while ~all(temp) && ~K_test
        test = UAVs.Cluster.Group > find(temp==0,1,'first');
        UAVs.Cluster.Group(test) = UAVs.Cluster.Group(test) - 1;
        U_G = unique([UAVs.Cluster.Group]);
        U_G(U_G==0) = [];
        temp = ismember(K_list, U_G);
        K_test = all(ismember(U_G, K_list));
    end
end

