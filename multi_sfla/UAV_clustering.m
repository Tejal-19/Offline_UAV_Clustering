function [UAVs, k] = UAV_clustering(UAVs, Target_data, params)

nVar = params.nVar;
Ntrgt = params.Ntrgt;

Nmin_clusters = min(Ntrgt, ceil(nVar/Ntrgt));
Nmax_clusters = max(max(Ntrgt, ceil(nVar/Ntrgt)), floor(nVar/2));
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

% UAV-Targets Distance computation
D_u_t = zeros(nVar,Ntrgt);
for i=1:nVar
    for j=1:Ntrgt
        D_u_t(i,j) = sqrt((U_P_x(i)-T_P_x(j))^2+...
            (U_P_y(i)-T_P_y(j))^2+(U_P_z(i)-T_P_z(j))^2);
    end
end

U_id = [UAVs.Features.U_id]';

D_u_u = zeros(nVar,nVar);
for i=1:nVar
    for j=1:nVar
        D_u_u(i,j) = sqrt((U_P_x(i)-U_P_x(j))^2+...
            (U_P_y(i)-U_P_y(j))^2+(U_P_z(i)-U_P_z(j))^2);
    end
end

Du = mean(D_u_u,2);
Dt = mean(D_u_t,2);

D_u_u(1:(nVar+1):end) = ones(1,nVar)*1e+15;

[med, index]=initmed(U_id, Du, Dt, k); % init the first cluster heads

err = nVar;
counter = 1;
while err > nVar/k
    med_prev = med;
    index_prev = index;
    [sdist, cid] = initmem(D_u_u,index); % add member to the cluster heads (nearest UAVs)
    err = abs(err-sdist);
    
    U_cid = unique(cid);
    temp = zeros(1,length(U_cid));
    for j = 1:length(U_cid)
        Grp = U_list(ismember(cid, U_cid(j)));
        temp(j) = sum(ismember(med, Grp));
    end
    if ~all(med) || ~all(temp == 1)
        med = med_prev;
        index = index_prev;
        counter = counter + 1;
    else    
    [med, index] = updatemed(U_id,D_u_u,D_u_t,cid,index,k);
    end

    if counter == floor(nVar/k)
        break
    end
end

    test = zeros(1,k);
    for j = 1:k
        Grp = U_list(ismember(cid, j));
        test(j) = ismember(med(j), Grp);
    end
    if ~all(test) || ~all(med) % failed to correctly place the UAVs in clusters
        
        UAVs.Cluster.Group = zeros(1, nVar);
        UAVs.Cluster.Role = -ones(1, nVar);
        UAVs.Cluster.Target = zeros(1, nVar);
        
        U_G = unique(cid);
U_G(U_G==0) = [];
for j = 1:length(U_G)
    Grp = U_list(ismember(cid, U_G(j)));
    [med, ~] = initmed(U_id(Grp), Du(Grp,:), Dt(Grp,:), 1);
    L_idx = find(U_id == med);
    F_idx = Grp(Grp ~= L_idx);
    if length(Grp)>1
        UAVs.Cluster.Group(Grp) = j;
        UAVs.Cluster.Role(L_idx) = 1;
        UAVs.Cluster.Role(F_idx) = 0;
    
    if ~isempty(D_u_t)
            [~, Trgt_idx] = min(D_u_t(L_idx,:));
                UAVs.Cluster.Target(Grp) = T_id(Trgt_idx);
                D_u_t(:, Trgt_idx) = [];
                T_id(Trgt_idx) = [];
        else
            UAVs.Cluster.Target(Grp) = 0;
    end
    end

end

    else
UAVs.Cluster.Group = cid;
UAVs.Cluster.Role = -ones(1, nVar);
UAVs.Cluster.Target = zeros(1, nVar);

c_count = histcounts(cid);
if any(c_count==1)
    for u = 1:nVar
        if (cid(u) == 0 && ismember(u, index)) || ...
                (sum(cid == cid(u)) == 1 && cid(u) ~= 0 && ismember(u, index))
                med(index == u) = Inf;
        end
        if sum(cid == cid(u)) == 1
            UAVs.Cluster.Group(u) = 0;
            UAVs.Cluster.Role(u) = -1;
            UAVs.Cluster.Target(u) = 0;
        end
    end
end
med(med == inf) = [];

U_G = unique([UAVs.Cluster.Group]);

U_G(U_G==0) = [];

for j = 1:length(U_G)
    Grp = U_list(ismember([UAVs.Cluster.Group], U_G(j)));
            L_idx = find(U_id == med(j));
            F_idx = Grp(Grp ~= L_idx);
        UAVs.Cluster.Role(L_idx) = 1;
        UAVs.Cluster.Role(F_idx) = 0;
        

        if ~isempty(D_u_t)
            [~, Trgt_idx] = min(D_u_t(L_idx,:));
                UAVs.Cluster.Target(Grp) = T_id(Trgt_idx);
                D_u_t(:, Trgt_idx) = [];
                T_id(Trgt_idx) = [];
        else
            UAVs.Cluster.Target(Grp) = 0;
        end

end

    end
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

    
end

