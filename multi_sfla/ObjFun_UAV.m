function out = ObjFun_UAV(UAVs, Target_data, params, N_clusters)

nVar = params.nVar;
CD = params.CD;
Ntrgt = params.Ntrgt;

Dist = zeros(nVar, nVar);
U_P_x = UAVs.Position.X;
U_P_y = UAVs.Position.Y;
U_P_z = UAVs.Position.Z;

for i=1:nVar
    for j=1:nVar
        Dist(i,j) = sqrt((U_P_x(i)-U_P_x(j))^2+...
            (U_P_y(i)-U_P_y(j))^2+(U_P_z(i)-U_P_z(j))^2);
    end
end

[F1, F2] = validity_check(UAVs, nVar, Dist);
[F3, F4] = CostFunc(UAVs, Target_data, Dist, params);

out = [F1, F2, F3, F4];
out = out + getnonlinear(UAVs, Target_data, nVar, Ntrgt, Dist, CD, N_clusters);

end

function Z=getnonlinear(UAVs, Target_data, nVar, Ntrgt, Dist, CD, N_clusters)

Z=0;
lam=10^15;

T_id = zeros(1,Ntrgt);
for i=1:Ntrgt
    T_id(i) = Target_data(i).Features.T_id;
end

U_Group = [UAVs.Cluster.Group];
N_G_clusters = unique(U_Group);
U_Target = [UAVs.Cluster.Target];
Dist(1:(nVar+1):end) = ones(1,nVar).*CD;
N_count = histcounts(U_Group);


Assigned_target = zeros();
for i=1:length(N_G_clusters)
    Assigned_target(i) = length(unique(U_Target(ismember(U_Group, N_G_clusters(i)))));
end

U_P_x = UAVs.Position.X;
U_P_y = UAVs.Position.Y;
U_P_z = UAVs.Position.Z;
Test_Ovlp = false(1, nVar);
for i=1:length(N_G_clusters)
    TestG = ismember(U_Group, N_G_clusters(i));

    is_inside_temp = false();
    is_inside = false(1, nVar);

    x_min = min(U_P_x(TestG));
    y_min = min(U_P_y(TestG));
    z_min = min(U_P_z(TestG));
    x_max = max(U_P_x(TestG));
    y_max = max(U_P_y(TestG));
    z_max = max(U_P_z(TestG));

    Px = [U_P_x(~TestG)', U_P_y(~TestG)', U_P_z(~TestG)'];

    P1 = [x_min, y_min, z_min];
    P2 = [x_min, y_max, z_min];
    P4 = [x_max, y_min, z_min];
    P5 = [x_min, y_min, z_max];

    u = cross(P1-P4, P1-P5);
    v = cross(P1-P2, P1-P5);
    w = cross(P1-P2, P1-P4);

    for p=1:size(Px,1)
        c1 = discretize(dot(u,Px(p,:)), ...
            [min(dot(u,P2),dot(u,P1)), max(dot(u,P2),dot(u,P1))]) == 1;
        c2 = discretize(dot(v,Px(p,:)), ...
            [min(dot(v,P1),dot(v,P4)), max(dot(v,P1),dot(v,P4))]) == 1;
        c3 = discretize(dot(w,Px(p,:)), ...
            [min(dot(w,P1),dot(w,P5)), max(dot(w,P1),dot(w,P5))]) == 1;

        if c1 && c2 && c3
            is_inside_temp(p) = true;
        else
            is_inside_temp(p) = false;
        end

    end
end

    is_inside(~TestG) = is_inside_temp;
    Test_Ovlp = Test_Ovlp | is_inside;

    EQ = false(nVar,nVar);
for i=1:nVar
    for j=1:nVar
        EQ(i,j) = isequal([U_P_x(i), U_P_y(i), U_P_z(i)], [U_P_x(j), U_P_y(j), U_P_z(j)]);
    end
end
EQ(1:(nVar+1):end) = false(1,nVar);

geq(1) = 1 - all(all(Dist >= CD));
geq(2) = 1 - all(U_Group); % check if all UAVs are in clusters (no outliers)
geq(3) = 1 - all(ismember(T_id, U_Target)); % all targets should be identified by clusters
geq(4) = 1 - all(Assigned_target == 1); % one waypoint per cluster
geq(5) = 1 - (sum(Test_Ovlp | ismember(U_Group, 0)) == 0); % clusters do not overlap
geq(6) = 1 - all(N_count > 2); % members of a cluster should more or equal to 3
geq(7) = 1 - max(U_Group)==N_clusters; % number of clusters should be equal to k as given in input
geq(8) = 1 - all(sum(EQ)==0);% a given coordiante (x,y,z) should not be occupied by more than one UAV


for q=1:length(geq)
   Z = Z + lam * geq(q)^2 * getHeq(geq(q));
end


end


function H=getHeq(geq)
if geq==0
   H=0;
else
   H=1;
end

end
