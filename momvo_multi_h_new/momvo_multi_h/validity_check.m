function [DB, R2] = validity_check(UAVs, nVar, Dist)

U_list = 1:nVar;
U_G = unique([UAVs.Cluster.Group]);
U_G(U_G == 0) = [];

L = [];
F = [];

if ~isempty(U_G)
    for i = 1:length(U_G)
        Lidx = U_list(UAVs.Cluster.Group == U_G(i) & UAVs.Cluster.Role == 1);
        Fidx = U_list(UAVs.Cluster.Group == U_G(i) & UAVs.Cluster.Role == 0);
        L = [L; [Lidx, U_G(i)]];
        F = [F; [Fidx', U_G(i)*ones(size(Fidx'))]];
        Clusters(i).Leader = Lidx;
        Clusters(i).Followers = Fidx;
    end

    LF = [L(:,1); F(:,1)];
    LF_idx = [L(:,2); F(:,2)];

    k = numel(Clusters);

    U_P_x = UAVs.Position.X;
    U_P_y = UAVs.Position.Y;
    U_P_z = UAVs.Position.Z;

    % Barycenter
    Gx = mean(U_P_x);
    Gy = mean(U_P_y);
    Gz = mean(U_P_z);

    sw = 0;
    sb = 0;
    swmn = zeros(1,k);
    D_LL = zeros(k);
    SM = zeros(k);
    for i=1:k
        Ck = length(Clusters(i).Followers)+1;
        sw = sw + sum(Dist(Clusters(i).Leader, Clusters(i).Followers));
        sb = sb + Ck*sqrt((U_P_z(Clusters(i).Leader)-Gx).^2+...
            (U_P_y(Clusters(i).Leader)-Gy).^2+...
            (U_P_z(Clusters(i).Leader)-Gz).^2);
        swmn(i) = mean(Dist(Clusters(i).Leader, Clusters(i).Followers));
        for jj=1:k
            SM(i, jj) = swmn(i) + swmn(jj);
            D_LL(i,jj) = Dist(Clusters(i).Leader, Clusters(jj).Leader);
        end
    end
    SM = SM - diag(diag(SM));
    D_LL(D_LL==0) = Inf;
    sw = sw/nVar;
    sb = sb/nVar;
    R2 = -(sw*sb)/(sw+sb); % to be maximized
    DB = sum((max(SM,[],2))./(min(D_LL,[],2)))/k; % to be minimized

else
    R2 = 1e+15;
    DB = 1e+15;
end


