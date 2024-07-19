
function [xnew] = Mutate(x, pm, params)

    nVar = params.nVar;
    jx = randi([1 nVar]);
    jy = randi([1 nVar]);
    jz = randi([1 nVar]);
    
    
    vx = pm*unifrnd(params.x_lb,params.x_ub);
%     vx = min(max(vx, params.x_lb), params.x_ub);
    
    vy = pm*unifrnd(params.y_lb,params.y_ub);
%     vy = min(max(vy, params.y_lb), params.y_ub);
    
    vz = pm*unifrnd(params.z_lb,params.z_ub);
%     vz = min(max(vz, params.z_lb), params.z_ub);    
    
    xnew = x;
    xnew.Position.X(jx) = vx;
    xnew.Position.Y(jy) = vy;
    xnew.Position.Z(jz) = vz;

    
% U_P_x = xnew.Position.X;
% U_P_y = xnew.Position.Y;
% U_P_z = xnew.Position.Z;
% 
% for i=1:nVar
%     for j=1:nVar
%         if j ~= i && isequal([U_P_x(i), U_P_y(i), U_P_z(i)],...
%                                [U_P_x(j), U_P_y(j), U_P_z(j)])
%             U_P_x(i) = rand().*(params.x_ub-params.x_lb)+params.x_lb;
%             U_P_y(i) = rand().*(params.y_ub-params.y_lb)+params.y_lb;
%             U_P_z(i) = rand().*(params.z_ub-params.z_lb)+params.z_lb;
%             U_P_x(j) = rand().*(params.x_ub-params.x_lb)+params.x_lb;
%             U_P_y(j) = rand().*(params.y_ub-params.y_lb)+params.y_lb;
%             U_P_z(j) = rand().*(params.z_ub-params.z_lb)+params.z_lb;
%         end
%     end
% end
% 
% xnew.Position.X = U_P_x;
% xnew.Position.Y = U_P_y;
% xnew.Position.Z = U_P_z;

