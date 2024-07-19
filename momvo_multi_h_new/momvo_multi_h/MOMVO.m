clc;
clear;
close all;


params.UAV_mode = 'Heterogeneous'; % can be 'Heterogeneous' or 'None'
params.Sp_min = 10; % Velocity in (m/s)
params.Sp_max = 40;
params.Tr_min = 1500; % Transmission range in (m)
params.Tr_max = 3000;
params.Bcp_min = 500; % Battery capacity in (mAh)
params.Bcp_max = 1200;
params.Bch_min = 1; % number of hours to charge/discharge in (h)
params.Bch_max = 1.5;


% upper and lower bounds for UAVs and Tasks positions
params.x_lb = 0;
params.x_ub = 7000;
params.y_lb = 0;
params.y_ub = 7000;
params.z_lb = 1500;
params.z_ub = 3000;

params.N_UAV_data = 30; % Number of UAVs available for deployment

% Optimization parameters
nVar = 15; % Number of UAVs to be deployed
objvNo = 4; % Number of objective functions
Max_time = 100;
nPop = 100;
params.nPop = nPop;
params.nVar = nVar;
params.CD = 200; % Colision-avoidance disatnce (meter)

% Generation of parameters for UAVs and target points
UAV_data = UAVs_generation(params);

params.Ntrgt = 2; % Number of Target points in the area of interest
params.TP_mode = 'Random'; % Target positionning mode, can be 'Random' or 'None'
% if TP_mode='None' -->, the target positions should be specified

Target_data = Target_generation(params);
% Target_data = Target_generation(params, [1500,1800], [2000,2300], [1000,1300]);

CostFunction  = @(a,b,c,d) ObjFun_UAV(a,b,c,d);


ArchMaxSize = ceil(Max_time/nPop)*nPop;

empty_individual.Position.X = [];
empty_individual.Position.Y = [];
empty_individual.Position.Z = [];
empty_individual.Features.U_id = [];
empty_individual.Features.Speed = [];
empty_individual.Features.Transmit_range = [];
empty_individual.Cluster.Group = [];
empty_individual.Cluster.Role = [];
empty_individual.Cluster.Target = [];


Arch_X = repmat(empty_individual, ArchMaxSize, 1);
Arch_F = ones(ArchMaxSize,objvNo)*inf;
Arch_mem_no=0;

Cost = zeros(nPop,objvNo);

WEP_Max=1;
WEP_Min=0.2;


pop = pop_init(UAV_data,params);

Time=1;

tic

while Time<Max_time+1
    
    fprintf('Progress %4s%%\n',num2str(roundn(Time/Max_time*100,-1)));
    
    WEP=WEP_Min+Time*((WEP_Max-WEP_Min)/Max_time);

    TDR=1-((Time)^(1/6)/(Max_time)^(1/6));
    

    for i=1:nPop

        pop(i).Position.X = min(max(pop(i).Position.X, params.x_lb), params.x_ub);
        pop(i).Position.Y = min(max(pop(i).Position.Y, params.y_lb), params.y_ub);
        pop(i).Position.Z = min(max(pop(i).Position.Z, params.z_lb), params.z_ub);
        
        [pop(i),k] = UAV_clustering(pop(i), Target_data, params);
        
        Cost(i,:) = CostFunction(pop(i), Target_data, params, k);
        
    end
    
    Ranks = non_dominated_ranking(Cost,objvNo,nPop);
    [~,sorted_indexes]=sort(Ranks);
    sorted_Cost = Cost(sorted_indexes,:);
    
    for newindex=1:nPop
        Sorted_pop(newindex)=pop(sorted_indexes(newindex));
    end
    
    
    pop(1)= Sorted_pop(1);
    
    [Arch_X,Arch_F, Arch_mem_no] = UpdateArchive(Arch_X,Arch_F,pop,Cost);
    
    if Arch_mem_no>ArchMaxSize
        Arch_mem_ranks=RankingProcess(Arch_F, ArchMaxSize, objvNo);
        [Arch_X,Arch_F,Arch_mem_ranks,Arch_mem_no]=...
  HandleFullArchive(Arch_X,Arch_F,Arch_mem_no,Arch_mem_ranks,ArchMaxSize);
    else
        Arch_mem_ranks=RankingProcess(Arch_F, ArchMaxSize, objvNo);
    end

    

   index=max(RouletteWheelSelection(1./Arch_mem_ranks),1);
  
   Best_pop=Arch_X(index);
   if isempty(Best_pop.Position.X)
       Best_pop = pop(1);
   end
   
  
    r1 = [true, false];
    for i=2:nPop
        Back_hole_index=i;
        for j=1:nVar
            if r1(randi([1,2]))
                White_hole_index=randi([1, nPop]);
                pop(Back_hole_index).Position.X(j)=Sorted_pop(White_hole_index).Position.X(j);
                pop(Back_hole_index).Position.Y(j)=Sorted_pop(White_hole_index).Position.Y(j);
                pop(Back_hole_index).Position.Z(j)=Sorted_pop(White_hole_index).Position.Z(j);
            end
            
                r2=rand();
                if r2<WEP
                    r3=rand();
                    if r3<0.5
                        pop(i).Position.X(j)=Best_pop(1).Position.X(j)+...
                            TDR*((params.x_ub-params.x_lb)*rand+params.x_lb);
                        pop(i).Position.Y(j)=Best_pop(1).Position.Y(j)+...
                            TDR*((params.y_ub-params.y_lb)*rand+params.y_lb);
                        pop(i).Position.Z(j)=Best_pop(1).Position.Z(j)+...
                            TDR*((params.z_ub-params.z_lb)*rand+params.z_lb);
                    end
                    if r3>0.5
                        pop(i).Position.X(j)=Best_pop(1).Position.X(j)-...
                            TDR*((params.x_ub-params.x_lb)*rand+params.x_lb);
                        pop(i).Position.Y(j)=Best_pop(1).Position.Y(j)-...
                            TDR*((params.y_ub-params.y_lb)*rand+params.y_lb);
                        pop(i).Position.Z(j)=Best_pop(1).Position.Z(j)-...
                            TDR*((params.z_ub-params.z_lb)*rand+params.z_lb);
                    end

                end
            
            
        end
    end   

    Time=Time+1;
end

toc;

[~,idF] = min(sum(normc(Arch_F),2));
Final_F = Arch_F(idF,:);
Final_P = Arch_X(idF);

if any(Final_F >= 1e+15)
    fprintf('---------------------------------------------\n');
    fprintf('Some clustring constraints are not satisfied\n');
    fprintf('Try modifiying the transmission range of UAVs\n');
    fprintf('---------------------------------------------\n');   
else

T_P_x = zeros(1,params.Ntrgt);
T_P_y = zeros(1,params.Ntrgt);
T_P_z = zeros(1,params.Ntrgt);
for i=1:params.Ntrgt
    T_P_x(i) = Target_data(i).Position.X;
    T_P_y(i) = Target_data(i).Position.Y;
    T_P_z(i) = Target_data(i).Position.Z;
end

U_list = 1:nVar;
U_P_x = Final_P.Position.X;
U_P_y = Final_P.Position.Y;
U_P_z = Final_P.Position.Z;
U_clusters = [Final_P.Cluster.Group];
U_roles = [Final_P.Cluster.Role];
U_targets = [Final_P.Cluster.Target];
U_G = unique(U_clusters);
U_G(U_G == 0) = [];
Cluster_heads = U_list(U_roles==1);
Cluster_targets = U_targets(Cluster_heads);

Lgnd = cell(1);
counter = 0;


for i = 1:length(U_G)
    Grp = U_list(U_clusters == U_G(i));
    Grp_color = rand(1,3);
    for j=1:length(Grp)
        plot3(U_P_x(Grp(j)), U_P_y(Grp(j)), U_P_z(Grp(j)), 'v', 'color', Grp_color)
        hold on;
        Lgnd{counter + j} = strcat('UAV ', num2str(counter + j));
    end
    counter = counter + length(Grp);
    grid on;
end

hold on;

for i=1:params.Ntrgt
    plot3(T_P_x(i), T_P_y(i), T_P_z(i), 'o')
    Lgnd{nVar + i} = strcat('Target ', num2str(i));
    hold on; grid on;
end

hold on;

% legend(Lgnd,'AutoUpdate', 'off');

U_G = unique(U_clusters);
U_G(U_G == 0) = [];
hold on;
C = {'r';'g';'b'};
N = 3;
wrapN = @(x, N) (1 + mod(x-1, N));
for i = 1:length(U_G)
    Grp = U_list(U_clusters == U_G(i));
    Grp(end+1) = Grp(1);
    patch(U_P_x(Grp), U_P_y(Grp), U_P_z(Grp), C{wrapN(i,N)}, 'facealpha', 0.1);
    hold on;
    clear Grp
end

hold on;
for i=1:length(U_G)
    text(U_P_x(Cluster_heads(i)), U_P_y(Cluster_heads(i)), U_P_z(Cluster_heads(i)), ...
        strcat('CH', num2str(i)))
    hold on;
end


hold on;
for i=1:params.Ntrgt
    text(T_P_x(i), T_P_y(i), T_P_z(i), strcat('TG', num2str(i)))
    hold on;
end

for i=1:length(U_G)
    Grp = U_list(U_clusters == U_G(i));
    if Cluster_targets(i) > 0
        line([mean(U_P_x(Grp)), T_P_x(Cluster_targets(i))], ...
            [mean(U_P_y(Grp)), T_P_y(Cluster_targets(i))], ...
            [mean(U_P_z(Grp)), T_P_z(Cluster_targets(i))], ...
            'Color', C{wrapN(i,N)}, 'LineStyle', '--')
        hold on;
    end
end
end


