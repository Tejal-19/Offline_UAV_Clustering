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
nVar = 6; % Number of UAVs to be deployed
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

pCrossover=0.9;                         
nCrossover=2*round(pCrossover*nPop/2);  



pMutation=0.1;                          
nMutation=round(pMutation*nPop);        

params.pmu=0.02;                   
params.mu=20;

Cost = zeros(nPop,objvNo);

empty_individual.Position.X = [];
empty_individual.Position.Y = [];
empty_individual.Position.Z = [];
empty_individual.Features.U_id = [];
empty_individual.Features.Speed = [];
empty_individual.Features.Transmit_range = [];
empty_individual.Cluster.Group = [];
empty_individual.Cluster.Role = [];
empty_individual.Cluster.Target = [];
empty_individual.Rank=[];
empty_individual.DominationSet=[];
empty_individual.DominatedCount=[];
empty_individual.CrowdingDistance=[];

pop = pop_init(UAV_data,params);

for i=1:nPop
    
    [pop(i),k] = UAV_clustering(pop(i), Target_data, params);
        
    Cost(i,:) = CostFunction(pop(i), Target_data, params, k);
    
end


[pop, F]=NonDominatedSorting(pop, Cost);


pop=CrowdingDistance(pop,Cost,F);


[pop, Cost, F]=SortPopulation(pop, Cost);


tic

for it=1:Max_time

    fprintf('Progress %4s%%\n',num2str(roundn(it/Max_time*100,-1)));
    
    popc=repmat(empty_individual,nCrossover/2,2);
    popc1_Cost = zeros(nCrossover/2, objvNo);
    popc2_Cost = zeros(nCrossover/2, objvNo);

    for k=1:nCrossover/2
        
        i1=randi([1 nPop]);
        p1=pop(i1);
        
        i2=randi([1 nPop]);
        p2=pop(i2);
        
        [popc(k,1), popc(k,2)]=Crossover(p1,p2,params);
        
        [popc(k,1),k1] = UAV_clustering(popc(k,1), Target_data, params);
        popc1_Cost(k,:) = CostFunction(popc(k,1), Target_data, params, k1);

        [popc(k,2),k2] = UAV_clustering(popc(k,2), Target_data, params);
        popc2_Cost(k,:) = CostFunction(popc(k,2), Target_data, params, k2);
        
    end
    popc=popc(:);
    popc_Cost = [popc1_Cost; popc2_Cost];
    
    
    popm=repmat(empty_individual,nMutation,1);
    popm_Cost = zeros(nMutation, objvNo);

    for k=1:nMutation
        
        i=randi([1 nPop]);
        p=pop(i);
        
        popm(k) = Mutate(p,params);
        
        [popm(k),k3] = UAV_clustering(popm(k), Target_data, params);
        popm_Cost(k,:) = CostFunction(popm(k), Target_data, params, k3);
        
    end
    
    
    pop = [pop; popc; popm];
    Cost = [Cost; popc_Cost; popm_Cost];
     
    
    [pop, F]=NonDominatedSorting(pop, Cost);
   
    pop=CrowdingDistance(pop, Cost, F);

    
    [pop, Cost, ~] = SortPopulation(pop, Cost);
    
    
    pop=pop(1:nPop);
    
    
    
    [pop, F]=NonDominatedSorting(pop, Cost);

    
    pop=CrowdingDistance(pop, Cost, F);

    
    [pop, Cost, F]=SortPopulation(pop, Cost);
    
    
    F1=pop(F{1});
    F1_cost = Cost(F{1},:);
    
end
toc;

[~,idF] = min(sum(normc(F1_cost),2));
Final_F = F1_cost(idF,:);
Final_P = F1(idF);

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
        line([U_P_x(Grp(U_roles(Grp)==1)), T_P_x(Cluster_targets(i))], ...
            [U_P_y(Grp(U_roles(Grp)==1)), T_P_y(Cluster_targets(i))], ...
            [U_P_z(Grp(U_roles(Grp)==1)), T_P_z(Cluster_targets(i))], ...
            'Color', C{wrapN(i,N)}, 'LineStyle', '--')
        hold on;
    end
end

end


