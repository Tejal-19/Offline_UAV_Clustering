clc;
clear;
close all;

tic
%% Problem Definition

% initializing the necessary parameters
%TestNo = 1;
params.UAV_mode = 'Heterogeneous'; % can be 'Heterogeneous' or 'None'
params.Sp_min = 10; % Velocity in (m/s)
params.Sp_max = 40;


% upper and lower bounds for UAVs and Tasks positions
params.x_lb = 0;
params.x_ub = 7000;
params.y_lb = 0;
params.y_ub = 7000;
params.z_lb = 1500;
params.z_ub = 3000;

params.N_UAV_data = 30; % Number of UAVs available for deployment

%[nVar, objvNo, lb, ub, CostFunction, TruePF] = ObjFun(TestNo);
% Optimization parameters
nVar = 23; % Number of UAVs to be deployed
objvNo = 4; % Number of objective functions
Max_time = 100;
params.nVar = nVar;
params.CD = 200; % Colision-avoidance disatnce (meter)

% Generation of parameters for UAVs and target points
UAV_data = UAVs_generation(params);

params.Ntrgt = 2; % Number of Target points in the area of interest
params.TP_mode = 'Random'; % Target positionning mode, can be 'Random' or 'None'
% if TP_mode='None' -->, the target positions should be specified

Target_data = Target_generation(params);
%% Problem Definition

CostFunction = @(a,b,c,d)ObjFun_UAV(a,b,c,d);  % Cost Function


VarSize = [1 nVar];     % Unknown Variables Matrix Size

%% SFLA Parameters

MaxIt = 100;        % Maximum Number of Iterations
nPop = 100;
nPopMemeplex = nPop/5;                          % Memeplex Size
nPopMemeplex = max(nPopMemeplex, nVar+1);   % Nelder-Mead Standard

nMemeplex = 5;                  % Number of Memeplexes
nPop = nMemeplex*nPopMemeplex;	% Population Size

I = reshape(1:nPop, nMemeplex, []);
params.nPop = nPop;

% FLA Parameters
fla_params.q = max(round(0.3*nPopMemeplex),2);   % Number of Parents
fla_params.alpha = 3;   % Number of Offsprings
fla_params.beta = 5;    % Maximum Number of Iterations
fla_params.sigma = 2;   % Step Size
fla_params.CostFunction = CostFunction;
% fla_params.lb = lb;
% fla_params.ub = ub;


%% Initialization

% Empty Individual Template
% empty_individual.Position = [];
% empty_individual.Cost = [];
empty_individual.Position.X = [];
empty_individual.Position.Y = [];
empty_individual.Position.Z = [];
empty_individual.Features.U_id = [];
empty_individual.Features.Speed = [];
empty_individual.Cluster.Group = [];
empty_individual.Cluster.Role = [];
empty_individual.Cluster.Target = [];
% Initialize Population Array
%pop = repmat(empty_individual, nPop, 1);


ArchMaxSize=ceil(Max_time/nPop)*nPop;
Arch_X = repmat(empty_individual, ArchMaxSize, 1);
Arch_F = ones(ArchMaxSize,objvNo)*inf;
Arch_mem_no = 0;
% Initialize Population Members
%for i=1:nPop
    %pop(i).Position = unifrnd(lb,ub,VarSize);
    pop = pop_init(UAV_data,params);
    Costs = zeros(nPop,objvNo);
%end
Time=1;

while Time<Max_time+1
    for i=1:nPop

        pop(i).Position.X = min(max(pop(i).Position.X, params.x_lb), params.x_ub);
        pop(i).Position.Y = min(max(pop(i).Position.Y, params.y_lb), params.y_ub);
        pop(i).Position.Z = min(max(pop(i).Position.Z, params.z_lb), params.z_ub);
        
        [pop(i),k] = UAV_clustering(pop(i), Target_data, params);
        
        Costs(i,:) = CostFunction(pop(i), Target_data, params, k);
        
    end
    % Sort Population
    Ranks = non_dominated_ranking(Costs,objvNo,nPop);
    [~,sorted_indexes]=sort(Ranks);
    sorted_Cost = Costs(sorted_indexes,:);
    
    for newindex=1:nPop
        Sorted_pop(newindex)=pop(sorted_indexes(newindex));
    end
    
    
    pop(1)= Sorted_pop(1);

%[pop,Costs,~] = SortPopulation(pop,Costs);

% Update Best Solution Ever Found
    BestSol = pop(1);


%% SFLA Main Loop

for it = 1:MaxIt

    fla_params.BestSol = BestSol;

    % Initialize Memeplexes Array
    Memeplex = cell(nMemeplex, 1);
    Memeplex_Costs = cell(nMemeplex, 1);

    % Form Memeplexes and Run FLA
    for j = 1:nMemeplex
        % Memeplex Formation
        Memeplex{j} = pop(I(j,:));
        Memeplex_Costs{j} = Costs(I(j,:),:);

        % Run FLA
        [Memeplex{j}, Memeplex_Costs{j}] = RunFLA(Memeplex{j}, Memeplex_Costs{j}, fla_params);
        
        % Insert Updated Memeplex into Population
        pop(I(j,:)) = Memeplex{j};
        Costs(I(j,:),:) = Memeplex_Costs{j};
    end
    
    [Arch_X,Arch_F, Arch_mem_no] = UpdateArchive(Arch_X,Arch_F,pop,Costs);
    
    if Arch_mem_no>ArchMaxSize
        Arch_mem_ranks=RankingProcess(Arch_F, ArchMaxSize, 2);
        [Arch_X,Arch_F,Arch_mem_ranks,Arch_mem_no]=...
  HandleFullArchive(Arch_X,Arch_F,Arch_mem_no,Arch_mem_ranks,ArchMaxSize);
    else
        Arch_mem_ranks=RankingProcess(Arch_F, ArchMaxSize, 2);
    end
    index=max(RouletteWheelSelection(1./Arch_mem_ranks),1);
  
    BestSol=Arch_X(index);
    if isempty(Bestsol.Position.X)
       BestSol = pop(1);
    end
    
    % Sort Population
    %[pop,Costs,~] = SortPopulation(pop, Costs);
    
    %BestSol = pop(1);
    
end
end

toc
[~,idF] = min(sum(normc(Arch_F),2));
Final_F = Arch_F(idF,:);
Final_P = Arch_X(idF);

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




% PF = Arch_F;

% if objvNo == 2
% %     plot(TruePF(:,1), TruePF(:,2), '.');
% %     hold on;
%     plot(PF(:,1),PF(:,2), 'ko','LineWidth',2,'MarkerEdgeColor','k',...
%     'MarkerFaceColor','k','MarkerSize',6);
%     xlabel('F_1', 'FontSize',14)
%     ylabel('F_2', 'FontSize',14)
%     legend('True PF','Obtained PF', 'FontSize',14)
%     title(strcat('ZDT', num2str(TestNo)), 'FontSize',14)


% end
