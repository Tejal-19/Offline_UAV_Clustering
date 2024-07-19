

function [pop, Costs] = RunFLA(pop, Costs, params)

    %% FLA Parameters
    q = params.q;           % Number of Parents
    alpha = params.alpha;   % Number of Offsprings
    beta = params.beta;     % Maximum Number of Iterations
    sigma = params.sigma;
    CostFunction = params.CostFunction;
    x_lb = params.x_lb;
    x_ub = params.x_ub;
    y_lb = params.y_lb;
    y_ub = params.y_ub;
    z_lb = params.z_lb;
    z_ub = params.z_ub;
    
    
    VarSize = size(pop(1).Position);
    BestSol = params.BestSol;
    
    nPop = numel(pop);      % Population Size
    
    % Calculate Population Range (Smallest Hypercube)
    LowerBound = pop(1).Position;
    UpperBound = pop(1).Position;
    for i = 2:nPop
        LowerBound = min(LowerBound, pop(i).Position);
        UpperBound = max(UpperBound, pop(i).Position);
    end

    
    
    %% FLA Main Loop

    for it = 1:beta
        
        % Select Parents
        L = randsample(nPop,q); %randsample(P,q);
        B = pop(L);
        B_Cost = Costs(L,:);
        
        % Generate Offsprings
        for k=1:alpha
            
            % Sort Population
            [B, B_Cost, SortOrder] = SortPopulation(B,B_Cost);
            L = L(SortOrder);
            
            % Flags
            ImprovementStep2 = false;
            Censorship = false;
            
            % Improvement Step 1
            NewSol1 = B(end);
            Step1 = sigma*rand(VarSize).*(B(1).Position-B(end).Position);
            NewSol1.Position = B(end).Position + Step1;
        
            if IsInRange(NewSol1.Position, x_lb, x_ub) && IsInRange(NewSol1.Position, y_lb, y_ub) && IsInRange(NewSol1.Position, z_lb, z_ub) 
                NewSol1_Cost = CostFunction(NewSol1.Position);
                if dominates(NewSol1_Cost,B_Cost(end,:))
                    B(end) = NewSol1;
                    B_Cost(end,:) = NewSol1_Cost;
                else
                    ImprovementStep2 = true;
                end
            else
                ImprovementStep2 = true;
            end
            
            % Improvement Step 2
            if ImprovementStep2
                NewSol2 = B(end);
                Step2 = sigma*rand(VarSize).*(BestSol.Position-B(end).Position);
                NewSol2.Position = B(end).Position + Step2;
                
    if IsInRange(NewSol2.Position, x_lb, x_ub ) && IsInRange(NewSol1.Position, y_lb, y_ub) && IsInRange(NewSol1.Position, z_lb, z_ub) 
                    NewSol2_Cost = CostFunction(NewSol2.Position);
                    if dominates(NewSol2_Cost,B_Cost(end,:))
                        B(end) = NewSol2;
                        B_Cost(end,:) = NewSol2_Cost;
                    else
                        Censorship = true;
                    end
                else
                    Censorship = true;
    end
                
            end
           
                
            % Censorship
            if Censorship
                B(end).Position = unifrnd(LowerBound, UpperBound);
                B_Cost(end,:) = CostFunction(B(end).Position);
            end
            
        end
        
        % Return Back Subcomplex to Main Complex
        pop(L) = B;
        Costs(L,:) = B_Cost;
        
    end
    
end