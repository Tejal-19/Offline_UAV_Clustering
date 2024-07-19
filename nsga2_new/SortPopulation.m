
function [pop, Cost, F]=SortPopulation(pop, Cost)

    % Sort Based on Crowding Distance
    [~, CDSO] = sort([pop.CrowdingDistance],'descend');
    pop = pop(CDSO);
    Cost = Cost(CDSO,:);
    
    % Sort Based on Rank
    [~, RSO] = sort([pop.Rank]);
    pop = pop(RSO);
    Cost = Cost(RSO,:);
    
    % Update Fronts
    Ranks = [pop.Rank];
    MaxRank = max(Ranks);
    F = cell(MaxRank,1);
    for r = 1:MaxRank
        F{r} = find(Ranks==r);
    end

end