
function [pop, Costs, SortOrder] = SortPopulation(pop, Costs)

    Ranks = non_dominated_ranking(Costs,2,size(Costs,1));
    % Sort the Costs Vector
    [~, SortOrder]=sort(Ranks); %sort(sum(Costs,2));
    
    % Apply the Sort Order to Population
    pop = pop(SortOrder);
    Costs = Costs(SortOrder,:);

end