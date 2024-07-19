function Rank=non_dominated_ranking(Cost,nobj,nPop)

front = 1;
F(front).f = [];
individual = [];


for i = 1 : nPop
    individual(i).n = 0;
    individual(i).p = [];
    for j = 1 : nPop
        dom_less = 0;
        dom_equal = 0;
        dom_more = 0;
          for k = 1 : nobj
            if (Cost(i,k) < Cost(j,k))
                dom_less = dom_less + 1;
            elseif (Cost(i,k) == Cost(j,k))
                dom_equal = dom_equal + 1;
            else
                dom_more = dom_more + 1;
            end
          end
         if dom_less == 0 && dom_equal ~= nobj
            individual(i).n = individual(i).n + 1;
        elseif dom_more == 0 && dom_equal ~= nobj
            individual(i).p = [individual(i).p j];
        end
    end  
    if individual(i).n == 0
        Rank(i,:)=1;
        F(front).f = [F(front).f i];
    end
end
while ~isempty(F(front).f)
   Q = [];
   for i = 1 : length(F(front).f)
       if ~isempty(individual(F(front).f(i)).p)
        	for j = 1 : length(individual(F(front).f(i)).p)
            	individual(individual(F(front).f(i)).p(j)).n = ...
                	individual(individual(F(front).f(i)).p(j)).n - 1;
        	   	if individual(individual(F(front).f(i)).p(j)).n == 0
               		Rank(individual(F(front).f(i)).p(j),:) = ...
                        front + 1;
                    Q = [Q individual(F(front).f(i)).p(j)];
                end
            end
       end
   end
   front =  front + 1;
   F(front).f = Q;
end

end