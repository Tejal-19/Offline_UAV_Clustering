function [index] = initmed(U_id, Du, Dt ,k)

ranks = non_dominated_ranking([Du, Dt], 2, length(U_id));
[~,index] = sort(ranks, 'ascend');
index=index(1:k,:);
% med=U_id(index,:);

end