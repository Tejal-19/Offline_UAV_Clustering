function [index]=updatemed(D_u_u,D_u_t,cid,index,k)

l=zeros(k,1);
% med=zeros(k,1);

for ik = 1:k
    ind = find(cid==ik);
    l(ik) = length(ind);
    if l(ik)>0
        sdist = sum(D_u_u(ind,ind))';
        tdist = sum(D_u_t(ind,:),2);
%         xa=U_id(ind,:);
        ranks = non_dominated_ranking([sdist, tdist], 2, length(ind));
        [~,indms] = sort(ranks, 'ascend');
%         med(ik,:) = xa(indms(1));
        index(ik) = ind(indms(1));
    end
end

end

