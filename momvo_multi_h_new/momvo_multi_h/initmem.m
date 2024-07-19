function [sdist, cid, dist] = initmem(D_u_u,index)

dist = D_u_u(index,:);

[nearest,cid] = min(dist);
sdist = sum(normr(nearest));

end