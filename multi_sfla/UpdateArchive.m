function [Arch_X_updated,Arch_F_updated,Arch_mem_no] = UpdateArchive(Arch_X,Arch_F,pop,Costs)


Arch_X_temp = [Arch_X; pop];
Arch_F_temp = [Arch_F; Costs];

o = zeros(1,size(Arch_F_temp,1));

for i=1:size(Arch_F_temp,1)
    o(i)=0;
    for j=1:i-1
        if any(Arch_F_temp(i,:) ~= Arch_F_temp(j,:))
            if dominates(Arch_F_temp(i,:),Arch_F_temp(j,:))
                o(j) = 1;
            elseif dominates(Arch_F_temp(j,:),Arch_F_temp(i,:))
                o(i) = 1;
                break;
            end
        else
            o(j) = 1;
            o(i) = 1;
        end
    end
end

Arch_mem_no = 0;
index = 0;
for i=1:size(Arch_F_temp,1)
    if o(i)==0
        Arch_mem_no = Arch_mem_no+1;
        Arch_X_updated(Arch_mem_no) = Arch_X_temp(i);
        Arch_F_updated(Arch_mem_no,:) = Arch_F_temp(i,:);
    else
        index=index+1;
    end
end

if ~exist('Arch_X_updated','var')
Arch_X_updated = Arch_X;
Arch_F_updated = Arch_F;
Arch_mem_no = size(Arch_X,1);
else
    Arch_X_updated = Arch_X_updated(:);
end
