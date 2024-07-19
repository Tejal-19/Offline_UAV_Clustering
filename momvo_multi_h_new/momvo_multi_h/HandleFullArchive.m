
function [Arch_X_Chopped,Arch_F_Chopped,Arch_mem_ranks_updated,Arch_member_no]=HandleFullArchive(Arch_X,Arch_F,Arch_member_no,Arch_mem_ranks,ArchiveMaxSize)

for i=1:size(Arch_F,1)-ArchiveMaxSize

    index = max(RouletteWheelSelection(Arch_mem_ranks),1);
    
    Arch_X=[Arch_X(1:index-1) ; Arch_X(index+1:Arch_member_no)];
    Arch_F=[Arch_F(1:index-1,:) ; Arch_F(index+1:Arch_member_no,:)];
    Arch_mem_ranks=[Arch_mem_ranks(1:index-1) Arch_mem_ranks(index+1:Arch_member_no)];
    Arch_member_no=Arch_member_no-1;
end

Arch_X_Chopped=Arch_X;
Arch_F_Chopped=Arch_F;
Arch_mem_ranks_updated=Arch_mem_ranks;

end