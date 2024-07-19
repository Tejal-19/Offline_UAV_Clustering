function  UAV_data = UAVs_generation(params)

N_UAV_data = params.N_UAV_data;
UAV_mode = params.UAV_mode;

UAV_data_empty.Features.U_id = [];
UAV_data_empty.Features.Speed = [];
UAV_data = repmat(UAV_data_empty, N_UAV_data, 1);

Sp_min = params.Sp_min;
Sp_max = params.Sp_max;

Sp = rand().*(Sp_max-Sp_min)+Sp_min;

for i=1:N_UAV_data
    
    if strcmp(UAV_mode, 'Heterogeneous')
        UAV_data(i).Features.Speed = rand().*(Sp_max-Sp_min)+Sp_min;
    else
        UAV_data(i).Features.Speed = Sp;
    end
    
    UAV_data(i).Features.U_id = i;
    
end

end

