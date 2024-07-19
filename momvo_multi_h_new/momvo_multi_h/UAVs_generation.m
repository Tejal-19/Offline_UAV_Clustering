function  UAV_data = UAVs_generation(params)

N_UAV_data = params.N_UAV_data;
UAV_mode = params.UAV_mode;

UAV_data_empty.Features.U_id = [];
UAV_data_empty.Features.Speed = [];
UAV_data_empty.Features.Transmit_range = [];
UAV_data_empty.Features.B_capacity = [];
UAV_data_empty.Features.B_charging = [];
UAV_data = repmat(UAV_data_empty, N_UAV_data, 1);

Sp_min = params.Sp_min;
Sp_max = params.Sp_max;
Tr_min = params.Tr_min;
Tr_max = params.Tr_max;
Bcp_min = params.Bcp_min;
Bcp_max = params.Bcp_max;
Bch_min = params.Bch_min;
Bch_max = params.Bch_max;

Sp = rand().*(Sp_max-Sp_min)+Sp_min;
Tr = rand().*(Tr_max-Tr_min)+Tr_min;
Bcp = rand().*(Bcp_max-Bcp_min)+Bcp_min;
Bch = rand().*(Bch_max-Bch_min)+Bch_min;

for i=1:N_UAV_data
    
    if strcmp(UAV_mode, 'Heterogeneous')
        UAV_data(i).Features.Speed = rand().*(Sp_max-Sp_min)+Sp_min;
        UAV_data(i).Features.Transmit_range = rand().*(Tr_max-Tr_min)+Tr_min;
        UAV_data(i).Features.B_capacity = rand().*(Bcp_max-Bcp_min)+Bcp_min;
        UAV_data(i).Features.B_charging = rand().*(Bch_max-Bch_min)+Bch_min;
    else
        UAV_data(i).Features.Speed = Sp;
        UAV_data(i).Features.Transmit_range = Tr;
        UAV_data(i).Features.B_capacity = Bcp;
        UAV_data(i).Features.B_charging = Bch;
    end
    
    UAV_data(i).Features.U_id = i;
    
end

end

