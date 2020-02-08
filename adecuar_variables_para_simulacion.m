%Creo timeseries
for k=1:n_out
    eval(strcat('Ech',string(k),'=timeseries(Ech',string(k),',0:T_muestreo:aux_Out*T_muestreo);'));
    eval(strcat('Edis',string(k),'=timeseries(Edis',string(k),',0:T_muestreo:aux_Out*T_muestreo);'));
    eval(strcat('Qchp',string(k),'=timeseries(Qchp',string(k),',0:T_muestreo:aux_Out*T_muestreo);'));
    eval(strcat('Qdisp',string(k),'=timeseries(Qdisp',string(k),',0:T_muestreo:aux_Out*T_muestreo);'));
end
for k=1:n_in
    eval(strcat('Input',string(k),'=timeseries(input',string(k),',0:T_muestreo:aux_Out*T_muestreo);'));
end
%Retomo nomenclatura dispatch factors de simulación
aux_m=0;
try
    aux_m=size(vector_df,2);
end
if(aux_m>0)
    for k=1:aux_m
        if(strcmp(vector_df(k).p,'1')==0)
            eval(strcat(vector_df(k).p,'=',vector_df(k).p,'p;'));
        end
    end
end

aux_mm=0;
try
        aux_mm = size(v_beta,2);
end
if(aux_mm>0)
    for k=1:aux_mm
        if(strcmp(v_beta(k).p,'1')==0)
            eval(strcat(v_beta(k).p,'=',v_beta(k).p,'p;'));
        end
    end
end
%Varias salidas, una activa únicamente
try
    clear aux;
    [blq] = obtener_blq_binarios(sys1);
    for k=1:size(blq,1)
        eval(strcat('aux=',get_param(char(blq(k)),'Salida_Binaria')));
        aux1=size(aux,2);
        auxiliar_total=[];
        for i=1:aux1
            auxiliar=strcat('S_Bin',string(aux(i)));
            auxiliar1=strcat('SS_Bin',string(aux(i)));
            eval(strcat(auxiliar1,'=timeseries(value(',auxiliar,'),0:T_muestreo:aux_Out*T_muestreo);'));
            auxiliar_total=strcat(auxiliar_total,'-',auxiliar1);
        end
        auxiliar_total=strcat('[',auxiliar_total,']');
        auxiliar_total=replace(auxiliar_total,'-',' ');
        set_param(char(blq(k)),'Salida_Binaria',auxiliar_total);
    end
end
