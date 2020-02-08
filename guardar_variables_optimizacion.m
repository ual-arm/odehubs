%Guardo las entradas
for k=1:n_in
    eval(strcat('input',string(k),'(t,1)=value(PP',string(k),'(t));'));
end
%Varias salidas, solo una activa
try
    clear aux;
    [blq] = obtener_blq_binarios(sys1);
    for k=1:size(blq,1)
        eval(strcat('aux=',get_param(char(blq(k)),'Salida_Binaria')));
        aux1=size(aux,2);
        for i=1:aux1
            auxiliar=strcat('S_Bin',string(aux(i)));%que hace auxiliar?
            eval(strcat(auxiliar,'(t,1)=value(jpn',string(aux(i)),'(t));'));
        end
    end
end

%Guardo lo relacionado a almacenaje
for k=1:n_out
    eval(strcat('Ech',string(k),'(t,1)=value(bin_carga',string(k),'(t));'));
    eval(strcat('Edis',string(k),'(t,1)=value(bin_descarga',string(k),'(t));'));
    eval(strcat('store',string(k),'(t+1)=value(store',string(k),'(t+1));'));
    eval(strcat('Qchp',string(k),'(t,1)=value(Qch',string(k),'(t));'));%hay que almacenarlos de alguna manera, sino desaparece a no ser que asignemos value
    eval(strcat('Qdisp',string(k),'(t,1)=value(Qdis',string(k),'(t));'));
    eval(strcat('almacenaje',string(k),'(t)=value(store',string(k),'(t));'));
end
%Guardo los dispatch factors
aux_m=0;
try
    aux_m=size(vector_df,2);
end
if(aux_m>0)
    for k=1:aux_m
        if(strcmp(vector_df(k).p,'1')==0)
            eval(strcat(vector_df(k).p,'p(t)=value(',vector_df(k).p,'(t));'));
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
            eval(strcat(v_beta(k).p,'p(t)=value(',v_beta(k).p,'(t));'));
        end
    end
end
%Alargo una muestra para simulación
if(horizonte==1)
    aux_m=0;
    aux_mm=0;
    try
        aux_m=size(vector_df,2);
    end
    if(aux_m>0)
        for k=1:aux_m
            if(strcmp(vector_df(k).p,'1')==0)
                eval(strcat(vector_df(k).p,'p(t+1)=value(',vector_df(k).p,'(t));'));
            end
        end
    end
    try
        aux_mm = size(v_beta,2);
    end
    if(aux_mm>0)
        for k=1:aux_mm
            if(strcmp(v_beta(k).p,'1')==0)
                eval(strcat(v_beta(k).p,'p(t+1)=value(',v_beta(k).p,'(t));'));
            end
        end
    end
    for k=1:n_out
        eval(strcat('Ech',string(k),'(t+1,1)=value(bin_carga',string(k),'(t));'));
        eval(strcat('Edis',string(k),'(t+1,1)=value(bin_descarga',string(k),'(t));'));
        eval(strcat('Qchp',string(k),'(t+1,1)=value(Qch',string(k),'(t));'));%hay que almacenarlos de alguna manera, sino desaparece a no ser que asignemos value
        eval(strcat('Qdisp',string(k),'(t+1,1)=value(Qdis',string(k),'(t));'));
    end
    for k=1:n_in
        eval(strcat('input',string(k),'(t+1,1)=value(PP',string(k),'(t));'));
    end
    %varias salidas, solo una activa
    try
        clear aux;
        [blq] = obtener_blq_binarios(sys1);
        for k=1:size(blq,1)
            eval(strcat('aux=',get_param(char(blq(k)),'Salida_Binaria')));
            aux1=size(aux,2);
            for i=1:aux1
                auxiliar=strcat('S_Bin',string(aux(i)));
                eval(strcat(auxiliar,'(t+1,1)=value(jpn',string(aux(i)),'(t));'));
            end
        end
    end
end