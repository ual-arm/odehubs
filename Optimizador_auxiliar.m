load_system(sys);
aux_Out=size(Out,2);
%%
[n_in,n_out]=manejo_puertos_EH(sys1);

%fijo numeros para jpn en mascaras bloques binarios
fijar_jpn_blq_binarios(sys1);

[Restricciones_fijas,cont,cpa,rd_carga,rd_descarga,alm_inic,bin_almacenamiento] = restricciones_fijas(n_out,n_in,sys,sys1,T_muestreo);
try%no tiene porqué haber df
    [Restricciones_fijas,cont,vector_df] = restricciones_df(Restricciones_fijas,cont,sys1);
catch
    warning('No existen coeficientes de reparto');
end
cont=cont-1;%sale como cont=cont+1;

%obtengo caminos
[Path,Path_Restricciones]=obtener_paths(sys1);

%creo variables de decision
aux_variables_decision=aux_Out+horizonte+2;
for k=1:n_in
    if(v_renovable(k)==0)
        eval(strcat('PP',string(k),'=sdpvar(1,',string(aux_variables_decision),');'));
    else
        eval(strcat('PP',string(k),'=Renovable',string(k),';'));%recurso renovable
    end
end
for k=1:n_out
    if(bin_almacenamiento(k)==1)
        eval(strcat('bin_carga',string(k),'=binvar(1,',string(aux_variables_decision),');'));
        eval(strcat('bin_descarga',string(k),'=binvar(1,',string(aux_variables_decision),');'));
        eval(strcat('store',string(k),'=[',string(alm_inic(k)),', sdpvar(1,',string(aux_variables_decision),')];'));
        eval(strcat('Qch',string(k),'=sdpvar(1,',string(aux_variables_decision),');'))
        eval(strcat('Qdis',string(k),'=sdpvar(1,',string(aux_variables_decision),');'))
    else
        eval(strcat('Qch',string(k),'=zeros(1,',string(aux_variables_decision),');'))
        eval(strcat('Qdis',string(k),'=zeros(1,',string(aux_variables_decision),');'))
        eval(strcat('bin_carga',string(k),'=zeros(1,',string(aux_variables_decision),');'));
        eval(strcat('bin_descarga',string(k),'=zeros(1,',string(aux_variables_decision),');'));
        eval(strcat('store',string(k),'=[',string(alm_inic(k)),', zeros(1,',string(aux_variables_decision),')];'));
    end
end
%coeficientes de reparto
try
    j=size(vector_df,2);
    for k=1:j
        if(strcmp(vector_df(k).p,'1')==0)
            eval(strcat(vector_df(k).p,'=sdpvar(1,',string(aux_variables_decision),');'))
        end
    end
end
%para el on/off de los dispositivos sin alfa
try
    [v_beta]=obtener_coeficientes_beta(sys1);
    j=size(v_beta,2);
    for k=1:j
            eval(strcat(v_beta(k).p,'=binvar(1,',string(aux_variables_decision),');'));
    end
end
try
    clear aux;
    [blq] = obtener_blq_binarios(sys1);
    for k=1:size(blq,1)
        eval(strcat('aux=',get_param(char(blq(k)),'Salida_Binaria')));
        aux1=size(aux,2);
        for i=1:aux1
            auxiliar=strcat('jpn',string(aux(i)));
            eval(strcat(string(auxiliar),'=binvar(1,',string(aux_variables_decision),');'));
        end
    end
end
% creo las restricciones
Constraintss=[];
for k=1:cont
    Constraintss=[Constraintss eval(Restricciones_fijas(k).p)];
end

%% algoritmo horizonte deslizante
for t=1:aux_Out
    c_r=1;
    if(horizonte+t-2==aux_Out)
        horizonte=horizonte-1;
    end
    
    [Restricciones,c_r] = restricciones_flujo(c_r,Path_Restricciones,Path);
    [Restricciones,c_r] = ecuaciones_caminos(Restricciones,c_r,horizonte,n_out,n_in,Path,Out,t,rd_carga,rd_descarga);
    [Restricciones,c_r]=obtener_ecuaciones_almacenamiento(n_out,c_r,horizonte,Restricciones,cpa,rd_carga,rd_descarga);
    % creo las restricciones
    Constraints=[Constraintss];
    for k=1:c_r-1
        Constraints=[Constraints eval(Restricciones(k).p)];
    end
    t
    % OPTIMIZACION: uso yalmip, baron y cplex
    options=sdpsettings('solver','baron');
    optimize(Constraints,eval(crear_objetivo(horizonte,n_in)),options);
    run('guardar_variables_optimizacion')
end
% run('adecuar_variables_para_simulacion')

