clear all;
close all;
clc
% Parámetros a introducir
run('Inicializacion_datos_microred')%script con los datos para inicialización
addpath('Funciones');
addpath(genpath('Archivos_de_terceros(descomprimir)'))

%Obtención de la fecha
fecha_inicio=[2014,03,18];%es necesario definir una fecha inicio, con posibilidad de añadir horas, minutos y segundos
fecha_limite=horizonte;%coge el valor del dato horizonte como límite superior para la creación del vertor de fechas
fecha_aux=datetime(fecha_inicio) + caldays(0:T_muestreo:fecha_limite);%crea el vector de fechas en datos datetime
fecha=datestr(datenum(fecha_aux));%convierte el vector de fechas de datos datetime en un vector de arrays de carácteres
%Con la variable 'fecha', se define en el script
%'adecuar_variables_para_simulacion' el vector de tiempo de las variables timeseries

load_system(sys);%abro diagrama de Simulink
tic
aux_Out=size(Out,2);%tamaño del vector de demandas
[n_in,n_out]=manejo_puertos_EH(sys1);%obtengo la cantidad de puertos entrada/salida
fijar_jpn_blq_binarios(sys1);%fijo números para bloques con una única salida activa

%Obtengo restricciones de los bloques de almacenamiento, recursos de
%entrada y aparatos con varias salidas, siendo solo una activa
[Restricciones_fijas,cont,cpa,rd_carga,rd_descarga, ... 
    alm_inic,bin_almacenamiento] = restricciones_fijas(n_out,n_in,sys,sys1,T_muestreo);

try%No tiene porqué haber coeficientes de reparto
    %Obtengo las restricciones de los coeficientes de reparto
    [Restricciones_fijas,cont,vector_df] = restricciones_df(Restricciones_fijas,cont,sys1);
catch
    warning('No existen coeficientes de reparto');
end
cont=cont-1;

%Obtengo los caminos
[Path,Path_Restricciones]=obtener_paths(sys1);

%Creo variables de decision para YALMIP
aux_variables_decision=aux_Out+horizonte+2;
for k=1:n_in
    if(v_renovable(k)==0)
        eval(strcat('PP',string(k),'=sdpvar(1,',string(aux_variables_decision),');'));
    else
        eval(strcat('PP',string(k),'=Renovable',string(k),';'));%recurso renovable, no variable de decisión
    end
end
%variables de almacenamiento
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
%Coeficientes de reparto
try
    j=size(vector_df,2);
    for k=1:j
        if(strcmp(vector_df(k).p,'1')==0)
            eval(strcat(vector_df(k).p,'=sdpvar(1,',string(aux_variables_decision),');'))
        end
    end
end
%Para el on/off de los dispositivos sin alfa
try
    [v_beta]=obtener_coeficientes_beta(sys1);
    j=size(v_beta,2);
    for k=1:j
        eval(strcat(v_beta(k).p,'=binvar(1,',string(aux_variables_decision),');'));
    end
end
try %Bloques con una sola salida activa
    clear aux;
    [blq] = obtener_blq_binarios(sys1);
    for k=1:size(blq,1)
        eval(strcat('aux=',get_param(char(blq(k)),'Salida_Binaria'),';'));
        aux1=size(aux,2);
        for i=1:aux1
            auxiliar=strcat('jpn',string(aux(i)));
            eval(strcat(string(auxiliar),'=binvar(1,',string(aux_variables_decision),');'));
        end
    end
end
% Almaceno las restricciones
Constraintss=[];
for k=1:cont
    Constraintss=[Constraintss eval(Restricciones_fijas(k).p)];
end

% Algoritmo horizonte deslizante variable
t=1;
while(t<=aux_Out)
    c_r=1;
    %Acorto las muestras
    if(horizonte+t-2==aux_Out)
        horizonte=horizonte-1;
    end
    %Obtengo restricciones que dependen del horizonte 
    [Restricciones,c_r] = restricciones_flujo(c_r,Path_Restricciones,Path);
    [Restricciones,c_r] = ecuaciones_caminos(Restricciones,c_r,horizonte,n_out,n_in,Path,Out,t,rd_carga,rd_descarga,sys1);
    [Restricciones,c_r]=obtener_ecuaciones_almacenamiento(n_out,c_r,horizonte,Restricciones,cpa,rd_carga,rd_descarga);
    %Recupero restricciones almacenadas y evalúo todas para YALMIP
    Constraints=[Constraintss];
    for k=1:c_r-1
        Constraints=[Constraints eval(Restricciones(k).p)];
    end
    % OPTIMIZACIÓN: uso YALMIP, BARON y CPLEX
    options=sdpsettings('solver','baron');
    optimize(Constraints,eval(crear_objetivo(horizonte,n_in)),options);
    %Guardo las variables de optimización, las variables YALMIP se sobreescriben
    run('guardar_variables_optimizacion')
    t=t+1;
end
%Renombro y acomodo las variables para simular
run('adecuar_variables_para_simulacion')
toc
% Resultados de coste de adquisición de recursos
precio=0;
for t=1:n_in
    eval(strcat('precio=F(t,:)*Input',string(t),'.data(1:aux_Out)+precio;'));
end
precio