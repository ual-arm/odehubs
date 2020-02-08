%Obtiene las retricciones que son invariantes con el tiempo
function [Res,cont,cpa,rd_carga,rd_descarga,alm_inic,bin_almacenamiento] = restricciones_fijas(n_out,n_in,sys,sys1,T_m)
% n_out: número de recursos de salida
% n_in: número de recursos de entrada
% sys: diagrama de Simulink
% sys1: diagrama del energy hub
% T_m: tiempo de muestreo
% Res: restricciones
% cont: contador de restricciones
% cpa: coeficiente de pérdidas de almacenamiento
% rd_carga: coeficiente de pérdidas de carga
% rd_descarga: coeficiente de pérdidas de descarga
% alm_inic: almacenamiento inicial
%bin_almacenamiento: Si hay o no almacenamiento de energía

%Restricciones de almacenamiento
%Ordeno bloques subsystem almacenamiento
bloques_almacen=ordenar_bloques_almacen(sys1);
cont=1;
for k=1:n_out
    fijo_variables_bloque_almacen(k,char(bloques_almacen(k)),T_m);
    %Leo parámetros almacenamiento
    min_almacen=get_param(char(bloques_almacen(k)),'Low1');
    max_almacen=get_param(char(bloques_almacen(k)),'Up1');
    min_flujo_c=get_param(char(bloques_almacen(k)),'Low2');
    max_flujo_c=get_param(char(bloques_almacen(k)),'Up2');
    min_flujo_d=get_param(char(bloques_almacen(k)),'Low');
    max_flujo_d=get_param(char(bloques_almacen(k)),'Up');
    alm_inic(k) = str2double(get_param(char(bloques_almacen(k)),'almacenamiento_inicial'));
    cpa(k)=str2double(get_param(char(bloques_almacen(k)),'cpa'));
    rd_carga(k)=str2double(get_param(char(bloques_almacen(k)),'r_c'));
    rd_descarga(k)=str2double(get_param(char(bloques_almacen(k)),'r_d'));
    aux=get_param(char(bloques_almacen(k)),'almacenamiento_on');
    if(strcmp(aux,'on')==1)
        bin_almacenamiento(k) = 1;
    else
        bin_almacenamiento(k) = 0;
    end
    %Restricciones de flujo
    aux=strcat('store',string(k));
    Res(cont).p = strcat(min_almacen,'<=',aux,'<=',max_almacen);
    cont=cont+1;
    Res(cont).p=strcat(min_flujo_c,'<=Qch',string(k),'<=',max_flujo_c);
    cont=cont+1;
    Res(cont).p=strcat(min_flujo_d,'<=Qdis',string(k),'<=',max_flujo_d);
    cont=cont+1;
    %Creo la restricción para evitar carga y descarga simultánea
   if(bin_almacenamiento(k)==1)
       aux_binario1=strcat('bin_carga',string(k));
       aux_binario2=strcat('bin_descarga',string(k));
       Res(cont).p = strcat('0<=',aux_binario1,'+',aux_binario2,'<=1');
       cont=cont+1;
   end
end

%Restricciones mínimas de recursos de entrada
for k=1:n_in
    Res(cont).p = strcat('0<=PP',string(k));
    cont=cont+1;
end

%Restricciones de aparatos con varias salidas y solo una activa
try
    [blq_bin] = obtener_blq_binarios(sys1);
    for k=1:size(blq_bin,2)
        aux_bloque=char(blq_bin(k));
        aux_bin=get_param(aux_bloque,'Salida_Binaria');
        eval(strcat('aux2=',aux_bin));
        aux_size=size(aux2,2);
        auxiliar=[];
        for kk=1:aux_size
            if(kk==1)
                auxiliar=[auxiliar strcat('jpn',string(aux2(kk)))];
            else
                if(kk==aux_size)
                    auxiliar=[strcat('0<=',auxiliar,'+jpn',string(aux2(kk)),'<=1')];
                else
                    auxiliar=[strcat(auxiliar,'+jpn',string(aux2(kk)))];
                end
            end
        end
        Res(cont).p = auxiliar;
        cont=cont+1;
    end
end
end
