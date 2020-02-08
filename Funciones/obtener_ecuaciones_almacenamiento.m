%Crea la ecuación de almacenamiento para los instantes de muestreo que se
%necesiten dependendiendo del horizonte
function [R,c] = obtener_ecuaciones_almacenamiento(n_out,c,horizonte,R,cpa,rd_carga,rd_descarga)
% n_out: número de recursos de salida
% c: contador de restricciones
% horizonte: muestras a optimizar
% R: Restricciones
% cpa: coeficiente de pérdidas de almacenamiento
% rd_carga: coeficiente de pérdidas de carga
% rd_descarga: coeficiente de pérdidas de descarga
for k=1:n_out
    aux_ecuacion=strcat('store',string(k),'(y)==',string(rd_carga(k)),'*Qch', ...
        string(k),'(t)*bin_carga',string(k),'(t)-',string(rd_descarga(k)),'*Qdis', ...
        string(k),'(t)*bin_descarga',string(k),'(t)+',string(cpa(k)),'*store',string(k),'(t)');
    for kk=1:horizonte
        aux_ecuacion1=replace(aux_ecuacion,'(t)',strcat('(t+',string(kk-1),')'));
        R(c).p=replace(aux_ecuacion1,'(y)',strcat('(t+',string(kk),')'));
        c=c+1;
    end
end
end