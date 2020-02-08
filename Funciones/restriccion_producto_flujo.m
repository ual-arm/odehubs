%Obtiene la restriccion del flujo
function [Restriccion] = restriccion_producto_flujo(path,bloque,k,parametro,signo)
%path: camino
%bloque: bloque de interés
%k: posición de celda dentro del vector camino
%parámetro: Max_in ó Min_in
%signo: <= ó >=
param=get_param(bloque,parametro);
producto = producto_flujo(path,k);
producto=replace(producto,'P','PP');
Restriccion=strcat(param,signo,producto);
end