%Obtiene la restriccion del flujo
function [Restriccion] = restriccion_producto_flujo(path,bloque,k,parametro,signo)
%path: camino
%bloque: bloque de inter�s
%k: posici�n de celda dentro del vector camino
%par�metro: Max_in � Min_in
%signo: <= � >=
param=get_param(bloque,parametro);
producto = producto_flujo(path,k);
producto=replace(producto,'P','PP');
Restriccion=strcat(param,signo,producto);
end