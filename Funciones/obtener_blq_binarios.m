%Devuelve los bloques que tienen una salida activada y varias salidas
function [blq_aux] = obtener_blq_binarios(sys1)
% Sys1: Diagrama del energy hub
% Blq_aux: bloques con una salida activada

blq = find_system(sys1,'BlockType','SubSystem');
blq = eliminar_subraices_directorios(blq,sys1);
j=size(blq,1);
contador=1;
for k=1:j
    Ud = get_param(char(blq(k)),'UserData');
    if(isfield(Ud,'Bloque_binario')==1)
        blq_aux(contador)=blq(k);
        contador=contador+1;
    end
end