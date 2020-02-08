%Ordena los bloques almacen en función de la coordenada y
function [blq_aux] = ordenar_bloques_almacen(sys1)
% blq_aux: bloques almacenamiento ordenados
% sys1: diagrama del energy hub
blq = find_system(sys1,'BlockType','SubSystem');
blq = eliminar_subraices_directorios(blq,sys1);
%Busco los que tienen el parámetro Store_system en la propiedad 'UserData'
j=size(blq,1);
contador=1;
for k=1:j
    Ud = get_param(char(blq(k)),'UserData');
    if(isfield(Ud,'Store_system')==1)
        blq_aux(contador)=blq(k);
        contador=contador+1;
    end
end

%Ordeno según la posición y
j=size(blq_aux,2);
for k=1:j-1
    for i=k:j
        pos1 = get_param(char(blq_aux(k)),'Position');
        pos2 = get_param(char(blq_aux(i)),'Position');
        if(pos1(2)>pos2(2))
            aux=blq_aux(k);
            blq_aux(k)=blq_aux(i);
            blq_aux(i)=aux;
        end
    end
end
end