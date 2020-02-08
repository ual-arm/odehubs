function [blq_especial,blq_aux,contador] = obtener_blq_especial(sys1)
blq = find_system(sys1,'BlockType','SubSystem');
blq = eliminar_subraices_directorios(blq,sys1);
%Busco los que tienen el parámetro Store_system en la propiedad 'UserData'
j=size(blq,1);
contador=1;
for k=1:j
    Ud = get_param(char(blq(k)),'UserData');
    if(isfield(Ud,'Bloque_especial')==1)
        blq_aux(contador)=blq(k);
        contador=contador+1;
    end
end
if(contador>1)
    blq_especial=1;
else
    blq_especial=0;
    blq_aux=0;
end