function [salida_asociada,dispatch_factor] = leo_bloques_especiales(n_out,sys1)
% n_out: número de salidas
% sys1: diagrama de energy hu
blq = find_system(sys1,'BlockType','SubSystem');
blq = eliminar_subraices_directorios(blq,sys1);
j=size(blq,1);
contador=1;
for k=1:j
    Ud = get_param(char(blq(k)),'UserData');
    if(isfield(Ud,'Bloque_especial')==1)
        aux_n_out=get_param(char(blq(k)),'n_salida');
        aux_n_out=aux_n_out(2:size(aux_n_out,2)-1);
        aux_n_out=split(aux_n_out,' ');
        for cx=1:size(aux_n_out,1)
        aux_n_out_p=str2double(char(aux_n_out(cx)));
        if(aux_n_out_p==n_out)
            salida_asociada=aux_n_out_p;
            dispatch_factor=get_param(char(blq(k)),'dispatch_factor');
            contador=contador+1;
            break;
        end
        end
    end
end
if(contador==1)%para que siempre devuelva algo
    salida_asociada=0;
    dispatch_factor='nada';
end
end