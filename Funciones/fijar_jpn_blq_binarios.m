% Fijo en la máscara de los bloques que tienen varias salidas y solo una de ellas activas los indices para nombrar jpn: jpn1, jpn2, etc.
function [] = fijar_jpn_blq_binarios(sys1)
%sys1: diagrama del energy hub

SS=find_system(sys1,'BlockType','SubSystem');
SF=find_system(sys1,'BlockType','S-Function');
[aux_contador] = fijar_indices_jpn(SF,1);
[aux_contador] = fijar_indices_jpn(SS,aux_contador);
end
%Función local
function [aux_cont] = fijar_indices_jpn(SF,aux_cont)
for k=1:size(SF,1)
    Ud=get_param(SF(k),'UserData');
    if(isfield(Ud{:},'Bloque_binario')==1)
        aux_efficiency=get_param(char(SF(k)),'LineHandles');
        aux_size=size(aux_efficiency.Outport,2);
        parametro_salida_binaria=[];
        for j=1:aux_size
            if(j>1)
                parametro_salida_binaria=strcat(parametro_salida_binaria,'-',char(string(aux_cont)));
            else
                parametro_salida_binaria=char(string(aux_cont));
            end
            aux_cont=aux_cont+1;
        end
        parametro_salida_binaria=strcat('[ ',replace(parametro_salida_binaria,'-',' '),']');
        set_param(char(SF(k)),'Salida_Binaria',parametro_salida_binaria);
    end
end
end