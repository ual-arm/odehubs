%Obtengo todos los coeficientes binarios que serán beta
function [vector] = obtener_coeficientes_beta(sys1)
%vector: coeficientes beta
%sys1: diagrama del energy hub
SS=find_system(sys1,'BlockType','SubSystem');
SF=find_system(sys1,'BlockType','S-Function');

contador=1;
if(size(SS,1)>0)
    for k=1:size(SS,1)
        Ud=get_param(SS(k),'UserData');
        if(isfield(Ud{:},'Ndevice')==1)
            coef_reparto=char(get_param(SS(k),'dispatch_factor'));
            if(strcmp(coef_reparto,'1')==0)
                if(strcmp(coef_reparto(1:4),'beta')==1)
                    vector(contador).p=coef_reparto;
                    contador=contador+1;
                end
            end
        end
    end
end
if(size(SF,1)>0)
    for k=1:size(SF,1)
        Ud=get_param(SF(k),'UserData');
        if(isfield(Ud{:},'Ndevice')==1)
            coef_reparto=char(get_param(SF(k),'dispatch_factor'));
            if(strcmp(coef_reparto,'1')==0)
                if(strcmp(coef_reparto(1:4),'beta')==1)
                    vector(contador).p=coef_reparto;
                    contador=contador+1;
                end
            end
        end
    end
end
end