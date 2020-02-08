%Elimino los bloques de los dispositivos enmascarados
function [bloques_libreria] = eliminar_subraices_directorios(bloques_libreria,system)
% bloques_libreria (entrada): nombre de los bloques a evaluar
% bloques_libreria (salida): nombres modificados
% system: nombre del diagrama hata donde preciso (EH)
i = size(bloques_libreria,1);
system=strcat(system,'/');
n_system=size(strfind(system,'/'),2); %tomo referencia en el segundo
aux_contador_eliminados=0;
for k=1:i
    aux_bloques=char(bloques_libreria(k-aux_contador_eliminados));
    y = size(aux_bloques,2);
    aux_contador=0;
    for j=1:y
        if(aux_bloques(j)=='/') %busco coincidencias
            aux_contador=aux_contador+1;
        end
    end
    if(aux_contador>n_system) %comparo con los del diagrama que quiero
        bloques_libreria(k-aux_contador_eliminados)=[];
        aux_contador_eliminados=aux_contador_eliminados+1; %necesario para evitar index exceed
    end
end
end