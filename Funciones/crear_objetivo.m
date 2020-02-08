%Creo la función a optimizar
function [Objective] = crear_objetivo(horizonte,n_in)
% horizonte: muestras a optimizar
% n_in: número de recursos de entrada
% Objective: función objetivo
c=1;
for k=1:horizonte
    for kk=1:n_in
        if(c==1)
            Objective=strcat('F(',string(kk),',t+',string(k-1),')*PP',string(kk),'(t+',string(k-1),')');
            c=c+1;
        else
            Objective=strcat(Objective,'+F(',string(kk),',t+',string(k-1),')*PP',string(kk),'(t+',string(k-1),')');
        end
    end
end
end