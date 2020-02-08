
%Obtengo la cantidad de recursos de entrada/salida y nombro los bloques de tipo inport 
function [n_in,n_out] = manejo_puertos_EH(sys1)
%sys1: diagrama del energy hub
%n_in: número de recursos de entrada
%n_out: número de recursos de salida

%Cambio el nombre de los bloques de tipo inport a P1, P2, P3, etc.
h=find_system(sys1,'BlockType','Inport');
h = eliminar_subraices_directorios(h,sys1);
a=strfind(strcat(sys1,'/'),'/');%busco coincidencias con '/'
y=a(size(a,2))+1;%obtengo la posición del caracter que me interesa
n=size(h,1);
%Compruebo que estan nombrados correctamente, en su defecto los nombro
for k=1:n
    aux_bloque=char(h(k));
    if(strcmp(aux_bloque(y:y+1),'In')==1)
        set_param(aux_bloque,'Name',strcat('P',aux_bloque(y+2)));
    end
end
%Obtengo el número de puertos de entrada y de salida del energy hub
aux_puertos = get_param(sys1,'PortHandles');
n_in=size(aux_puertos.Inport,2);
n_out=size(aux_puertos.Outport,2);
end