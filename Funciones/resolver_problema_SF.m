%Resuelvo el problema de crear mux y demux para SFunction, me salto los
%bloques de almacenamiento y las entradas sin transformación de energía
%tipo "nanofiltracion"
function [lineaxx] = resolver_problema_SF(sys,linea)
% sys: diagrama del energy hub
% línea: vector con información relativa a las conexiones
% líneaxx: vector modificado con información relativa a las conexiones

%Creo la relación entre demux y mux con bloques s-function
n=size(linea,2);
cont=1;
for k=1:n
    if(strcmp(linea(k).destino_tipo, 'Demux')==1)
        for j=1:n
            if(strcmp(linea(k).destino,linea(j).origen)==1)
                lineax(cont).origen=linea(k).origen;
                lineax(cont).origen_tipo=linea(k).origen_tipo;
                lineax(cont).destino=linea(j).destino;
                lineax(cont).destino_tipo=linea(j).destino_tipo;
                lineax(cont).puerto_origen=linea(j).puerto_origen;
                lineax(cont).puerto_destino=linea(j).puerto_destino;
                cont=cont+1;
            end
        end
    else
        if(strcmp(linea(k).origen_tipo, 'Mux')==1)
            for kk=1:n
                if(strcmp(linea(k).origen,linea(j).destino)==1)
                    lineax(cont).origen=linea(j).origen;
                    lineax(cont).origen_tipo=linea(j).origen_tipo;
                    lineax(cont).destino=linea(k).destino;
                    lineax(cont).destino_tipo=linea(k).destino_tipo;
                    lineax(cont).puerto_origen=linea(j).puerto_origen;
                    lineax(cont).puerto_destino=linea(j).puerto_destino;
                    cont=cont+1;
                end
            end
        else
            lineax(cont)=linea(k);
            cont=cont+1;
        end
    end
end

% Borro mux y demux que se repiten
y=size(lineax,2);
aux_cont=1;
for k=1:y
    aux=strcmp(lineax(k).origen_tipo,'Demux');
    aux1=strcmp(lineax(k).destino_tipo,'Demux');
    aux2=strcmp(lineax(k).origen_tipo,'Mux');
    aux3=strcmp(lineax(k).destino_tipo,'Mux');
    if(aux==0&&aux1==0&&aux2==0&&aux3==0)
        lineaxx(aux_cont)=lineax(k);
        aux_cont=aux_cont+1;
    end
end

%Busco todos los bloques de tipo device

blq_sf=find_system(sys,'BlockType','S-Function');
bloques_libreria_SS=find_system(sys,'BlockType','SubSystem');
contador=size(blq_sf,1);
contador=contador+1;
m = size(bloques_libreria_SS,1);
for k=1:m
    Ud=get_param(char(bloques_libreria_SS(k)),'UserData');
    if(isfield(Ud,'SFunction')==1)
        blq_sf(contador)=bloques_libreria_SS(k);
        contador=contador+1;
    end
end
%%%%%%%%%%%%%%%%%%%%%
%Recorrre la estructura buscando los bloques especiales, son aquellos que tienen varias entradas y solo una de ellas con transformacion de energia
lineaxxx=[];
% cont=1;
% y=size(lineaxx,2);
% aux_size_sf=size(blq_sf);
% for j=1:aux_size_sf
%     Ud=get_param(char(blq_sf(j)),'UserData');
%     if(isfield(Ud,'bloque_especial')==1)
%         for k=1:y     
%              if(strcmp(lineaxx(k).origen,blq_sf(j))==0||(isempty(find(Ud.puertos_sin_transformacion==lineaxx(k).puerto_origen))==1))
%                 if(strcmp(lineaxx(k).destino,blq_sf(j))==1)
%                     if(isempty(find(Ud.puertos_sin_transformacion==lineaxx(k).puerto_destino))==0)
%                         for i=1:y
%                             if(strcmp(lineaxx(k).destino,lineaxx(i).origen)==1&&lineaxx(k).puerto_destino==lineaxx(i).puerto_origen)
%                                 lineaxxx(cont).origen=lineaxx(k).origen;
%                                 lineaxxx(cont).origen_tipo=lineaxx(k).origen_tipo;
%                                 lineaxxx(cont).destino=lineaxx(i).destino;
%                                 lineaxxx(cont).destino_tipo=lineaxx(i).destino_tipo;
%                                 lineaxxx(cont).puerto_origen=lineaxx(k).puerto_origen;
%                                 lineaxxx(cont).puerto_destino=lineaxx(i).puerto_destino;
%                                 cont=cont+1;
%                             end
%                         end
%                     else
%                         lineaxxx(cont)=lineaxx(k);
%                         cont=cont+1;
%                     end
%                 else
%                     lineaxxx(cont)=lineaxx(k);
%                     cont=cont+1;
%                 end
%             end
%         end
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%5
if(isempty(lineaxxx)==1)
    lineaxxx=lineaxx;
end

%Me salto los de almacenamiento
clear lineax
n=size(lineaxxx,2);
cont=1;
for k=1:n
    Ud=get_param(lineaxxx(k).destino,'UserData');
    if(strcmp(lineaxxx(k).destino_tipo, 'SubSystem')==1&&isfield(Ud,'Store_system')==1)
        for j=1:n
            if(strcmp(lineaxxx(k).destino,lineaxxx(j).origen)==1)
                lineax(cont).origen=lineaxxx(k).origen;
                lineax(cont).origen_tipo=lineaxxx(k).origen_tipo;
                lineax(cont).destino=lineaxxx(j).destino;
                lineax(cont).destino_tipo=lineaxxx(j).destino_tipo;
                lineax(cont).puerto_origen=lineaxxx(k).puerto_origen;
                lineax(cont).puerto_destino=lineaxxx(j).puerto_destino;
                cont=cont+1;
            end
        end
    else
        if(strcmp(lineaxxx(k).origen_tipo, 'SubSystem')==1&&isfield(Ud,'Store_system')==1)
            for j=1:n
                if(strcmp(lineaxxx(k).origen,lineaxxx(j).destino)==1)
                    lineax(cont).origen=lineaxxx(j).origen;
                    lineax(cont).origen_tipo=lineaxxx(j).origen_tipo;
                    lineax(cont).destino=lineaxxx(k).destino;
                    lineax(cont).destino_tipo=lineaxxx(k).destino_tipo;
                    lineax(cont).puerto_origen=lineaxxx(j).puerto_origen;
                    lineax(cont).puerto_destino=lineaxxx(k).puerto_destino;
                    cont=cont+1;
                end
            end
        else
            lineax(cont)=lineaxxx(k);
            cont=cont+1;
        end
    end
end

%Borro el almacenamiento
y=size(lineax,2);
clear lineaxx
contador=1;
for k=1:y
    src=get_param(lineax(k).origen,'UserData');
    dst=get_param(lineax(k).destino,'UserData');
    if(isfield(src,'Store_system')==0&&isfield(dst,'Store_system')==0)
        lineaxx(contador)=lineax(k);
        contador=contador+1;
    end
end
end