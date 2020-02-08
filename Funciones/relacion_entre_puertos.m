%Obtiene la relacion entre los bloques dos a dos con el número del puerto
function linea = relacion_entre_puertos(sys)
% sys: diagrama del energy hub
% linea: estructura con la información de las relaciones
h = find_system(sys,'FindAll','On','type', 'line');
h = eliminar_subraices_directorios(h,sys);
m=size(h,1);%Obtengo el número de líneas
v=0;
try
    for j=1:m
        dst=getfullname(get_param(h(j),'DstBlockHandle'));
        if(iscell(dst)==0)%compruebo que no se trata de un cell array
            v=v+1;
            linea(v).origen=getfullname(get_param(h(j),'SrcBlockHandle'));
            linea(v).origen_tipo=get_param(linea(v).origen,'BlockType');
            aux_line_origen=get_param(linea(v).origen,'LineHandles');
            linea(v).destino=getfullname(get_param(h(j),'DstBlockHandle'));
            linea(v).destino_tipo=get_param(linea(v).destino,'BlockType');
            aux_line_dst=get_param(linea(v).destino,'LineHandles');
            aux_size_dst=size(aux_line_dst.Inport,2);
            aux_size_origen=size(aux_line_origen.Outport,2);
            for k=1:aux_size_origen
                aux_full_name_origen=getfullname(aux_line_origen.Outport(k));
                for t=1:aux_size_dst
                    aux_full_name_dst=getfullname(aux_line_dst.Inport(t));
                    %Comparo nombres completos, incluidos puertos
                    if(strcmp(aux_full_name_origen,aux_full_name_dst)==1)
                        linea(v).puerto_origen=k;
                        linea(v).puerto_destino=t;
                    end
                end
            end
        end
    end
catch
    error('Compruebe que todos los puertos estén conectados')
end
end