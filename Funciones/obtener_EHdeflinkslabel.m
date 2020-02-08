%% función "obtener_EHdeflinkslabel.m"
%obtengo las relaciones entre bloques 2 a 2
function linkslabel = obtener_EHdeflinkslabel(linea,sys)
% línea: vector con información de las líneas
% sys: diagrama de energy hub
% linkslabel: estructura de relación de conexiones 2 a 2
n=size(linea,2);
k=1;
for j=1:n
    if(strcmp(linea(j).origen_tipo,'Inport')==1)
        %Cuando origen es inport
        if(strcmp(linea(j).destino_tipo,'Outport')==1)
            %Si destino es outport
            linkslabel.end(k)=obtener_identificador(linea(j),sys,'e');
            linkslabel.begin(k).p=string(get_param(linea(j).origen,'Name'));
            linkslabel.linea(k).p=linea(j);
            k=k+1;
        else
            if(strcmp(linea(j).destino_tipo,'Sum')==0)
                %Si destino no es suma
                linkslabel.end(k)=obtener_identificador(linea(j),sys,'e');
                linkslabel.begin(k).p=string(get_param(linea(j).origen,'Name'));
                linkslabel.linea(k).p=linea(j);
                k=k+1;
            else            
                [aux,aux2,aux3,cont] = bloque_final_de_sum(linea(j),linea);
                for q=1:cont
                    %Para cada bloque destino
                    if(strcmp(aux2(q),'Outport')==1)
                        linea_aux.destino=aux(q);
                        linea_aux.destino_tipo=aux2(q);
                        linea_aux.puerto_destino=aux3(q);
                        linkslabel.end(k)=obtener_identificador(linea_aux,sys,'e');
                        linkslabel.begin(k)=obtener_identificador(linea(j),sys,'b');
                        linkslabel.linea(k).p=linea(j);
                        linkslabel.linea(k).p.destino=aux(q);
                        linkslabel.linea(k).p.destino_tipo=aux2(q);
                        linkslabel.linea(k).p.puerto_destino=aux3(q);
                        k=k+1;
                    else
                        aux4.destino=aux(q);
                        aux4.destino_tipo=aux2(q);
                        aux4.puerto_destino=aux3(q);
                        linkslabel.end(k)=obtener_identificador(aux4,sys,'e');
                        linkslabel.begin(k)=obtener_identificador(linea(j),sys,'b');
                        linkslabel.linea(k).p=linea(j);
                        linkslabel.linea(k).p.destino=aux(q);
                        linkslabel.linea(k).p.destino_tipo=aux2(q);
                        linkslabel.linea(k).p.puerto_destino=aux3(q);
                        k=k+1;
                    end    
                end
            end
        end
    else
        %Cuando no es ni de from workspace ni sum
        if(strcmp(linea(j).origen_tipo,'Sum')==0)
            if(strcmp(linea(j).destino_tipo,'Sum')==0)
                if(strcmp(linea(j).destino_tipo,'Outport')==1)
                    linkslabel.end(k).p=string(get_param(linea(j).destino,'Name'));
                    linkslabel.begin(k)=obtener_identificador(linea(j),sys,'b');
                    linkslabel.linea(k).p=linea(j);
                    k=k+1;
                else
                    %Ya solo puede ser de tipo subsystem y S-Function
                    linkslabel.end(k)=obtener_identificador(linea(j),sys,'e');
                    linkslabel.begin(k)=obtener_identificador(linea(j),sys,'b');
                    linkslabel.linea(k).p=linea(j);
                    k=k+1;
                end
            else
                [aux,aux2,aux3,cont] = bloque_final_de_sum(linea(j),linea);
                for q=1:cont
                    if(strcmp(aux2(q),'Outport')==1)
                        linkslabel.end(k).p=string(get_param(aux(q),'Name'));
                        linkslabel.begin(k)=obtener_identificador(linea(j),sys,'b');
                        linkslabel.linea(k).p=linea(j);
                        linkslabel.linea(k).p.destino=aux(q);
                        linkslabel.linea(k).p.destino_tipo=aux2(q);
                        linkslabel.linea(k).p.puerto_destino=aux3(q);
                        k=k+1;
                    else
                        aux4.destino=aux(q);
                        aux4.destino_tipo=aux2(q);
                        aux4.puerto_destino=aux3(q);
                        linkslabel.end(k)=obtener_identificador(aux4,sys,'e');
                        linkslabel.begin(k)=obtener_identificador(linea(j),sys,'b');
                        linkslabel.linea(k).p=linea(j);
                        linkslabel.linea(k).p.destino=aux(q);
                        linkslabel.linea(k).p.destino_tipo=aux2(q);
                        linkslabel.linea(k).p.puerto_destino=aux3(q);
                        k=k+1;
                    end    
                end
            end
            %Tipo suma; no lo tengo en cuenta el origen suma porque lo
            %considero todo desde aguas arriba hacia aguas abajo (izqda-derecha)
        end 
    end   
end
%%
% Función local
function [label] = obtener_identificador(linea,sys,aux)
if(strcmp(aux,'e')==1)%si es end
    if(strcmp(linea.destino_tipo,'Outport')==1)
        Ud=get_param(linea.destino,'UserData');
        if(isempty(Ud)==0)
            label.p=string(get_param(linea.destino,'Name'));
        else
            label.p=string(get_param(aux,'Name'));
        end
    else        
        label.p=string(strcat(linea.destino,'df'));
    end
else %si es begin solo sera SFunction o SubSystem
    Ud=get_param(linea.origen,'UserData');
    if(strcmp(linea.origen_tipo,'Inport')==0&&(isfield(Ud,'SFunction')==1||strcmp(linea.origen_tipo,'S-Function')==1))
            label.p=string(strcat(linea.origen,'/',string(linea.puerto_origen))); 
    end
    if(strcmp(linea.origen_tipo,'Inport')==1)%cuando inport va a bloque sum y luego a outport inport entra aqui
        label.p=string(get_param(linea.origen,'Name'));
    end
end