%Obtiene las restricciones de flujo de entrada a los dispositivos
function [R,c] = restricciones_flujo(c,P_R,Path)
%c: contador de restricciones
%P_R: caminos para restriccionoes
%Path: caminos
%R: restricciones

for v=1:size(P_R,2)
    m=size(P_R(v).p,2);
    if(m>2)
        for k=2:m-1%me quito los inputs y los outputs
            aux=char(P_R(v).p(k));
            o=size(aux,2);
            if(strcmp(aux(o-1:o),'df')==1)%restricción de entrada
                bloque=aux(1:o-2);%me quito el df del nombre
                parametros = get_param(bloque,'ObjectParameters'); 
                if(isfield(parametros,'Max_in')==1)
                    if(strcmp(parametros.Max_in,'inf')==0)
                        R(c).p = restriccion_producto_flujo(Path(v),bloque,k,'Max_in','>=');
                        c=c+1;
                    end
                end
                if(isfield(parametros,'Min_in')==1)
                    if(strcmp(parametros.Min_in,'-inf')==0)
                        R(c).p = restriccion_producto_flujo(Path(v),bloque,k,'Min_in','<=');
                        c=c+1;
                    end
                end
            else%restricción de máximo    
                Ud=get_param(aux(1:o-2),'UserData');
                if(isfield(Ud,'Bloque_binario')==1)
                    maximo=get_param(aux(1:o-2),'Max_Out');%me quito el '/puerto' del bloque
                    eval(strcat('maximo=',maximo,';'));
                    producto = producto_flujo(Path(v),k);
                    producto=replace(producto,'P','PP');
                    auxiliar_jpn=strfind(producto,'jpn');
                    R(c).p=strcat(string(maximo(str2double(producto(auxiliar_jpn+3)))),'>=',producto);
                    c=c+1;
                else
                    maximo=get_param(aux(1:o-2),'Max_Out');%me quito el '/puerto' del bloque
                    eval(strcat('maximo=',maximo,';'));
                    aux_size=size(maximo,2);
                    for j=1:aux_size
                        producto = producto_flujo(Path(v),k);
                        producto=replace(producto,'P','PP');
                        R(c).p=strcat(string(maximo(j)),'>=',producto);
                        c=c+1;
                    end
                end
            end
        end
    end
end
end