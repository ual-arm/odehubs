%Devuelve el bloque de inicio y fin de línea, puerto y tipo de bloque
function [Path,Path_Restricciones] = obtener_paths(sys1)
% sys1: diagrama del energy Hub
% Path: camino desde el recurso de entrada hasta el de salida
% Path_Restricciones: camino para crear restricciones
linea=relacion_entre_puertos(sys1);
linea=resolver_problema_SF(sys1,linea);
%Línea lleva los bloques que interesan
EH = obtener_EHdeflinkslabel(linea,sys1);%creo EH

%Junto begin con end
for k=1:size(EH.end,2)
    a1=EH.begin(k).p;
    a2=EH.end(k).p;
    minipath(k).p=[a1 a2];
end

Path = obtener_path_completo(minipath,sys1);
Path_Restricciones=Path;

%Restricciones_flujos
%Sustituye los nombres con df y los nombres de los puertos
%con los coeficientes deseados
for k=1:size(Path,2)
    for kk=1:size(Path(k).p,2)
        aux=char(Path(k).p(kk));
        x=size(aux,2);
        if(x>1)
            if(strcmp(aux(x-1:x),'df')==1)
                Path(k).p(kk)=string(get_param(aux(1:x-2),'dispatch_factor'));
            end
        end
        aux2=strfind(aux,'/');
        if(size(aux2,2)==3)
            posicion = aux2(3)+1;
            Ud=get_param(aux(1:(posicion-2)),'UserData');
            if(isfield(Ud,'Bloque_binario')==0)
                try
                    aux1=get_param(aux(1:(posicion-2)),'efficiency');
                    eval(strcat('aux2=',char(aux1),';'));
                    num=str2double(aux(posicion));
                    Path(k).p(kk)=string(aux2(num));
                catch
                    aux_salidas=get_param(aux(1:(posicion-2)),'LineHandles');
                    aux_salidas=size(aux_salidas.Outport,2);
                    aux1=get_param(aux(1:(posicion-2)),'efficiency');
                    aux1=aux1(2:size(aux1,2)-1);
                    aux1=split(aux1,' ');
                    num=str2double(aux(posicion));
                    Path(k).p(kk)=strcat(char(aux1(num)),'(t)');
                end      
            else
                try 
                    aux1=get_param(aux(1:(posicion-2)),'efficiency');
                    eval(strcat('aux3=',char(aux1),';'));
                    jpn=get_param(aux(1:(posicion-2)),'Salida_Binaria');
                    eval(strcat('aux_jpn=',jpn,';'));
                    rango1=aux2(3)+1;
                    jpn=strcat('jpn',string(aux_jpn(str2num(aux(rango1:size(aux,2))))));
                    num=str2double(aux(posicion));
                    Path(k).p(kk)=string(strcat(string(aux3(num)),'*',jpn));
                catch
                    aux_salidas=get_param(aux(1:(posicion-2)),'LineHandles');
                    aux_salidas=size(aux_salidas.Outport,2);
                    aux1=get_param(aux(1:(posicion-2)),'efficiency');
                    aux1=aux1(2:size(aux1,2)-1);
                    aux1=split(aux1,' ');
                    jpn=get_param(aux(1:(posicion-2)),'Salida_Binaria');
                    eval(strcat('aux_jpn=',jpn,';'));
                    rango1=aux2(3)+1;
                    jpn=strcat('jpn',string(aux_jpn(str2num(aux(rango1:size(aux,2))))));
                    num=str2double(aux(posicion));
                    Path(k).p(kk)=strcat(char(aux1(num)),'(t)*',jpn);
                end
            end
        end
    end
end
end