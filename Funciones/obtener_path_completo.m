%Obtiene el camino completo
function minipath = obtener_path_completo(minipath,sys1)
%minipath: vector de conexiones 2 a 2
%sys1: diagrama del energy hub
bloque=get_param(sys1,'Blocks');
aux_size_bloque=size(bloque,1);
EH.def.I.N=0;%i latina
%Obtengo el número total de inputs y asigno números a los devices en la propiedad UserData
contador=1;
for k=1:aux_size_bloque
    aux_bloque=char(bloque(k));
    if(strcmp(get_param(strcat(sys1,'/',aux_bloque),'BlockType'),'Inport')==1)
        EH.def.I.N=EH.def.I.N+1;
    end
    if(strcmp(get_param(strcat(sys1,'/',aux_bloque),'BlockType'),'S-Function')==1)
        Ud=get_param(strcat(sys1,'/',aux_bloque),'UserData');
        Ud.Ndevice=contador;
        set_param(strcat(sys1,'/',aux_bloque),'UserData',Ud);
        set_param(strcat(sys1,'/',aux_bloque),'UserDataPersistent','on');
        contador=contador+1;
    end
    if(strcmp(get_param(strcat(sys1,'/',aux_bloque),'BlockType'),'SubSystem')==1)
        Ud=get_param(strcat(sys1,'/',aux_bloque),'UserData');
        if(isfield(Ud,'SFunction')==1)
            Ud.Ndevice=contador;
            set_param(strcat(sys1,'/',aux_bloque),'UserData',Ud);
            set_param(strcat(sys1,'/',aux_bloque),'UserDataPersistent','on');
            contador=contador+1;
        end
    end
end
%Cambio nomenclatura para adecuarla a la funcion "P_paths.m"
%y guardo los cambios en Nomenclatura
n=size(minipath,2);
contador=1;
for k=1:n
    Nomenclatura.b(k,1) = minipath(k).p(1);
    aux=char(minipath(k).p(1));
    size_aux1=size(aux,2);
    %Cuando hay dispatch factor en outputs
    aux1=minipath(k).p(1);
    aux2=minipath(k).p(2);
    aux_size_sys1=size_sys(sys1);
    if(size_aux1>aux_size_sys1)  
        Ud=get_param(aux(1:size_aux1-2),'UserData');
        ndevice=Ud.Ndevice;
        aux1=string(ndevice);
        Nomenclatura.b(k,2)=string(strcat('D',aux1,'O',aux(size_aux1)));
        EH.def.links.b(k)=Nomenclatura.b(k,2);
    else
        EH.def.links.b(k)=minipath(k).p(1);
        Nomenclatura.b(k,2)=minipath(k).p(1);
    end
        Nomenclatura.e(k,1) = minipath(k).p(2);
        aux=char(minipath(k).p(2));
        size_aux2=size(aux,2);
        if(strcmp(aux(1:3),'Out')==0)
            %Se le resta size_aux2-2 pq todos los end terminaran en df
            Ud=get_param(aux(1:size_aux2-2),'UserData');
            ndevice=Ud.Ndevice;
            aux_ndevice=string(ndevice);
            Nomenclatura.e(k,2)=string(strcat('D',aux_ndevice));%dispatch factor
            EH.def.links.e(k)=Nomenclatura.e(k,2);
        else%cuando son de tipo Out
            aux_out=char(minipath(k).p(2));
            size_out=size(aux_out,2);
            EH.def.links.e(k)=string(strcat('L',aux_out(4:size_out)));
            Nomenclatura.e(k,2)=string(strcat('L',aux_out(4:size_out)));
        end
end
EH.def.links.b=EH.def.links.b';
EH.def.links.e=EH.def.links.e';
[c,paths,inv_paths] = P_paths(EH);%no es de mi autoría, obtiene el camino completo

%Retomo mi nomenclatura
m=size(paths,1);
for k=1:m
    Path(k).p=split(paths(k),'-');
end
clear minipath %aseguro que se crea desde cero
for k=1:m
    n=size(Path(k).p);
    minipath(k).p=[];
    for kk=1:n
        minipath(k).p=[minipath(k).p Path(k).p(kk)];
    end
end
coincidencia=0;
for k=1:m
    n=size(minipath(k).p,2);
    kk=1;
    while(kk<=n)
        %Busco en la nomenclatura de begin si hay coincidencia
        aux=strfind(Nomenclatura.b(:,2),char(minipath(k).p(kk)));
        q = size(Nomenclatura.b,1);
        coincidencia=0;
        caso_especial=0;
        for w=1:q
            if(isempty(char(aux{w}))==0)
                minipath(k).p(kk)=string(Nomenclatura.b(w,1));
                auxx=char(Nomenclatura.b(w,1));
                if(strcmp(auxx(1),'P')==0)
                    coincidencia=0;
                    auxx2=char(Nomenclatura.b(w,2));
                    auxx3=split(auxx2,'O');
                    %Para obtener dispatch factor desde Nomenclatura.e
                    minipath(k)=desplazo_derecha(minipath(k),kk,auxx3(1));
                    n=n+1;
                else
                    coincidencia=1;
                end
                break
            end
        end
        if(coincidencia==0)%si no hay coincidencia busco en la nomenclatura de end
            aux=strfind(Nomenclatura.e(:,2),char(minipath(k).p(kk)));
            q = size(Nomenclatura.e,1);
            for w=1:q
                if(isempty(char(aux{w}))==0)
                    auxx=char(Nomenclatura.e(w,2));
                    if(strcmp(auxx(1),'L')==1)
                        %Compruebo Ud param y desplazo si no es empty
                        system=strcat(sys1,'/',char(Nomenclatura.e(w,1)));
                        Ud=get_param(system,'UserData');
                        try
                            if(isempty(Ud{:})==0)%si no esta vacío da error por los { }
                                %Si esta vacío no entra en catch, desplazo una a la derecha
                                %lo dejo por si entrara por alguna razón
                                minipath(k)=desplazo_derecha(minipath(k),size(minipath(k).p,2),char(Ud.disp_fact));
                                   %No aumento n porque es el ultimo bloque, el out
                                    %n=n+1;%no aumentar n porque ultimo bloque out
                            end
                        catch%da error, por lo que no esta vacío
                            if(isempty(Ud)==0)%si no esta vacío da error y desplazo una a la derecha
                                minipath(k)=desplazo_derecha(minipath(k),size(minipath(k).p,2),char(Ud.disp_fact));
                                caso_especial=1;
                                     %n=n+1;%no aumentar n porque ultimo bloque out
                                break
                            end
                        end
                    end
                    if(caso_especial==0)%no ha entrado porque caso especial==1
                        minipath(k).p(kk)=string(Nomenclatura.e(w,1));
                        break
                    end
                    caso_especial=0;
                end
            end
        end
        kk=kk+1;
    end
end
end
%%
%Función local
function [minipath2]=desplazo_derecha(minipath,kk,label)
%Amplío hacia la derecha una posición, en la posición de kk sitúo label
%que luego me dará el bloque para el dispatch factor del dispositivo
j=size(minipath.p,2);
c=1;
for k=1:j+1
    if(k<kk)
        minipath2.p(k)=minipath.p(k);
        c=c+1;
    else
        if(k==kk)
            minipath2.p(k)=string(label);
        end
        if(k>kk)
            minipath2.p(k)=minipath.p(c);
            c=c+1;
        end
    end
end
end