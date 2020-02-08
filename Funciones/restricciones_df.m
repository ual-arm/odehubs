%Crea las restricciones de los coeficientes de reparto
function [R,c,vector_df] = restricciones_df(R,c,sys1)
%R: restricciones
%c: contador de restricciones
%sys1: diagrama del energy hub
%vector_df: vector con los nombres de los coeficientes de reparto creados
h = find_system(sys1,'FindAll','On','type', 'line');
h = eliminar_subraices_directorios(h,sys1);
m=size(h,1);
aux_contador=1;
%Obtengo todos los bloques origen de las líneas que son de tipo cell
for j=1:m 
    a=getfullname(get_param(h(j),'DstBlockHandle'));
    if(iscell(a)==1)
       v_origen(aux_contador).p=getfullname(get_param(h(j),'SrcBlockHandle'));
       aux_handle =get_param(v_origen(aux_contador).p,'LineHandles');
       aux_out=size(aux_handle.Outport,2);
       for k=1:aux_out
           v_origen(aux_contador).p=getfullname(aux_handle.Outport(k));
           aux_contador=aux_contador+1;
       end
    end
end
%Elimino todos los bloques origen que se repiten, no tengo en cuenta el puerto de origen
m=size(v_origen,2);
k=1;
b=1;
while(k<=m)
    v_destino(k).p=[];
    kk=k+1;
    while(kk<=m)
        if(strcmp(v_origen(k).p,v_origen(kk).p)==1)
            v_origen(kk)=[];
            m=m-1;
            kk=k;
        end
        kk=kk+1;
    end
    k=k+1;
end

%Obtengo los bloques destino de cada origen
[m,n]=size(h);
aux_size=size(v_origen,2);
for k=1:m
    aux_dst_block=getfullname(get_param(h(k),'DstBlockHandle'));
    if(iscell(aux_dst_block)==0) 
        aux_src_nombre=getfullname(get_param(h(k),'SrcBlockHandle'));
        aux_line =get_param(aux_dst_block,'LineHandles');
        aux_in=size(aux_line.Inport,2);
        for j=1:aux_in
            aux_line_nombre=getfullname(aux_line.Inport(j));
            for q=1:aux_size
                aux_size_dst=size(v_destino(q).p,2);
                if(strcmp(aux_line_nombre,v_origen(q).p)==1&&isempty(strfind(v_origen(q).p,aux_src_nombre))==0)
                   aux_size_dst=aux_size_dst+1;
                    v_destino(q).p(aux_size_dst).p=getfullname(get_param(h(k),'DstBlockHandle'));
                end
            end
        end
    end
end
lineas=relacion_entre_puertos(sys1);%obtengo la relación entre bloques
lineas=resolver_problema_SF(sys1,lineas);
aux_cont=1;
for k=1:aux_size
    clear aux
    aux_linea = v_destino(k).p;
    %Creo restricción
    q=size(aux_linea,2);
    aux_restriccion=[];
    for j=1:q
        if(strcmp(get_param(aux_linea(j).p,'BlockType'),'Sum')==1)
            aux.destino=char(aux_linea(j).p);
            [out,out2,out3,w] = bloque_final_de_sum(aux,lineas);
            aux_linea(j).p=out;
        end   
        if(strcmp(get_param(aux_linea(j).p,'BlockType'),'Outport')==0)
            df=char(get_param(aux_linea(j).p,'dispatch_factor'));
            vector_df(aux_cont).p=df;
            aux_cont=aux_cont+1;
            R(c).p=strcat('[0<=',df,'<=1]');
            c=c+1;
            if(j==q)%restricción de cantidad alfa1+alfa2=1
                R(c).p=strcat('[',aux_restriccion,df,'==1]');
                c=c+1;
            else
                aux_restriccion=strcat(df,'+',aux_restriccion);
            end
        end
    end
end
end