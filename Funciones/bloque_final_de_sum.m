%Obtengo el bloque final al que se dirige el camino
function [out,out2,out3,w] = bloque_final_de_sum(linea,lineass)
%línea: linea de origen de la que se parte
%líneass: vector de líneas
j=size(lineass,2);
p=linea;
w=1;
k=1;
contador=1;
while k<=j
    if(strcmp(p.destino,lineass(k).origen)==1)
        %Si es de tipo sum lo almaceno en un vector
        if(strcmp(lineass(k).destino_tipo,'Sum')==1)            
            vect_sum(contador)=lineass(k);
            vect_sum_k(contador)=k;
            contador =contador+1;
        else
            %Si no es de tipo sum, lo almaceno en las salidas
            out(w) =cellstr(lineass(k).destino);%contador por si el final es de varios bloques
            out2(w)=cellstr(lineass(k).destino_tipo);
            out3(w)=lineass(k).puerto_destino;
            w=w+1;
        end
    end
    k=k+1;
end
%Recorro el vector de coincidencias bloque sum
for i =1:contador-1
    %Función recursiva
    [outt,outt2,outt3,aux_w] = bloque_final(vect_sum(i),lineass);
    for q=1:aux_w
        out(w) =outt(q);%contador por si el final es de varios bloques
        out2(w)=outt2(q);
        out3(w)=outt3(q);
        w=w+1;
    end
end
w=w-1;
end
%%
%Función local
function [out,out2,out3,w] = bloque_final(linea,lineass)
%Función recursiva, variable recursiva: p
j=size(lineass,2);
p=linea;
w=1;
k=1;
while k<=j
    if(strcmp(p.destino,lineass(k).origen)==1)
        if(strcmp(lineass(k).destino_tipo,'Sum')==1)
            %Reasigno p y reinicio búsqueda
            p=lineass(k);
            k=1;
        else
            out(w) =cellstr(lineass(k).destino);%contador por si el final es de varios bloques
            out2(w)=cellstr(lineass(k).destino_tipo);
            out3(w)=lineass(k).puerto_destino;
            w=w+1;
        end
    end
    k=k+1;
end
w=w-1;
end