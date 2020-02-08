function [producto]=producto_flujo(Path,k)
%path: camino
%k: posici�n celda dentro del camino
%producto: producto del camino hasta posici�n k
producto=[];
for g=1:k
    if(g==1)
        producto=strcat(producto,char(Path.p(g)),'*');
    else
        if(g==k)
            producto=strcat(producto,char(Path.p(g)));
        else
            producto=strcat(producto,char(Path.p(g)),'*');
        end
    end
end
end