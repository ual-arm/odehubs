%Ordena el vector en función del número de la salida Li
function [Path] = ordenar_segun_Li(Path)
%Path: vector que indica el camino desde el inicio hasta el final
for k=1:(size(Path,2)-1)
   for j=(k+1):size(Path,2)
       aux1=char(Path(k).p(size(Path(k).p,2)));
       a1=size(aux1,2);
       aux2=char(Path(j).p(size(Path(j).p,2)));
       a2=size(aux2,2);
    if(str2num(aux1(4:a1))>str2num(aux2(4:a2)))
        aux=Path(k);
        Path(k)=Path(j);
        Path(j)=aux;
    end
   end
end