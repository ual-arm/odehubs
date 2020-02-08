%Obtiene las ecuaciones de balance de energía del energy hub
function [camino] = camino_instante_t(Pv,t,Out,rd_carga,rd_descarga,n_in)%% llevarmelo a un archivo .m
%Pv: path para cada salida
% t: instante de muestreo
% Out: demanda de una salida para el instante t
% rd_carga: eficiencia de carga
% rd_descarga: eficiencia de descarga
% n_in: número de entradas
contador=1;
for k=1:size(Pv,2)
    n=size(Pv(k).P,2);
    c1=1;
    for j=1:n
        m=size(Pv(k).P(j).pp,2);
        for i=1:m
            if(i==m)
                if(m==1&&j>1)
                    C(contador).p=strcat('+',char(Pv(k).P(j).pp(i)));
                else
                    if(m==1)
                        C(contador).p=strcat(char(Pv(k).P(j).pp(i)));
                    else
                        C(contador).p=strcat(C(contador).p,char(Pv(k).P(j).pp(i)));
                    end
                end
            else
                if(i>1)
                    C(contador).p=strcat(C(contador).p,char(Pv(k).P(j).pp(i)),"*");
                else
                    C(contador).p=strcat(char(Pv(k).P(j).pp(i)),"*");
                end
            end
        end
        if(c1<n)
            if(c1==1)
                C(contador).p1=strcat(C(contador).p,'+');
                c1=c1+1;
            else
                C(contador).p1=strcat(C(contador).p1,C(contador).p,'+');
                c1=c1+1;
            end
        else
            if(n>1)
                C(contador).p=strcat(C(contador).p1,C(contador).p);
            end
        end
    end
    aux=strcat(C(contador).p,'-',string(rd_carga(k)),'*Qch',string(k),'(t)*bin_carga',string(k),'(t)+',string(rd_descarga(k)),'*Qdis',string(k),'(t)*bin_descarga',string(k),'(t)==',Out);
    camino(contador).p=string(crear_PPi_t(aux,n_in));
    contador=contador+1;
end
end
