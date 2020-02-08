function [R,c] = ecuaciones_caminos(R,c,horizonte,n_out,n_in,Path,Out,t,rd_carga,rd_descarga,sys1)
%R: restricciones
%c: contador de restricciones
%horizonte: horizonte con el que se optimiza
%n_out: número de salidas
%n_in: número de entradas
%Path: caminos
%Out: Demandas
%t: instante de muestreo
%rd_carga: eficiencia de carga
% rd_descarga: eficiencia de descarga
% sys1: diagrama de energy hub
Path=ordenar_segun_Li(Path);
%Cambio alfa por alfa(t) en Path
for k=1:size(Path,2)
    j=size(Path(k).p,2);
    for i=2:j-1
        if(isempty(strfind(Path(k).p(i),'alfa'))==0||isempty(strfind(Path(k).p(i),'beta'))==0)
            Path(k).p(i)=strcat(Path(k).p(i),'(t)');
        end        
    end
end
%Cambio alfa por alfa(t) en R es diferente porque
%en uno es un vector de varias celdas de tipo string
%y en R es un vector de caracteres
for k=1:size(R,2)
    aux_alfa = strfind(R(k).p,'alfa');
    aux_beta = strfind(R(k).p,'beta');
    R(k).p=replace(R(k).p,'jpn1',strcat('jpn1','(t)'));
    R(k).p=replace(R(k).p,'jpn2',strcat('jpn2','(t)'));
    if(isempty(aux_alfa)==0||isempty(aux_beta)==0)
        if(isempty(aux_alfa)==0)% se podría meter en una funcion
            [R(k).p]=sustituir_df_x_dft('alfa',R(k).p);
        end
        aux_beta = strfind(R(k).p,'beta');
        if(isempty(aux_beta)==0)
            [R(k).p]=sustituir_df_x_dft('beta',R(k).p);
        end
        for i=1:horizonte-1
            R2 = replace(R(k).p,'(t)',strcat('(t+',string(i),')')); 
            for j=1:n_in
                R2 = replace(R2,strcat('PP',string(j)),strcat('PP',string(j),'(t+',string(i),')'));
            end
            R(c).p=R2;
            c=c+1;
        end
    end
end
for k=1:size(R,2)
    R2 =R(k).p;
    for j=1:n_in
        R2 = replace(R2,strcat('PP',string(j)),strcat('PP',string(j),'(t)'));
    end
    R(k).p=replace(R2,'(t)(t+','(t+');
end
%Creo la estructura para organizarlos por salidas, aux_indice1 es la salida
%y contador la cantidad de caminos que van a esa salida
contador=1;
aux_indice2=1;
for k=1:size(Path,2)
    if(k>1)
        aux2=char(Path(k-1).p(n));
        aux_indice2=aux2(4:size(aux2,2));
    end
    n=size(Path(k).p,2);
    aux1=char(Path(k).p(n));
    aux_indice1=aux1(4:size(aux1,2));
    if(strcmp(aux_indice1,aux_indice2)==0&&k>1)
        contador=1;
    end
    Pv(str2num(aux_indice1)).P(contador).pp=Path(k).p(1:n-1);
    contador=contador+1;
end
% Me devuelve todas las ecuaciones de caminos
for k=1:horizonte
    for j =1:n_out
        [salida_asociada,dispatch_factor] = leo_bloques_especiales(j,sys1);
        if(j==salida_asociada)
            aux_Out = strcat(string(Out(j,t+k-1)),'*',dispatch_factor,'(t)');
        else
            aux_Out = string(Out(j,t+k-1));
        end
        constraint_caminos(k).p=camino_instante_t(Pv,k,aux_Out,rd_carga,rd_descarga,n_in);
        aux=replace(constraint_caminos(k).p(j).p,'(t)',strcat('(t+',string(k-1),')'));
        R(c).p=[aux];
        c=c+1;
    end
end
end
%%
% Función local
function [R2]=sustituir_df_x_dft(aux,R)
%Sustituye t por (t)
aux_R = split(R,'*');
size_aux_R = size(aux_R,1);
for k=1:size_aux_R
    if(isempty(strfind(aux_R{k},aux))==0)
        cadena1=strcat(aux_R{k},'(t)');
        if(k>1)
            R2 = strcat(R2,'*',cadena1);
        else
            R2 = strcat(cadena1);
        end
    else
        if(k>1)
            R2 = strcat(R2,'*',aux_R{k});
        else
            R2 = strcat(aux_R{k});
        end
    end
end
end