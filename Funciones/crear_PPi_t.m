function aux = crear_PPi_t(aux,n_in)
%aux: ecuación
%n_in: número de entradas
    for q=1:n_in
        aux=replace(aux,strcat('P',string(q)),strcat('PP',string(q),'(t)'));
    end
    for k=1:100%limitado a 100 salidas binarias
        aux1=aux;
        aux=replace(aux,strcat('jpn',string(k)),strcat('jpn',string(k),'(t)'));
    end
end