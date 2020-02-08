function [c,paths,inv_paths] = P_paths(EH)
%Handle to local function: fh{1}=P_count_paths
fh=localfunctions;

%alphanumeric definition of paths from I to O
c=1;
paths="";
for i=1:1:EH.def.I.N    %para cada input
    [c,paths] = fh{1}(strcat("P",num2str(i),"-"),EH.def.links.b,EH.def.links.e,c,paths);
end
c=c-1;
paths=paths';
inv_paths=paths;
for i=1:1:length(paths)
    Ndev=strfind(paths(i),"-"); %determine the number of devices
    devices=strings(1,length(Ndev)+1); %empty array to store devices
    Ndev=[0 Ndev length(char(paths(i)))+1]; %include I-O
    for k=1:length(Ndev)-1          %store devices
       aux=char(paths(i));
       devices(k)=string(aux(Ndev(k)+1:Ndev(k+1)-1)); %devices, including I-O
    end  
    aux="";
    for k=1:length(devices)          %store devices
       aux=strcat(devices(k),"-",aux);       
    end
    aux=char(aux);
    inv_paths(i)=string(aux(1:length(aux)-1));
end
end

function [c,paths] = P_count_paths(p,b,e,c,paths)
%recursive function to find paths from I to O based on defined links

%get the last item from path p
pitems=char(p);
if contains(pitems,'-')
    aux=strfind(pitems,'-');%busca todos los guiones
    if length(aux)>1%si hay mas de un guion
    lastitem=pitems(aux(end-1)+1:aux(end)-1);
    else
    lastitem=pitems(1:aux(end)-1);%si hay un guion
    end
end
lastitem=string(lastitem);

for i=1:1:length(b)  %para cada relacion
    baux=char(b{i});
    eaux=char(e{i});
    
    %check devices with different outputs and just get the device number
    if contains(baux,'O')%si contiene el begin O (multiples salidas)
        bOaux=baux(strfind(baux,'O'):end);
        baux=baux(1:strfind(baux,'O')-1);
    else
        bOaux="";
    end        
    baux=(string(baux));
    
    if(lastitem==baux) %find item in links 
        if(eaux(1)=='L')          %si el final es de Ox  
            if length(aux)>1%si hay mas de un guion
                paths(c)=strcat(pitems(1:aux(end-1)),baux,bOaux,"-",eaux); %genero path
            else%si no hay mas de un guion
                paths(c)=strcat(pitems,eaux); %genero path
            end     
            c=c+1;%aumento el contador del path
        else%si el final no es d etipo Ox
            if length(aux)>1%si hay mas de un guion
                p=strcat(pitems(1:aux(end-1)),baux,bOaux,"-",eaux,"-"); %genero path pero en p
            else%si no hay mas de un guion
                p=strcat(pitems,eaux,"-"); %genero path pero en p
            end            
            [c,paths] = P_count_paths(p,b,e,c,paths);%vuelvo a llamar a la misma funcion
        end 
    end
end

end