% ------------------------------------------------------------------------
% This file is part of the ODEHubs Toolbox v1.0
%
% ODEHubs - Optimal Dispatch of Energy Hubs
% Copyright (C) 2020  Automatic Control, Robotics and Mechatronics (ARM) 
% research group (TEP-197), University of Almeria
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <https://www.gnu.org/licenses/>.
%
% Find more information at: https://github.com/ual-arm/odehubs
% ------------------------------------------------------------------------

function [c,paths,inv_paths] = P_paths(EH)
%Find the paths from Inputs to Outputs by analising the provided links.

%Handle to local function: fh{1}=P_count_paths
fh=localfunctions;

%alphanumeric definition of paths from I to O
c=1;
paths="";
for i=1:1:EH.def.I.N    
    [c,paths] = fh{1}(strcat("I",num2str(i),"-"),...
        EH.def.links.b,EH.def.links.e,c,paths);
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
    aux=strfind(pitems,'-');
    if length(aux)>1
    lastitem=pitems(aux(end-1)+1:aux(end)-1);
    else
    lastitem=pitems(1:aux(end)-1);
    end
end
lastitem=string(lastitem);

for i=1:1:length(b)  
    baux=char(b{i});
    eaux=char(e{i});
    
    %check devices with different outputs and just get the device number
    if contains(baux,'O')
        bOaux=baux(strfind(baux,'O'):end);
        baux=baux(1:strfind(baux,'O')-1);
    else
        bOaux="";
    end        
    baux=(string(baux));
    
    if(lastitem==baux) %find item in links 
        if(eaux(1)=='O')            
            if length(aux)>1
                paths(c)=strcat(pitems(1:aux(end-1)),baux,bOaux,"-",eaux); 
            else
                paths(c)=strcat(pitems,eaux); 
            end     
            c=c+1;
        else
            if length(aux)>1
                p=strcat(pitems(1:aux(end-1)),baux,bOaux,"-",eaux,"-"); 
            else
                p=strcat(pitems,eaux,"-"); 
            end            
            [c,paths] = P_count_paths(p,b,e,c,paths);
        end 
    end
end

end