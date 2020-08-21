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

function [c,dev,dev2] = D_count(EH)
%get devices and their outputs
%include all the devices naming DX, X= number of device

aux2="";
aux3=zeros(1,3);
for i=1:1:EH.def.dev.N
    aux2=[aux2;strcat("D",string(i))];
end
dev=aux2(2:length(aux2));

%add the outputs of each device as new virtual devices
aux2="";
for i=1:1:EH.def.dev.N    
    if length(EH.def.dev.D(i).O.n)>1
        for j=1:1:length(EH.def.dev.D(i).O.n)
            aux2=[aux2;strcat("D",string(i),"O",string(j))];
            aux3(length(aux2),:)=[i,j,...
             EH.def.dev.D(i).O.s(j)];
        end
    else
        
        aux2=[aux2;strcat("D",string(i))];
        aux3(length(aux2),:)=[i,1,1];
    end
end
dev=aux2(2:length(aux2));
dev2=aux3(2:length(aux2),:);
c=length(dev);
end


