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

function [b,e] = rename_links(EH)
%Rename links and labels to numeric notation

%Preset
b=EH.def.linkslabel.b;
e=EH.def.linkslabel.e;

%Handle to local function: fh{1}=matchelement
fh=localfunctions;
for j=1:length(EH.def.linkslabel.e)
    %Inputs
    b(j)=fh{1}(EH.def.linkslabel.b(j),EH);
    e(j)=fh{1}(EH.def.linkslabel.e(j),EH);
end
clearvars i j
end

function [element] = matchelement(element_label,EH)
%Match each element by its label with its name I/O/D y nº
auxI=[strcat(EH.def.I.t,EH.def.I.l) strcat("I",string(EH.def.I.n))'];
auxO=[strcat(EH.def.O.t,EH.def.O.l) strcat("O",string(EH.def.O.n))'];
auxD=[":)" ":("];
for i=1:EH.def.dev.N
    auxD=[auxD; EH.def.dev.D(i).l strcat("D",string(i))];
end
auxD=auxD(2:EH.def.dev.N+1,:);
aux=[auxI;auxO;auxD]; %first column: label, second: name
if strfind(element_label,"_O")>0
    aux5=char(element_label);
    element_label=aux5(1:strfind(element_label,"_O")-1);
    aux2=aux5(strfind(aux5,'_O')+1:length(aux5));
else
    aux2="";
end
aux3=strcmp(string(element_label),aux(:,1));
element=strcat(aux(find(aux3,1),2),aux2);
end



