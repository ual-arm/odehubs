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

function [S0] = S0_func(EH)
%Returns the initial state (storage) of the system
%

S0=NaN(EH.def.O.N,1); %Intital storage state

for i=1:length(EH.def.dev.S)
    if isvarname(EH.def.dev.S(i).l)
        if  ischar(EH.def.dev.S(i).S0{:}) %initial state
            right_term=[num2str(EH.def.dev.S(i).S0{:}),...
                    '(EH);']; %function to get S0
        elseif  isnumeric(EH.def.dev.S(i).S0{:})
            right_term=[num2str(EH.def.dev.S(i).S0{:}),';']; %S0 value
        else
            error('Invalid initial state for storage device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_S0=',right_term));  
    else
        error('Invalid name or label for device %d',i)
    end
end

for i=1:EH.def.O.N
if EH.def.O.st(i)>0 %Storage system
S0(i)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_S0")));
else
S0(i)=0;
end
end

end

