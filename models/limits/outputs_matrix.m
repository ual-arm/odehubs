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

function [O] = outputs_matrix(EH,data,date,samples,tm)
%Returns the matrix O for each output
%

%Resample data according to tm
Time=downsample(data.Time,tm);
data=retime(data,Time,'mean');

%Select the suitable time range
TR=timerange(date,date+minutes(samples*tm));
data=data(TR,:);

O=NaN(EH.def.O.N,samples);

%Create one variable from either coeficients or functions containing the
%output vector for the 'samples' horizon
for i=1:EH.def.O.N
    if isvarname(strcat(EH.def.O.t(i),EH.def.O.l(i)))
        if  ischar(EH.def.O.O{i}) 
            if ismember(EH.def.O.O{i},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.O.O{i}),');'];
            else
                right_term=[num2str(EH.def.O.O{i}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.O.O{i})
            right_term=[num2str(EH.def.O.O{i}),...
                '*ones(1,samples);'];
        else
            error('Invalid demand for output %d: %s',i,...
                strcat(EH.def.O.t(i),EH.def.O.l(i)));
        end
        eval(strcat(strcat(EH.def.O.t(i),EH.def.O.l(i)),'=',right_term))       
    else
        error('Invalid name or label for output %d',i)
    end
end

for i=1:EH.def.O.N
O(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i})));
end

O=mat2cell(O',ones(samples,1),EH.def.O.N);

end

