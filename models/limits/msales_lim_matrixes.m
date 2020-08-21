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

function [M_max,M_min] = msales_lim_matrixes(EH,data,date,samples,tm)
%Returns the matrixes for the upper and lower limits of each sale
%

%Resample data according to tm
Time=downsample(data.Time,tm);
data=retime(data,Time,'mean');

%Select the suitable time range
TR=timerange(date,date+minutes(samples*tm));
data=data(TR,:);

M_max=NaN(EH.def.O.N,samples); %Intput matrix (M)
M_min=NaN(EH.def.O.N,samples); %Intput matrix (M)

auxmax=10000000; %generic upper limit

%Create one variable from either coeficients or functions containing the
%market limit vector for the 'samples' horizon
for i=1:EH.def.O.N
    if isvarname(strcat(EH.def.O.l(i),EH.def.O.t(i)))
        if  ischar(EH.def.O.Mmin{i}) %min
            if ismember(EH.def.O.Mmin{i},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.O.Mmin{i}),');'];
            else
                right_term=[num2str(EH.def.O.Mmin{i}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.O.Mmin{i})
            right_term=[num2str(EH.def.O.Mmin{i}),...
                '*ones(1,samples);'];
        else
            error('Invalid lower market bound for output %d: %s',i,...
           strcat(EH.def.O.t(i),EH.def.O.l(i)));
        end
        eval(strcat(strcat(EH.def.O.t(i),EH.def.O.l(i)),'_m_min=',right_term))
        
        if  ischar(EH.def.O.Mmax{i}) %max
            if ismember(EH.def.O.Mmax{i},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.O.Mmax{i}),');'];
            else
                right_term=[num2str(EH.def.O.Mmax{i}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.O.Mmax{i})
            right_term=[num2str(EH.def.O.Mmax{i}),...
                '*ones(1,samples);'];
        else
            error('Invalid upper market bound for output %d: %s',i,...
           strcat(EH.def.O.t(i),EH.def.O.l(i)));
        end
        eval(strcat(strcat(EH.def.O.t(i),EH.def.O.l(i)),'_m_max=',right_term))       
    else
        error('Invalid name or label for output %d',i)
    end
end

for i=1:EH.def.O.N
if EH.def.O.ms(i)>0
M_max(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_m_max")));
M_min(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_m_min")));
else
M_max(i,:)=zeros(1,samples);
M_min(i,:)=zeros(1,samples);
end
end

%Cap inf values
M_max(M_max==inf)=auxmax;


M_max=mat2cell(M_max',ones(samples,1),EH.def.O.N);
M_min=mat2cell(M_min',ones(samples,1),EH.def.O.N);
end

