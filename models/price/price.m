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

function [c,s] = price(EH,data,date,samples,tm)  
%Returns the costs matrix for each input-market sale and sample
%

%Resample data according to tm
Time=downsample(data.Time,tm);
data=retime(data,Time,'mean');

%Select the suitable time range
TR=timerange(date,date+minutes(samples*tm));
data=data(TR,:);

%% Costs
%Preset
c=NaN(EH.def.I.N,samples);    
auxmin=0.0000001;

%Create one variable from either coeficients or functions containing the
%prices vector for the 'samples' horizon
for i=1:EH.def.I.N
    if isvarname(strcat(EH.def.I.t(i),EH.def.I.l(i)))
        if  ischar(EH.def.I.cost{i}) 
            if ismember(EH.def.I.cost{i},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.I.cost{i}),');'];
            else
                right_term=[num2str(EH.def.I.cost{i}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.I.cost{i})
            right_term=[num2str(EH.def.I.cost{i}),...
                '*ones(1,samples);'];
        else
            error('Invalid cost for input %d',i);
        end       
        eval(strcat('c_',strcat(EH.def.I.t(i),EH.def.I.l(i)),'=',right_term));       
    else
        error('Invalid name or label for input %d',i);
    end
end

%Bug detected: a difference of 0.111e-15 in water price gives very
%different results. %% Comment if not required.
%when using eval on value 1.1*0.497, the conversion is not precise enough
%and results in evaluating 1.1*0.497-0.111e-15 instead, which badly
%influence in the results obtained ¿maybe due to difficulties to represent
%a number with so many decimals?
if exist('c_Wi')==1
c_Wi=c_Wi+0.111e-15; %1.1*0.497+0.111e-15
end


%% Build cost vector
for k=1:EH.def.I.N   
c(k,:)=eval(char(strcat("c_",EH.def.I.t{k},EH.def.I.l{k})));

end
%set 0 cost to minimum cost to avoid "wasting" resources
c(c==0)=auxmin;
%end setting to 0
c=c/(60/tm); %costs per kW, m3/h o kg/h

%% Sales
%Preset
s=NaN(EH.def.O.N,samples);    

%Create one variable from either coeficients or functions containing the
%prices vector for the 'samples' horizon
for i=1:EH.def.O.N
    if isvarname(strcat(EH.def.O.t(i),EH.def.O.l(i)))
        if  ischar(EH.def.O.price{i}) 
            if ismember(EH.def.O.price{i},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.O.price{i}),');'];
            else
                right_term=[num2str(EH.def.O.price{i}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.O.price{i})
            right_term=[num2str(EH.def.O.price{i}),...
                '*ones(1,samples);'];
        else
            error('Invalid price for output %d',i);
        end
        eval(strcat('s_',strcat(EH.def.O.t(i),EH.def.O.l(i)),'=',right_term))       
    else
        error('Invalid name or label for output %d',i)
    end
end

for i=1:EH.def.O.N   
if EH.def.O.ms(i)>0
s(i,:)=eval(char(strcat("s_",EH.def.O.t{i},EH.def.O.l{i})));
else
s(i,:)=0;
end
end

s=s/(60/tm); %costs per kW, m3/h o kg/h

c=mat2cell(c',ones(samples,1),EH.def.I.N);
s=mat2cell(s',ones(samples,1),EH.def.O.N);

end

