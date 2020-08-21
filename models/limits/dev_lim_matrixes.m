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

function [Di_max,Di_min,...
    Do_max,Do_min] = dev_lim_matrixes(EH,data,date,samples,tm)
%Returns the matrixes for the upper and lower limits of each device
%

%Resample data according to tm
Time=downsample(data.Time,tm);
data=retime(data,Time,'mean');

%Select the suitable time range
TR=timerange(date,date+minutes(samples*tm));
data=data(TR,:);

Di_max=NaN(EH.feat.dev.N,samples); %Device matrix (Di)
Di_min=NaN(EH.feat.dev.N,samples); %Device matrix (Di)
Do_max=NaN(EH.feat.dev.N,samples); %Device matrix (Do)
Do_min=NaN(EH.feat.dev.N,samples); %Device matrix (Do)

auxmax=1000000;


%Create one variable from either coeficients or functions containing the
%device limit vector for the 'samples' horizon
for i=1:EH.def.dev.N
    if isvarname(EH.def.dev.D(i).l)
        for j=1:max(EH.def.dev.D(i).O.n)
            if  ischar(EH.def.dev.D(i).Dimin{j}) %Dimin
                if ismember(EH.def.dev.D(i).Dimin{j},data.Properties.VariableNames)
                    right_term=['transpose(data.',num2str(EH.def.dev.D(i).Dimin{j}),');'];
                else
                    right_term=[num2str(EH.def.dev.D(i).Dimin{j}),...
                        '(data,date,samples,tm);'];
                end
            elseif  isnumeric(EH.def.dev.D(i).Dimin{j})
                right_term=[num2str(EH.def.dev.D(i).Dimin{j}),...
                    '*ones(1,samples);'];
            else
                error('Invalid lower input capacity bound for device %d: %s',i,EH.def.dev.D(i).l);
            end
            eval(strcat(EH.def.dev.D(i).l,'_min_O',...
                num2str(j),'_i=',right_term));
            
            if  ischar(EH.def.dev.D(i).Dimax{j}) %Dimax
                if ismember(EH.def.dev.D(i).Dimax{j},data.Properties.VariableNames)
                    right_term=['transpose(data.',num2str(EH.def.dev.D(i).Dimax{j}),');'];
                else
                    right_term=[num2str(EH.def.dev.D(i).Dimax{j}),...
                        '(data,date,samples,tm);'];
                end
            elseif  isnumeric(EH.def.dev.D(i).Dimax{j})
                right_term=[num2str(EH.def.dev.D(i).Dimax{j}),...
                    '*ones(1,samples);'];
            else
                error('Invalid upper input capacity bound for device %d: %s',i,EH.def.dev.D(i).l);
            end
            eval(strcat(EH.def.dev.D(i).l,'_max_O',...
                num2str(j),'_i=',right_term));
            
             if  ischar(EH.def.dev.D(i).Domin{j}) %Domin
                if ismember(EH.def.dev.D(i).Domin{j},data.Properties.VariableNames)
                    right_term=['transpose(data.',num2str(EH.def.dev.D(i).Domin{j}),');'];
                else
                    right_term=[num2str(EH.def.dev.D(i).Domin{j}),...
                        '(data,date,samples,tm);'];
                end
            elseif  isnumeric(EH.def.dev.D(i).Domin{j})
                right_term=[num2str(EH.def.dev.D(i).Domin{j}),...
                    '*ones(1,samples);'];
            else
                error('Invalid lower output capacity bound for device %d: %s',i,EH.def.dev.D(i).l);
            end
            eval(strcat(EH.def.dev.D(i).l,'_min_O',...
                num2str(j),'_o=',right_term));
            
            if  ischar(EH.def.dev.D(i).Domax{j}) %Domax
                if ismember(EH.def.dev.D(i).Domax{j},data.Properties.VariableNames)
                    right_term=['transpose(data.',num2str(EH.def.dev.D(i).Domax{j}),');'];
                else
                    right_term=[num2str(EH.def.dev.D(i).Domax{j}),...
                        '(data,date,samples,tm);'];
                end
            elseif  isnumeric(EH.def.dev.D(i).Domax{j})
                right_term=[num2str(EH.def.dev.D(i).Domax{j}),...
                    '*ones(1,samples);'];
            else
                error('Invalid upper output capacity bound for device %d: %s',i,EH.def.dev.D(i).l);
            end
            eval(strcat(EH.def.dev.D(i).l,'_max_O',...
                num2str(j),'_o=',right_term));
        end
    else
        error('Invalid name or label for device %d',i);
    end
end

%Build matrixes
for i=1:length(EH.feat.dev.dev)
aux1=strfind(EH.feat.dev.dev(i),"O");
if isempty(aux1)==1
    device=EH.feat.dev.dev2(i,1); %device n
    Di_max(i,:)=eval(char(strcat(EH.def.dev.D(device).l,...
        "_max_O1_i")));
    Di_min(i,:)=eval(char(strcat(EH.def.dev.D(device).l,...
        "_min_O1_i")));   
    Do_max(i,:)=eval(char(strcat(EH.def.dev.D(device).l,...
        "_max_O1_o")));
    Do_min(i,:)=eval(char(strcat(EH.def.dev.D(device).l,...
        "_min_O1_o")));
else
    device=EH.feat.dev.dev2(i,1); %device n
    output=EH.feat.dev.dev2(i,2); %output n of device n       
    Di_max(i,:)=eval(char(strcat(EH.def.dev.D(device).l,...
        "_max_O",string(output),"_i")));
    Di_min(i,:)=eval(char(strcat(EH.def.dev.D(device).l,...
        "_min_O",string(output),"_i")));
    Do_max(i,:)=eval(char(strcat(EH.def.dev.D(device).l,...
        "_max_O",string(output),"_o")));
    Do_min(i,:)=eval(char(strcat(EH.def.dev.D(device).l,...
        "_min_O",string(output),"_o")));
end


end

%Cap inf values
Di_max(Di_max==inf)=auxmax;
Do_max(Do_max==inf)=auxmax;

Di_max=mat2cell(Di_max',ones(samples,1),length(EH.feat.dev.dev));
Di_min=mat2cell(Di_min',ones(samples,1),length(EH.feat.dev.dev));
Do_max=mat2cell(Do_max',ones(samples,1),length(EH.feat.dev.dev));
Do_min=mat2cell(Do_min',ones(samples,1),length(EH.feat.dev.dev));



end

