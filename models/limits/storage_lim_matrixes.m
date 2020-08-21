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

function [Qd_min,Qd_max,...
          Qc_min,Qc_max,...
          S_min,S_max] = storage_lim_matrixes(EH,data,date,samples,tm)
%Returns the matrixes for the upper and lower limits of storage device
%

%Resample data according to tm
Time=downsample(data.Time,tm);
data=retime(data,Time,'mean');

%Select the suitable time range
TR=timerange(date,date+minutes(samples*tm));
data=data(TR,:);

auxmax=10000000; %generic upper limit

Qd_max=NaN(EH.def.O.N,samples); %Discharge matrix (Qd)
Qd_min=NaN(EH.def.O.N,samples); %Discharge matrix (Qd)
Qc_max=NaN(EH.def.O.N,samples); %Charge matrix (Qc)
Qc_min=NaN(EH.def.O.N,samples); %Charge matrix (Qc)
S_max=NaN(EH.def.O.N,samples); %Storage matrix (S)
S_min=NaN(EH.def.O.N,samples); %Storage matrix (S)

%Create one variable from either coeficients or functions containing the
%storage limit vector for the 'samples' horizon
for i=1:length(EH.def.dev.S)
    if isvarname(EH.def.dev.S(i).l)
        if  ischar(EH.def.dev.S(i).Qchmin{:}) %charge min
            if ismember(EH.def.dev.S(i).Qchmin{:},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.dev.S(i).Qchmin{:}),');'];
            else
                right_term=[num2str(EH.def.dev.S(i).Qchmin{:}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.dev.S(i).Qchmin{:})
            right_term=[num2str(EH.def.dev.S(i).Qchmin{:}),...
                '*ones(1,samples);'];
        else
            error('Invalid lower charge bound for storage device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_c_min=',right_term));
        
        if  ischar(EH.def.dev.S(i).Qchmax{:}) %charge max
            if ismember(EH.def.dev.S(i).Qchmax{:},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.dev.S(i).Qchmax{:}),');'];
            else
                right_term=[num2str(EH.def.dev.S(i).Qchmax{:}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.dev.S(i).Qchmax{:})
            right_term=[num2str(EH.def.dev.S(i).Qchmax{:}),...
                '*ones(1,samples);'];
        else
            error('Invalid upper charge bound for storage device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_c_max=',right_term));
        
        if  ischar(EH.def.dev.S(i).Qdismin{:}) %discharge min
            if ismember(EH.def.dev.S(i).Qdismin{:},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.dev.S(i).Qdismin{:}),');'];
            else
                right_term=[num2str(EH.def.dev.S(i).Qdismin{:}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.dev.S(i).Qdismin{:})
            right_term=[num2str(EH.def.dev.S(i).Qdismin{:}),...
                '*ones(1,samples);'];
        else
            error('Invalid lower discharge bound for storage device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_d_min=',right_term));
        
        if  ischar(EH.def.dev.S(i).Qdismax{:}) %discharge max
            if ismember(EH.def.dev.S(i).Qdismax{:},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.dev.S(i).Qdismax{:}),');'];
            else
                right_term=[num2str(EH.def.dev.S(i).Qdismax{:}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.dev.S(i).Qdismax{:})
            right_term=[num2str(EH.def.dev.S(i).Qdismax{:}),...
                '*ones(1,samples);'];
        else
            error('Invalid upper discharge bound for storage device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_d_max=',right_term));
        
        if  ischar(EH.def.dev.S(i).Smin{:}) %capacity min
            if ismember(EH.def.dev.S(i).Smin{:},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.dev.S(i).Smin{:}),');'];
            else
                right_term=[num2str(EH.def.dev.S(i).Smin{:}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.dev.S(i).Smin{:})
            right_term=[num2str(EH.def.dev.S(i).Smin{:}),...
                '*ones(1,samples);'];
        else
            error('Invalid lower capacity bound for storage device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_s_min=',right_term));
        
        if  ischar(EH.def.dev.S(i).Smax{:}) %capacity max
            if ismember(EH.def.dev.S(i).Smax{:},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.dev.S(i).Smax{:}),');'];
            else
                right_term=[num2str(EH.def.dev.S(i).Smax{:}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.dev.S(i).Smax{:})
            right_term=[num2str(EH.def.dev.S(i).Smax{:}),...
                '*ones(1,samples);'];
        else
            error('Invalid upper capacity bound for storage device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_s_max=',right_term));
        
        
    else
        error('Invalid name or label for device %d',i)
    end
end

%Build matrixes
for i=1:EH.def.O.N
if EH.def.O.st(i)>0 %Storage system
Qd_max(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_d_max")));
Qd_min(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_d_min")));
Qc_max(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_c_max")));
Qc_min(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_c_min")));
S_max(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_s_max")));
S_min(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_s_min")));
else 
Qd_max(i,:)=zeros(1,samples);
Qd_min(i,:)=zeros(1,samples);
Qc_max(i,:)=zeros(1,samples);
Qc_min(i,:)=zeros(1,samples);
S_max(i,:)=zeros(1,samples);
S_min(i,:)=zeros(1,samples);
end
end

%Cap inf values
Qd_max(Qd_max==inf)=auxmax;
Qc_max(Qc_max==inf)=auxmax;
S_max(S_max==inf)=auxmax;

Qd_max=mat2cell(Qd_max',ones(samples,1),EH.def.O.N);
Qd_min=mat2cell(Qd_min',ones(samples,1),EH.def.O.N);
Qc_max=mat2cell(Qc_max',ones(samples,1),EH.def.O.N);
Qc_min=mat2cell(Qc_min',ones(samples,1),EH.def.O.N);
S_max=mat2cell(S_max',ones(samples,1),EH.def.O.N);
S_min=mat2cell(S_min',ones(samples,1),EH.def.O.N);
end

