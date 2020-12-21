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

function [I_max,I_min] = input_lim_matrixes(EH,data,date,samples,tm)
%Returns the matrixes for the upper and lower limits of each input
%

%Resample data according to tm
Time=downsample(data.Time,tm);
data=retime(data,Time,'mean');

%Select the suitable time range
TR=timerange(date,date+minutes(samples*tm));
data=data(TR,:);

I_max=NaN(EH.def.I.N,samples); %Intput matrix (I)
I_min=NaN(EH.def.I.N,samples); %Intput matrix (I)

auxmax=10000000;

%Create one variable from either coeficients or functions containing the
%input limit vector for the 'samples' horizon
for i=1:EH.def.I.N
    if isvarname(strcat(EH.def.I.t(i),EH.def.I.l(i)))
        if  ischar(EH.def.I.Imin{i}) %min
            if ismember(EH.def.I.Imin{i},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.I.Imin{i}),');'];
            else
                right_term=[num2str(EH.def.I.Imin{i}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.I.Imin{i})
            right_term=[num2str(EH.def.I.Imin{i}),...
                '*ones(1,samples);'];
        else
            error('Invalid lower input bound for output %d: %s',i,...
           strcat(EH.def.I.t(i),EH.def.I.l(i)));
        end
        eval(strcat(strcat(EH.def.I.t(i),EH.def.I.l(i)),'_min=',right_term))
        
        if  ischar(EH.def.I.Imax{i}) %max
            if ismember(EH.def.I.Imax{i},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.I.Imax{i}),');'];
            else
                right_term=[num2str(EH.def.I.Imax{i}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.I.Imax{i})
            right_term=[num2str(EH.def.I.Imax{i}),...
                '*ones(1,samples);'];
        else
            error('Invalid upper input bound for output %d: %s',i,...
           strcat(EH.def.I.t(i),EH.def.I.l(i)));
        end
        eval(strcat(strcat(EH.def.I.t(i),EH.def.I.l(i)),'_max=',right_term))       
    else
        error('Invalid name or label for input %d',i)
    end
end

% %Ei limits
% Ei_max=auxmax*ones(1,samples);
% Ei_min=auxmin+zeros(1,samples);
% 
% %Ri limits
% Rpk_max=auxmax*ones(1,samples);
% Rpk_min=auxmin+zeros(1,samples);
% Rcs_max=auxmax*ones(1,samples);
% Rcs_min=auxmin+zeros(1,samples);
% Ri_max=auxmax*ones(1,samples);
% Ri_min=auxmin+zeros(1,samples);
% 
% %Pi limits
% Pi_max=auxmax*ones(1,samples);
% Pi_min=auxmin+zeros(1,samples);
% 
% %Bi limits
% Bi_max=auxmax*ones(1,samples);
% Bi_min=auxmin+zeros(1,samples);
% 
% %Wi limits
% Wi_max=auxmax*ones(1,samples);
% Wi_min=auxmin+zeros(1,samples);
% 
% %Sdi limits
% Si_max=auxmax*ones(1,samples);
% Si_min=auxmin+zeros(1,samples);

%I matrixes
for i=1:EH.def.I.N
I_max(i,:)=eval(char(strcat(EH.def.I.t{i},EH.def.I.l{i},"_max")));
I_min(i,:)=eval(char(strcat(EH.def.I.t{i},EH.def.I.l{i},"_min")));
end

%Cap inf values
I_max(I_max==inf)=auxmax;

I_max=mat2cell(I_max',ones(samples,1),EH.def.I.N);
I_min=mat2cell(I_min',ones(samples,1),EH.def.I.N);

end

