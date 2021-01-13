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

function [model] = MPC_predictions(predict,EH,model,data,...
                                    prevdata,date,tm,H_MPC)                           
%Set the horizon and call dependant functions to obtain the matrixes that 
%model the problem  
                                
%prevdata will be employed in future releases for prediction models

if isempty(H_MPC)
    %variable horizon by electricity pricing
    %(See: "Heterogeneous resources management
    %in energy hubs with self-consumption: Contributions and
    %application example")    
    
    %Since prices usally are published 15 min before o'clock times
    %price publication
    
    %    MD      MI1     MI2     MI3     MI4     MI5     MI6     MI7
    %    D-1     D-1     D-1      D       D       D       D       D
    %   17:00   20:45   23:45   03:45   06:45   10:45   14:45   20:45
    
    date_vec=datevec(date);
    time=date_vec(4)+date_vec(5)/60+date_vec(6)/3600;
    if time>=18&&time<24
        H_MPC=min(EH.simparam.H_MPC,60*24);
    else
        H_MPC=min(EH.simparam.H_MPC,...
            60*(24-(date_vec(4)+date_vec(5)/60+date_vec(6)/3600)));
    end 
end
model.H=H_MPC;
model.samples=H_MPC/EH.simparam.tm_MPC;

%Read variables for simulation (nm samples)
%Output matrix (O)
if predict
    %To be defined in future releases
else
    O=outputs_matrix(EH,data,date,model.samples,tm);
end
model.O={cell2mat(O)'};

%Inputs limits (I)
if predict
    %To be defined in future releases
else
    [I_max,I_min]=input_lim_matrixes(EH,data,date,model.samples,tm);
end
model.I_max={cell2mat(I_max)'};
model.I_min={cell2mat(I_min)'};

%Devices inputs limits (D)
if predict
    %To be defined in future releases
else
    [Di_max,Di_min,Do_max,Do_min]=dev_lim_matrixes(EH,data,date,model.samples,tm);
end
model.Di_max={cell2mat(Di_max)'};
model.Di_min={cell2mat(Di_min)'};
model.Do_max={cell2mat(Do_max)'};
model.Do_min={cell2mat(Do_min)'};

%Market sales limits (M)
if predict
    %To be defined in future releases
else
    [M_max,M_min]=msales_lim_matrixes(EH,data,date,model.samples,tm);
end
model.M_max={cell2mat(M_max)'};
model.M_min={cell2mat(M_min)'};

%Path limits (P)
model.P_max={Inf*ones(EH.feat.paths.N,model.samples)};
model.P_min={0*ones(EH.feat.paths.N,model.samples)};

%Storage limits (S,Qdis,Qch)
if predict
    %To be defined in future releases
else
    [Qdis_min,Qdis_max,...
        Qch_min,Qch_max,...
        S_min,S_max]=storage_lim_matrixes(EH,data,date,model.samples,tm);
end
model.Qdis_min={cell2mat(Qdis_min)'};
model.Qdis_max={cell2mat(Qdis_max)'};
model.Qch_min={cell2mat(Qch_min)'};
model.Qch_max={cell2mat(Qch_max)'};
model.S_min={cell2mat(S_min)'};
model.S_max={cell2mat(S_max)'};

%conversion factors (C,Ci,Cdis,Cch,L)
if predict
    %To be defined in future releases
else
    [C,Ci,Cdis,Cch,Cs,Cdi,Cdo]=conversion_factors(EH,data,date,model.samples,tm);
end
model.C={cell2mat(reshape(C,[1,1,model.samples]))};
model.Ci={cell2mat(reshape(Ci,[1,1,model.samples]))};
model.Cdis={cell2mat(Cdis)'};
model.Cch={cell2mat(Cch)'};
model.Cs={cell2mat(Cs)'};
model.Cdi={cell2mat(reshape(Cdi,[1,1,model.samples]))};
model.Cdo={cell2mat(reshape(Cdo,[1,1,model.samples]))};

%cost/sales path matrixes (c,s)
if predict
    %To be defined in future releases
else
    [c,s]=price(EH,data,date,model.samples,tm);
end
model.c={cell2mat(c)'};
model.s={cell2mat(s)'};

% Fix single elements producing non-cell variables
notcell_variables=model.Properties.VariableNames(...
          not(varfun(@iscell,model,'OutputFormat','uniform')));
for j =1:length(notcell_variables)
    eval(strcat('model.',notcell_variables{j},...
        '=num2cell(model{:,notcell_variables(j)});'))
end
end