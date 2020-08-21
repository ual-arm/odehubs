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

function [data] = EH_system(EH,S0,model,control_action,tm,k)
%Simulate EH system. Returns the plant response. To be developed in future
%releases.

%% Preset variables
H=1;
data=EH.model.data(k,:);
data.infeas=0;

%Set data behaviur as predicted from control action
data=solver2model(EH,control_action,H,data); %(see optimization > variables_conversion folder)
data.S0{:}=S0;

%% Cap variables according to real bounds

%Inputs (to be included in future releases)
% I_control=model.Ci{:}*data.P{:};
% I_model=min(I_control,model.I_max{:}.*data.dI{:});
% I_model=max(I_model,model.I_min{:}.*data.dI{:});
% 
% I_cap=I_model./I_control; %cap ratio
% I_cap(isnan(I_cap))=1;
% 
% %Get cap referred to variables in P
% data.P{:}=data.P{:}.*(I_cap'*model.Ci{:})';

%Devices (to be included in future releases)
%code to cap P due to devices production limits goes here

%Get the real state of the system (S)

%Build dO matrix
dO=zeros(EH.def.O.N,size(data.dD{:},2));
for j=1:EH.def.O.N
    if EH.def.O.d(j)>0
        dev=find((EH.feat.dev.dev2(:,1)==EH.def.O.d(j))&...
            (EH.feat.dev.dev2(:,3)==EH.def.O.dO(j)));
        dO(j,:)=data.dD{:}(dev,:);
    elseif EH.def.O.ch(j)>0&&~EH.def.O.dis(j)>0
        dO(j,:)=data.dQch{:}(EH.def.O.ch(j),:)*(EH.def.O.ch(j)>0);
    elseif EH.def.O.dis(j)>0&&~EH.def.O.ch(j)
        dO(j,:)=data.dQdis{:}(EH.def.O.dis(j),:)*(EH.def.O.dis(j)>0);
    elseif EH.def.O.ch(j)>0&&EH.def.O.dis(j)>0
        dO(j,:)=data.dQdch{:}(EH.def.O.ch(j),:)*(EH.def.O.ch(j)>0)+...
            data.dQdis{:}(EH.def.O.dis(j),:)*(EH.def.O.dis(j)>0);
    else
        dO(j,:)=ones(1,size(data.dD{:},2));
    end
end

%I-O balance
Q=data.M{:}+diag(dO)*model.O{:}-model.C{:}*data.P{:};

Q_model=Q;
data.Qdis{:}(Q==0)=0;
data.Qch{:}(Q==0)=0;
data.dQdis{:}(Q==0)=0;
data.dQch{:}(Q==0)=0;

%cap discharge
Q_model(Q>0)=min(Q(Q>0),model.Qdis_max{:}(Q>0));
Q_model(Q>0)=max(Q_model(Q>0),model.Qdis_min{:}(Q>0));
data.Qdis{:}(Q>0)=Q_model(Q>0);
data.Qch{:}(Q>0)=0;
data.dQdis{:}(Q>0)=1;
data.dQch{:}(Q>0)=0;

%cap charge
Q_model(Q<0)=min(-Q(Q<0),model.Qch_max{:}(Q<0));
Q_model(Q<0)=max(Q_model(Q<0),model.Qch_min{:}(Q<0));
data.Qdis{:}(Q<0)=0;
data.Qch{:}(Q<0)=Q_model(Q<0);
data.dQdis{:}(Q<0)=1;
data.dQch{:}(Q<0)=0;

%infeasibilities
tol=1e-12;
if max((abs(Q_model)-abs(Q))>tol)~=0
data.infeas=data.infeas+sum((abs(Q_model)-abs(Q))>tol);
end

%Storage balance
S=diag(model.Cs{:})*data.S0{:}...
          +tm/60*diag(model.Cch{:})*data.Qch{:}...
          -tm/60*diag(model.Cdis{:})*data.Qdis{:};

%cap storage
S_model=min(S,model.S_max{:});
S_model=max(S_model,model.S_min{:});
data.S{:}=S;

if max(abs(S_model)-abs(S)>tol)~=0
data.infeas=data.infeas+sum((abs(Q_model)-abs(Q))>tol);
end

%Account for constraints violation (infeasibilities)
if data.infeas>0
    keyboard
    warning('%d contraints violation at iteration %d',data.infeas,k)
end

end

