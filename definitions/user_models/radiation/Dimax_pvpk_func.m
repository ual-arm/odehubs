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

function PVpk_max = Dimax_pvpk_func(data,date,samples,tm)
%User defined function for ODEHub Examples
%

dates=datevec(data.Time); 

%Tempreature and radiation data
G=data.G(1:samples);
Gbn=data.Gbn(1:samples);
Gd=data.Gd(1:samples);

%PVpk limits
beta_pvpk=7;
phi_pvpk=36.83;
psi_pvpk=-2.40;
gamma_pvpk=-21;
Act_pvpk=8110.30;
%Scaled to the tenth
Act_pvpk=Act_pvpk/10;

Gtpvpk=...
                Gt_model(G,Gd,Gbn,...
                dates(:,4)+dates(:,5)/60+dates(:,6)/3600,dates(:,1:3),...
                beta_pvpk,phi_pvpk,0,... %psiref=0
                psi_pvpk,gamma_pvpk,0.1,... %ro_g=0.1
                tm,...  %tm
                0); %sky_model
PVpk_max=Gtpvpk'*Act_pvpk/1000;

end