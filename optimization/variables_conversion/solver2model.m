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

function [results] = solver2model(EH,XOPT,H,results)
%Convert solver-formart matrixes into matrix definition
%


%(M,P,Qch,Qdis,S,int_I,int_M,int_Qch,int_Qdis,int_devices)
Xaux=reshape(XOPT,EH.feat.nx,H);
results.M{:}=Xaux(1:EH.def.O.N,:);
results.P{:}=Xaux(EH.def.O.N+1:EH.def.O.N+EH.feat.paths.N,:);
results.Qch{:}=Xaux(EH.def.O.N+EH.feat.paths.N+1:...
    EH.def.O.N+EH.feat.paths.N+EH.def.O.N,:);
results.Qdis{:}=Xaux(EH.def.O.N+EH.feat.paths.N+EH.def.O.N+1:...
    EH.def.O.N+EH.feat.paths.N+EH.def.O.N+EH.def.O.N,:);
results.S{:}=Xaux(EH.def.O.N+EH.feat.paths.N+EH.def.O.N*2+1:...
    EH.def.O.N+EH.feat.paths.N+EH.def.O.N*2+EH.def.O.N,:);
results.dI{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N+1:...
    EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N+EH.def.I.N,:);
results.dM{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
    +EH.def.I.N+1:EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
    +EH.def.I.N+EH.def.O.N,:);
results.dQch{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
    +EH.def.I.N+EH.def.O.N+1:...
    EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
    +EH.def.I.N+EH.def.O.N+EH.def.O.N,:);
results.dQdis{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
    +EH.def.I.N+EH.def.O.N+EH.def.O.N+1:...
    EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
    +EH.def.I.N+EH.def.O.N+EH.def.O.N+EH.def.O.N,:);
results.dD{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
    +EH.def.I.N+EH.def.O.N+EH.def.O.N+EH.def.O.N+1:...
    EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
    +EH.def.I.N+EH.def.O.N+EH.def.O.N+EH.def.O.N+EH.feat.dev.N,:);
end

