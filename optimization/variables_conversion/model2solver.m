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

function [solver] = model2solver(EH,S0,model,H,tm,solver)
%Convert matrix definition into solver-formart matrixes
%

%integer variables            
int=EH.feat.nx+1-EH.feat.nxint:EH.feat.nx; 
for k=1:H-1
int=[int...
    EH.feat.nx*k+EH.feat.nx+1-EH.feat.nxint:EH.feat.nx*(k+1)];
end
solver.intcon={int};

%variables for CPLEX
vars=char(double('C')*ones(EH.feat.nx*H,1));
vars(solver.intcon{:})='B';
solver.ctype={vars};
clearvars k

%lower and upper bounds %(M,P,Qch,Qdis,S,int_I,int_M,int_Qch,int_Qdis,int_devices) 
%lower bounds
solver.lb={reshape([model.M_min{:};model.P_min{:};...
    model.Qch_min{:};...
    model.Qdis_min{:};...
    model.S_min{:};...
    zeros(EH.def.I.N,H);...
    zeros(EH.def.O.N,H);... %dM
    zeros(EH.feat.nxint-(EH.def.I.N+EH.def.O.N),H)],...
    EH.feat.nx*H,1)};

%upper bounds
solver.ub={reshape([model.M_max{:};model.P_max{:};...
    model.Qch_max{:};...
    model.Qdis_max{:};model.S_max{:};...
    ones(EH.def.I.N,H);...
    repmat(EH.def.O.ms',1,H);... %dM
    ones(EH.feat.nxint-(EH.def.I.N+EH.def.O.N),H)],...
    EH.feat.nx*H,1)};

%equality and inequality constraints
[Aeq,beq] = eqcon(EH,H,tm,...
    EH.feat.nx,model.C{:},model.Cdo{:},model.O{:},model.Cdis{:},...
    model.Cch{:},model.Cs{:},S0);
solver.Aeq={Aeq};
solver.beq={beq};
[Aineq,bineq] = ineqcon(EH,H,...
    tm,EH.feat.nx,model.Qch_max{:},model.Qch_min{:},...
    model.Qdis_max{:},model.Qdis_min{:},model.I_max{:},...
    model.I_min{:},model.M_max{:},model.M_min{:},...
    model.Di_max{:},model.Di_min{:},model.Do_max{:},model.Do_min{:},...
    model.Ci{:},model.Cdi{:},model.Cdo{:});
solver.Aineq={Aineq};
solver.bineq={bineq};

%Get cost vector (cp) referred to variables in P (cp(k)=c(k)*Ci(k))
cp=NaN(EH.feat.paths.N,H);
for k=1:H
    cp(:,k)=reshape(model.c{:}(:,k),1,EH.def.I.N)*model.Ci{:}(:,:,k);
end

solver.fc={reshape([-model.s{:}; %
    cp;
    zeros(EH.feat.nx...
    -EH.def.O.N-EH.feat.paths.N,...
    H)],EH.feat.nx*H,1)};
end

