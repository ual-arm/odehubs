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

function [data,solver] = EH_system_old(EH,S0,model,control,tm,k)
%Simulate EH system. Returns the plant response taking into account that 
%the feasible decision variables must be as equal to control variables as
%possible. To be developed in future releases.

H=1;

%Measure computation time
tini=tic;

%Preset variables
data=EH.model.data(k,:);

EH.model.solver=repmat(struct(...
                    'lb',NaN(EH.feat.nx*1,1),... %lower bounds
                    'ub',NaN(EH.feat.nx*1,1),... %upper bounds
                    'intcon',NaN(EH.feat.nxint*1,1),... %interger vars
                    'ctype',cell(1,1),... %variable type (for CPLEX)
                    'Hc',NaN(EH.feat.nx*1,EH.feat.nx*1),... %cost func
                    'fc',NaN(EH.feat.nx*1,1),... %cost func
                    'x0',NaN(EH.feat.nx*1,1),... %cost func
                    'Aeq',NaN(EH.feat.eqconN*1,...
                              EH.feat.nx*1),... %eq. const
                    'beq',NaN(EH.feat.eqconN*1,1),... %eq. const
                    'Aineq',NaN(EH.feat.ineqconN*1,...
                                EH.feat.nx*1),... %ineq. const
                    'bineq',NaN(EH.feat.ineqconN*1,1),... %ineq. const
                    'XOPT',NaN(EH.feat.nx*1,1),... %optim. results
                    'OUT',struct([]),... %optim. out
                    'FVAL',NaN(1,1),... %optim. fval
                    'EX',NaN(1,1),... %optim. exit
                    'topt',NaN(1,1)),... %optim. time
                     EH.simparam.nm/EH.simparam.tm, 1); 

EH.model.solver=struct2table(EH.model.solver); 
EH.model.solver.Time=downsample(EH.simparam.data.Time(1:EH.simparam.nm),...
                    EH.simparam.tm);      
EH.model.solver=table2timetable(EH.model.solver); 


solver=EH.model.solver(k,:);

%Initial state
solver.x0{:}=control;

%Get parameters/matrix for optimization solver
[solver] = model2solver(EH,S0,model,H,tm,solver); %H=1

%Get cost vector and matrix for MIQP
%(M,P,Qch,Qdis,S,int_I,int_M,int_Qch,int_Qdis,int_devices) 
%H is a diagonal matrix so 0.5*x'Hx = x'x
%since x contains more decision variables than the ones involved in the
%function cost, the diagonal's elements are set to 0 in order to ignore
%the decision variables which do not appear in the function cost
solver.Hc{:}=diag([2*ones(EH.def.O.N+EH.feat.paths.N+...
                        EH.def.O.N+EH.def.O.N,1);...
                   zeros(EH.feat.nx-(EH.def.O.N+EH.feat.paths.N+...
                        EH.def.O.N+EH.def.O.N),1)]);
                    
%f is a vector so f*x = -2*xobj*x
%xobjt contain the values obtained in the control layer for M,P,Qch & Qdis
%while the rest of values are set to zero
solver.fc{:}=-2*[control(1:EH.def.O.N+EH.feat.paths.N+...
                        EH.def.O.N+EH.def.O.N);...
              zeros(EH.feat.nx-(EH.def.O.N+EH.feat.paths.N+...
                        EH.def.O.N+EH.def.O.N),1)];           

     
%% Optimization        
%% CPLEX solver   
[solver.XOPT{:},solver.FVAL,solver.EX,solver.OUT{:}]=...  
                    cplexmiqp(solver.Hc{:},solver.fc{:},... %costs function
                    solver.Aineq{:},solver.bineq{:},... %ineq. constraints
                    solver.Aeq{:},solver.beq{:},...  %eq. constraints
                    [],[],[],... %SOS constraints
                    solver.lb{:},solver.ub{:},... %lower and upper bounds
                    solver.ctype{:}',... %variable type
                    solver.x0{:}... %initial state
                    );            

%% Get results in form of matrix
%(M,P,Qch,Qdis,S,int_I,int_M,int_Qch,int_Qdis,int_devices) 
Xaux=reshape(solver.XOPT{:},EH.feat.nx,H);
data.M{:}=Xaux(1:EH.def.O.N,:);
data.P{:}=Xaux(EH.def.O.N+1:EH.def.O.N+EH.feat.paths.N,:);
data.Qch{:}=Xaux(EH.def.O.N+EH.feat.paths.N+1:...
        EH.def.O.N+EH.feat.paths.N+EH.def.O.N,:);
data.Qdis{:}=Xaux(EH.def.O.N+EH.feat.paths.N+EH.def.O.N+1:...
        EH.def.O.N+EH.feat.paths.N+EH.def.O.N+EH.def.O.N,:);    
data.S{:}=Xaux(EH.def.O.N+EH.feat.paths.N+EH.def.O.N*2+1:...
        EH.def.O.N+EH.feat.paths.N+EH.def.O.N*2+EH.def.O.N,:);
data.dI{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N+1:...
        EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N+EH.def.I.N,:);    
data.dM{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
        +EH.def.I.N+1:EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
        +EH.def.I.N+EH.def.O.N,:);
data.dQch{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
        +EH.def.I.N+EH.def.O.N+1:...
        EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
        +EH.def.I.N+EH.def.O.N+EH.def.O.N,:);
data.dQdis{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
        +EH.def.I.N+EH.def.O.N+EH.def.O.N+1:...
        EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
        +EH.def.I.N+EH.def.O.N+EH.def.O.N+EH.def.O.N,:);
data.dD{:}=Xaux(EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
        +EH.def.I.N+EH.def.O.N+EH.def.O.N+EH.def.O.N+1:...
        EH.def.O.N+EH.feat.paths.N+3*EH.def.O.N...
        +EH.def.I.N+EH.def.O.N+EH.def.O.N+EH.def.O.N+EH.feat.dev.N,:);  
solver.topt=toc(tini);
clearvars tini
end

