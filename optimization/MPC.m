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

function [results,solver] = MPC(EH,S0,model,H,tm,i)
%Get control acctions
%

%Measure computation time
tini=tic;

%Preset variables
results=EH.control.MPC.results(i,:);
solver=EH.control.MPC.solver(i,:);

%Get parameters/matrix for optimization solver
[solver] = model2solver(EH,S0,model,H,tm,solver);

%% Optimization parameters
%% MATLAB solver
if ~EH.simparam.YALMIP
    if strcmp(EH.simparam.solver,'MATLAB')
        if isempty(EH.simparam.solver_opt)
            options = optimoptions('intlinprog',...
                'Display','iter',...
                'AbsoluteGapTolerance',1e-4,...
                'MaxTime',500,...
                'CutGeneration','advanced',...
                'CutMaxIterations',50,...
                'ObjectiveCutOff',500,...
                'ObjectiveImprovementThreshold',1e-4,...
                'IntegerTolerance',1e-4,...
                'Heuristics','advanced',...
                'HeuristicsMaxNodes',1000,...
                'IntegerPreprocess','advanced'...
                );
        else
            options=EH.simparam.solver_opt;
        end
        [solver.XOPT{:},solver.FVAL,solver.EX,solver.OUT{:}]=...
            intlinprog(solver.fc{:},... %cost function
            solver.intcon{:},... %integer variables
            solver.Aineq{:},solver.bineq{:},... %ineq. constraints
            solver.Aeq{:},solver.beq{:},...  %eq. constraints
            solver.lb{:},solver.ub{:},... %lower and upper bounds
            options);
    elseif strcmp(EH.simparam.solver,'cplex')
        if isempty(EH.simparam.solver_opt)
            options = cplexoptimset('cplex');
        else
            options = EH.simparam.solver_opt;
        end
        %% CPLEX solver
        [solver.XOPT{:},solver.FVAL,solver.EX,solver.OUT{:}]=...
            cplexmilp(solver.fc{:}',... %cost function
            solver.Aineq{:},solver.bineq{:},... %ineq. constraints
            solver.Aeq{:},solver.beq{:},...  %eq. constraints
            [],[],[],... %SOS constraints
            solver.lb{:},solver.ub{:},... %lower and upper bounds
            solver.ctype{:}',... %variable type
            [],... %x0
            options);
    else
        error('Invalid solver selection.')
    end
else
    %% YALMIP    
    [solver.XOPT{:}] = yamilp(solver.fc{:},... %cost function
        solver.Aineq{:}, solver.bineq{:},... %ineq. constraints
        solver.Aeq{:}, solver.beq{:},... %eq. constraints
        solver.lb{:},solver.ub{:},... %lower and upper bounds
        solver.intcon{:},H, EH.feat.nx,... %binary and total variables
        EH.simparam.solver,EH.simparam.solver_opt); %solcver and options
end

%% Postprocessing
% Get results in form of matrixes
results = solver2model(EH,solver.XOPT{:},H,results);

%Measure computation time
solver.topt=toc(tini);
end

