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

function [x] = yamilp(fc, Aineq, bineq, Aeq, beq, lb, ub,...
    intcon, H, nx, solver, ops)
%Problem definition for YALMIP
%

%% Preprocess
yalmip('clear')
x = sdpvar(repmat(nx*H,1,1),repmat(1,1,1));
constraints = [];

%% Cost function
objective = sum(fc.*x);

%% Constraints
% Binary variables
for i = 1 : length(intcon)
    a = intcon(i);
    constraints = [constraints, binary(x(a))];
end

constraints = [constraints, Aineq*x <= bineq]; % Ineq.
constraints = [constraints, Aeq*x == beq]; % Eq.
constraints = [constraints, lb <= x <= ub]; % Bounds

%% Optimization problem
if isempty(ops)
    ops = sdpsettings('solver',solver, 'warning','1','debug',1);
    optimize(constraints, objective, ops);
else
    optimize(constraints, objective, ops);
end

%% Results
x = value(x);
end


