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


%% Simulation main file
% Set and parameters and execute the simulation
% The EH struct contains all the data of the problem/system

%% Preprocess
clear
clc
close all
%Determine the m-file's folder location
EH.def.folder = fileparts(which(mfilename));
%Add that folder plus all subfolders to the path
addpath(genpath(EH.def.folder));

%% EH definition from MATLAB code or Simulink
%switch to 'code' to define the EH via MATLAB code or type the name of a
%valid Simulink file which contains the definition of the EH

EH.def.definition='EH_agr_cluster';

if strcmp(EH.def.definition,'code')
    [EH] = EH_definition_code(EH);
    fprintf('Loading definition from code. \n')
else
    fprintf('Loading definition from Simulink. \n')
    try
        %Load the file that contains the EH definition
        load_system(EH.def.definition)
    catch        
        error('EH not defined, switch to code or a valid Simulink file.')
    end
    [EH] = EH_definition_Simulink(EH);
end
EH=EH_features(EH); %Variables depending on EH definition (features)

%% Simulation parameters
EH.simparam.nm=60*24;       %simulation length in minutes
EH.simparam.tm=60;         %plant model sample time in minutes
EH.simparam.tm_MPC=60;     %MPC sample time in minutes
EH.simparam.H_MPC=60*24;    %max horizon in minutes
EH.simparam.date=[2018 12 17 0 0 0];     %start day for simulations
EH.simparam.MPCmode=false;  %switch to 'false' to use scheduling mode
EH.simparam.YALMIP=false; %switch to 'true' to interface YALMIP
EH.simparam.solver='MATLAB'; %select 'MATLAB', 'cplex' or any YALMIP compatible solver
EH.simparam.solver_opt=[]; %define solver options according to the solver selected
EH.simparam.pred=false; %switch to 'true' to load data from the previous day
EH.results.exportgraphics=true; %switch to 'true' to save grahics
EH.results.showmodel=false; %show model simulation or control actions

%% Load all data required by the models
fprintf('Loading data for simulation. \n')
[EH] = load_data(EH);

%% Variables preset
[EH] = preset(EH);

%% Initial conditions
EH.model.data.S0{1}=S0_func(EH);

%% Control loop
fprintf('\nSimulation starts:\n')
for k=1:EH.simparam.nm/EH.simparam.tm
    
    %Execute each tm_MPC period or just once if scheduling mode selected
    if mod(k-1,EH.simparam.tm_MPC/EH.simparam.tm)==0
        %Get iteration number
        i=(k-1)/(EH.simparam.tm_MPC/EH.simparam.tm)+1;
        if EH.simparam.MPCmode||(~EH.simparam.MPCmode&&i==1)
            %Get prediction data for MPC control
            EH.control.MPC.model(i,:)=...
                MPC_predictions(false,... %true for pred. models/false for data
                EH,EH.control.MPC.model(i,:),...
                EH.simparam.data,...
                [],... %prevdata will be employed in future releases for prediction models
                EH.simparam.dateV(k),EH.simparam.tm_MPC,...
                []); %switch to EH.simparam.H_MPC for fixed horixon
            %Get control signals for the plant (MPC results)
            [EH.control.MPC.results(i,:),...
                EH.control.MPC.solver(i,:)]=MPC(...
                EH,EH.model.data.S0{k},...
                EH.control.MPC.model(i,:),...
                EH.control.MPC.model(i,:).samples{:},...
                EH.simparam.tm_MPC,i);
            control_action=EH.control.MPC.solver.XOPT{i}(1:EH.feat.nx);
        end
        if ~EH.simparam.MPCmode
            if EH.simparam.nm>EH.simparam.H_MPC
                error('Scheduling horizon too shoort. Simulation aborted.');
            else
                control_action=...
                    EH.control.MPC.solver.XOPT{1}...
                    (1+EH.feat.nx*(i-1):EH.feat.nx*i);
            end
        end
    end
    
    %Simulate energy hub response (beta version)
    [EH.model.data(k,:)]=...
        EH_system(EH,EH.model.data.S0{k},...
        EH.model.param(k,:),...
        control_action,...
        EH.simparam.tm,k);
    
    
     %[EH.model.data(k,:),...
        %EH.model.solver(k,:)]=...
        %EH_system_old(EH,EH.model.data.S0{k},...
        %EH.model.param(k,:),...
        %control_action,...
        %EH.simparam.tm,k);
    
    %Get next state (storage state)
    if (k<=EH.simparam.nm/EH.simparam.tm-1)&&EH.simparam.MPCmode
        if EH.results.showmodel
            EH.model.data.S0{k+1}=EH.model.data.S{k}; %(beta version)
        else
            EH.model.data.S0{k+1}=EH.control.MPC.results.S{i}(:,1);
        end
    end
    
    if EH.simparam.MPCmode
        fprintf('\nIteration %d of %d. \n',k,EH.simparam.nm/EH.simparam.tm)
    elseif k==1
        fprintf('\nIteration 1 of 1.\n')
    end
end
clearvars -except EH
fprintf('\nSimulation completed. \n')
%% Save and show results
%Save results in a file named according to date
save(strcat(EH.def.folder,"\results\","Sim_",...
    strrep(datestr(EH.simparam.date,'yyyy-mm-dd'),'-','_')),'EH')
close all
EH=process_results(EH,EH.results.showmodel);