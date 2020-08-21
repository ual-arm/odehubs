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

function [EH] = preset(EH)
%Preset the size and content of matrixes as verification
%

%% Matrix size for MPC optimization
EH.simparam.samples_MPC=EH.simparam.H_MPC/EH.simparam.tm_MPC; %number of samples for optimization
if EH.simparam.MPCmode
    rep_MPC=EH.simparam.nm/EH.simparam.tm_MPC; %number of time that optimization is executed
else
    rep_MPC=1; %number of time that optimization is executed
end
EH.simparam.samples=EH.simparam.nm/EH.simparam.tm; %number of samples for simulation
EH.control.MPC.model=repmat(struct(...
    'H',EH.simparam.H_MPC+zeros(1,1),... %MPC Horizon
    'samples',EH.simparam.samples_MPC+zeros(1,1),... %MPC samples
    'O',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Output matrix (O)
    'I_max',NaN(EH.def.I.N,EH.simparam.samples_MPC),... %Intput matrix (I)
    'I_min',NaN(EH.def.I.N,EH.simparam.samples_MPC),... %Intput matrix (I)
    'M_max',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Market sales matrix (M)
    'M_min',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Market sales matrix (M)
    'P_max',NaN(EH.feat.paths.N,EH.simparam.samples_MPC),... %Path matrix (P)
    'P_min',NaN(EH.feat.paths.N,EH.simparam.samples_MPC),... %Path matrix (P)
    'Di_max',NaN(EH.feat.dev.N,EH.simparam.samples_MPC),... %Internal matrix (Di)
    'Di_min',NaN(EH.feat.dev.N,EH.simparam.samples_MPC),... %Internal matrix (Di)
    'Do_max',NaN(EH.feat.dev.N,EH.simparam.samples_MPC),... %Internal matrix (Do)
    'Do_min',NaN(EH.feat.dev.N,EH.simparam.samples_MPC),... %Internal matrix (Do)
    'Qdis_min',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Discharge (Qd)
    'Qdis_max',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Discharge (Qd)
    'Qch_min',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Charge (Qc)
    'Qch_max',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Charge (Qc)
    'S_min',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Storage (S)
    'S_max',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Storage (S)
    'Cdis',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Discharge efficiency (Cdis)
    'Cch',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Charge efficiency (Cch)
    'Cs',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Degradation (Cs)
    'C',NaN(EH.def.O.N,...
    EH.feat.paths.N,EH.simparam.samples_MPC),... %Coupling matrix(C)
    'Ci',NaN(EH.def.I.N,EH.feat.paths.N),... %Coupling matrix(Ci)
    'Cdi',NaN(EH.feat.dev.N,...
    EH.feat.paths.N,EH.simparam.samples_MPC),... %Coupling matrix for devices  (inputs)
    'Cdo',NaN(EH.feat.dev.N,...
    EH.feat.paths.N,EH.simparam.samples_MPC),... %Coupling matrix for devices  (otputs)
    'c',NaN(EH.def.I.N,EH.simparam.samples_MPC),... %costs matrix (c)
    's',NaN(EH.def.O.N,EH.simparam.samples_MPC)),... %sales matrix (s)
    rep_MPC, 1);

EH.control.MPC.model=struct2table(EH.control.MPC.model,'AsArray',true);
Time=downsample(...
    EH.simparam.data.Time(1:EH.simparam.nm),...
    EH.simparam.tm_MPC);
EH.control.MPC.model.Time=Time(1:rep_MPC);
EH.control.MPC.model=table2timetable(EH.control.MPC.model);

% Matrix size for optimization using MILP on Matlab
EH.control.MPC.solver=repmat(struct(...
    'lb',NaN(EH.feat.nx*EH.simparam.samples_MPC,1),... %lower bounds
    'ub',NaN(EH.feat.nx*EH.simparam.samples_MPC,1),... %upper bounds
    'intcon',NaN(EH.feat.nxint*EH.simparam.samples_MPC,1),... %interger vars
    'ctype',cell(1,1),... %variable type (for CPLEX)
    'fc',NaN(EH.feat.nx*EH.simparam.samples_MPC,1),... %cost func
    'Aeq',NaN(EH.feat.eqconN*EH.simparam.samples_MPC,...
    EH.feat.nx*EH.simparam.samples_MPC),... %eq. const
    'beq',NaN(EH.feat.eqconN*EH.simparam.samples_MPC,1),... %eq. const
    'Aineq',NaN(EH.feat.ineqconN*EH.simparam.samples_MPC,...
    EH.feat.nx*EH.simparam.samples_MPC),... %ineq. const
    'bineq',NaN(EH.feat.ineqconN*EH.simparam.samples_MPC,1),... %ineq. const
    'XOPT',NaN(EH.feat.nx*EH.simparam.samples_MPC,1),... %optim. results
    'OUT',struct([]),... %optim. out
    'FVAL',NaN(1,1),... %optim. fval
    'EX',NaN(1,1),... %optim. exit
    'topt',NaN(1,1)),... %optim. time
    rep_MPC, 1);

EH.control.MPC.solver=struct2table(EH.control.MPC.solver,'AsArray',true);
EH.control.MPC.solver.Time=Time(1:rep_MPC);
EH.control.MPC.solver=table2timetable(EH.control.MPC.solver);

%(M,P,Qch,Qdis,S,int_I,int_M,int_Qch,int_Qdis,int_devices)+dO+I
EH.control.MPC.results=repmat(struct(...
    'M',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Market sales matrix (M)
    'P',NaN(EH.feat.paths.N,EH.simparam.samples_MPC),... %Path matrix (P)
    'Qdis',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Discharge (Qd)
    'Qch',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Charge (Qc)
    'S',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Storage (S)
    'dI',NaN(EH.def.I.N,EH.simparam.samples_MPC),... %Intput matrix (dI)
    'dM',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Market sales matrix (dM)
    'dQch',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Charge (dQc)
    'dQdis',NaN(EH.def.O.N,EH.simparam.samples_MPC),... %Discharge (dQd)
    'dD',NaN(EH.feat.dev.N,EH.simparam.samples_MPC)),... %Devices (dD)
    rep_MPC, 1);

EH.control.MPC.results=struct2table(EH.control.MPC.results,'AsArray',true);
EH.control.MPC.results.Time=Time(1:rep_MPC);
EH.control.MPC.results=table2timetable(EH.control.MPC.results);

%% Data for model simulation

%Preset variables
EH.model.param=repmat(struct('O',NaN(EH.def.O.N,1),... %Output matrix (O)
    'I_max',NaN(EH.def.I.N,1),... %Intput matrix (I)
    'I_min',NaN(EH.def.I.N,1),... %Intput matrix (I)
    'M_max',NaN(EH.def.O.N,1),... %Market sales matrix (M)
    'M_min',NaN(EH.def.O.N,1),... %Market sales matrix (M)
    'Di_max',NaN(EH.feat.dev.N,1),... %Internal matrix (Di)
    'Di_min',NaN(EH.feat.dev.N,1),... %Internal matrix (Di)
    'Do_max',NaN(EH.feat.dev.N,1),... %Internal matrix (Do)
    'Do_min',NaN(EH.feat.dev.N,1),... %Internal matrix (Do)
    'Qdis_min',NaN(EH.def.O.N,1),... %Discharge (Qd)
    'Qdis_max',NaN(EH.def.O.N,1),... %Discharge (Qd)
    'Qch_min',NaN(EH.def.O.N,1),... %Charge (Qc)
    'Qch_max',NaN(EH.def.O.N,1),... %Charge (Qc)
    'S_min',NaN(EH.def.O.N,1),... %Storage (S)
    'S_max',NaN(EH.def.O.N,1),... %Storage (S)
    'Cdis',NaN(EH.def.O.N,1),... %Discharge efficiency (Cdis)
    'Cch',NaN(EH.def.O.N,1),... %Charge efficiency (Cch)
    'Cs',NaN(EH.def.O.N,1),... %Degradation (Cs)
    'C',NaN(EH.def.O.N,EH.feat.paths.N,1),... %Coupling matrix (C)
    'Ci',NaN(EH.def.I.N,EH.feat.paths.N,1),... %Coupling matrix( Ci)
    'Cdi',NaN(EH.feat.dev.N,EH.feat.paths.N,1),... %Coupling matrix for devices (inputs)
    'Cdo',NaN(EH.feat.dev.N,EH.feat.paths.N,1),... %Coupling matrix for devices (outputs)
    'c',NaN(EH.def.I.N,1),... %costs matrix (c)
    's',NaN(EH.def.O.N,1)),... %sales matrix (s)
    EH.simparam.samples, 1);

EH.model.param=struct2table(EH.model.param);
EH.model.param.Time=downsample(EH.simparam.data.Time(1:EH.simparam.nm),...
    EH.simparam.tm);
EH.model.param=table2timetable(EH.model.param);

%Read variables for simulation (nm samples)
%Output matrix (O)
O=outputs_matrix(EH,EH.simparam.data,...
    EH.simparam.dateV(1),...
    EH.simparam.samples,...
    EH.simparam.tm);
EH.model.param.O=cellfun(@ctranspose,O,'UniformOutput',false);

%Inputs limits (I)
[I_max,...
    I_min]=input_lim_matrixes(EH,EH.simparam.data,...
    EH.simparam.dateV(1),...
    EH.simparam.samples,...
    EH.simparam.tm);
EH.model.param.I_max=cellfun(@ctranspose,I_max,'UniformOutput',false);
EH.model.param.I_min=cellfun(@ctranspose,I_min,'UniformOutput',false);

%Devices inputs limits (D)
[Di_max,Di_min,Do_max,Do_min]=...
    dev_lim_matrixes(EH,EH.simparam.data,...
    EH.simparam.dateV(1),...
    EH.simparam.samples,...
    EH.simparam.tm);
EH.model.param.Di_max=cellfun(@ctranspose,Di_max,'UniformOutput',false);
EH.model.param.Di_min=cellfun(@ctranspose,Di_min,'UniformOutput',false);
EH.model.param.Do_max=cellfun(@ctranspose,Do_max,'UniformOutput',false);
EH.model.param.Do_min=cellfun(@ctranspose,Do_min,'UniformOutput',false);

%Market sales limits (M)
[M_max,...
    M_min]=msales_lim_matrixes(EH,EH.simparam.data,...
    EH.simparam.dateV(1),...
    EH.simparam.samples,...
    EH.simparam.tm);
EH.model.param.M_max=cellfun(@ctranspose,M_max,'UniformOutput',false);
EH.model.param.M_min=cellfun(@ctranspose,M_min,'UniformOutput',false);

%Storage limits (S,Qdis,Qch)
[Qdis_min,Qdis_max,...
    Qch_min,Qch_max,...
    S_min,S_max]=...
    storage_lim_matrixes(EH,EH.simparam.data,...
    EH.simparam.dateV(1),...
    EH.simparam.samples,...
    EH.simparam.tm);
EH.model.param.Qdis_max=cellfun(@ctranspose,Qdis_max,'UniformOutput',false);
EH.model.param.Qdis_min=cellfun(@ctranspose,Qdis_min,'UniformOutput',false);
EH.model.param.Qch_max=cellfun(@ctranspose,Qch_max,'UniformOutput',false);
EH.model.param.Qch_min=cellfun(@ctranspose,Qch_min,'UniformOutput',false);
EH.model.param.S_max=cellfun(@ctranspose,S_max,'UniformOutput',false);
EH.model.param.S_min=cellfun(@ctranspose,S_min,'UniformOutput',false);

%conversion factors (C,Ci,Cdis,Cch,Cs,Cdi,Cdo)
[EH.model.param.C,EH.model.param.Ci,...
    Cdis,Cch,Cs,EH.model.param.Cdi,...
    EH.model.param.Cdo]=...
    conversion_factors(EH,EH.simparam.data,...
    EH.simparam.dateV(1),...
    EH.simparam.samples,...
    EH.simparam.tm);
EH.model.param.Cdis=cellfun(@ctranspose,Cdis,'UniformOutput',false);
EH.model.param.Cch=cellfun(@ctranspose,Cch,'UniformOutput',false);
EH.model.param.Cs=cellfun(@ctranspose,Cs,'UniformOutput',false);


%cost/sales path matrixes (c,s)
[c,s]=price(EH,EH.simparam.data,EH.simparam.dateV(1),...
    EH.simparam.samples,...
    EH.simparam.tm);
EH.model.param.c=cellfun(@ctranspose,c,'UniformOutput',false);
EH.model.param.s=cellfun(@ctranspose,s,'UniformOutput',false);

EH.model.data=repmat(struct(...
    'infeas',zeros(1,1),... %Constraints violation (infeasibilities)
    'S0',NaN(EH.def.O.N,1),... %Initial storage matrix (S0)
    'M',NaN(EH.def.O.N,1),... %Market sales matrix (M)
    'P',NaN(EH.feat.paths.N,1),... %Path matrix (P)
    'Qdis',NaN(EH.def.O.N,1),... %Discharge (Qd)
    'Qch',NaN(EH.def.O.N,1),... %Charge (Qc)
    'S',NaN(EH.def.O.N,1),... %Storage (S)
    'dI',NaN(EH.def.I.N,1),... %Intput matrix (dI)
    'dM',NaN(EH.def.O.N,1),... %Market sales matrix (dM)
    'dQch',NaN(EH.def.O.N,1),... %Charge (dQc)
    'dQdis',NaN(EH.def.O.N,1),... %Discharge (dQd)
    'dD',NaN(EH.feat.dev.N,1)),... %Devices (dD)
    EH.simparam.samples, 1);

EH.model.data=struct2table(EH.model.data);
EH.model.data.Time=downsample(EH.simparam.data.Time(1:EH.simparam.nm),...
    EH.simparam.tm);
EH.model.data=table2timetable(EH.model.data);
end

