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

function [EH] = load_data(EH)
%Load the data required for simulation and check availability
%

%% Check data availability for the simulation length and load them
load DataEH.mat

%%If predictions model are emplyed to get future variables, historical data
%%might be required. By default the begining is set at current day.
if EH.simparam.pred
%previous day (begining)
auxb=find(datenum(dataEH.Time)==datenum(datetime(EH.simparam.date))-...
     days(1));  %change here the number of days if more historical data are required.
else
%current day (begining)
auxb=find(datenum(dataEH.Time)==datenum(EH.simparam.date));    
end

%current day (middle)
auxm=find(datenum(dataEH.Time)==datenum(EH.simparam.date));


if EH.simparam.MPCmode
%following day (end)    
auxe=find(datenum(dataEH.Time)==datenum(datetime(EH.simparam.date)+...
    minutes((EH.simparam.nm+...
    EH.simparam.H_MPC-1))));
else
%current day (end)
auxe=find(datenum(dataEH.Time)==datenum(datetime(EH.simparam.date)+...
    minutes((max(EH.simparam.nm,EH.simparam.H_MPC)-1))));    
end

if (isempty(auxb)==0)&&(isempty(auxe)==0)&&...
        (isregular(dataEH(auxb:auxe,:)))
    EH.simparam.prevdata=dataEH(auxb:auxm-1,:);
    EH.simparam.data=dataEH(auxm:auxe,:);
    EH.simparam.dateV=downsample(EH.simparam.data.Time,EH.simparam.tm);        
else
    error('The date selected is not valid. Not enough data available.')
end
end

