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

function [EH] = EH_definition_code(EH)
%Define the technical characteristics of the energy hub from code: number
%of inputs, outputs, conversion devices, storage devices, and either their
%models or static paramteres.


%% Devices (nx: number of device, n: number of inputs/outputs
%         t: type of resource, s: simultaneous inputs/outputs, l:label)

%Device 1: PV PK
EH.def.dev.D(1).n=1;
EH.def.dev.D(1).I.n=1:1;
EH.def.dev.D(1).I.t=["R"];
EH.def.dev.D(1).O.n=1:1;
EH.def.dev.D(1).O.t=["E"];
EH.def.dev.D(1).O.s=[1];
EH.def.dev.D(1).l=["PVpk"];
EH.def.dev.D(1).Dimin={0};
EH.def.dev.D(1).Dimax={'Dimax_pvpk_func'};
EH.def.dev.D(1).Domin={0};
EH.def.dev.D(1).Domax={inf};
EH.def.dev.D(1).nd={'npvpk_func'};

%Device 2: GH H
EH.def.dev.D(2).n=2;
EH.def.dev.D(2).I.n=1:1;
EH.def.dev.D(2).I.t=["P"];
EH.def.dev.D(2).O.n=1:1;
EH.def.dev.D(2).O.t=["H"];
EH.def.dev.D(2).O.s=[1];
EH.def.dev.D(2).l=["PBgh"];
EH.def.dev.D(2).Dimin={0};
EH.def.dev.D(2).Dimax={6.8};
EH.def.dev.D(2).Domin={0};
EH.def.dev.D(2).Domax={inf};
EH.def.dev.D(2).nd={11.54};

%Device 3: GH B
EH.def.dev.D(3).n=3;
EH.def.dev.D(3).I.n=1:1;
EH.def.dev.D(3).I.t=["B"];
EH.def.dev.D(3).O.n=1:2;
EH.def.dev.D(3).O.t=["H";"G"];
EH.def.dev.D(3).O.s=[1 1];
EH.def.dev.D(3).l=["BBgh"];
EH.def.dev.D(3).Dimin={1,1};
EH.def.dev.D(3).Dimax={40,40};
EH.def.dev.D(3).Domin={0,0};
EH.def.dev.D(3).Domax={inf,inf};
EH.def.dev.D(3).nd={4.25,1.76};

%Device 4: IP
EH.def.dev.D(4).n=4;
EH.def.dev.D(4).I.n=1:1;
EH.def.dev.D(4).I.t=["W"];
EH.def.dev.D(4).O.n=1:1;
EH.def.dev.D(4).O.t=["W"];
EH.def.dev.D(4).O.s=[1];
EH.def.dev.D(4).l=["IPgh"];
EH.def.dev.D(4).Dimin={0};
EH.def.dev.D(4).Dimax={inf};
EH.def.dev.D(4).Domin={0};
EH.def.dev.D(4).Domax={5};
EH.def.dev.D(4).nd={1};

%Add more devices here

%% Inputs
%Definition
EH.def.I.t=["E";"R";"P";"B";"W"]; %type of resource
EH.def.I.l=["i";"pk";"i";"i";"i"]; %label
EH.def.I.colour=[...
                 0.6 0.6 0.6;...
                 1 0.8 0;...
                 1 0.6 1;...                 
                 0.4 0.2 0.4;...
                 0.4 1 0.4;...
                 ];
%Model
EH.def.I.cost={'e_cost_func',...               
               0,...
               1.4*1.21,... % https://propanogas.com/faq/precio-propano %1.4 €/kg (21% IVA not included)
               0.255,... % https://www.dropbox.com/s/ptepq3s8hby7jyv/Indice_AVEBIOM_Precio_Pellets%20de%20Madera_Diciembre2017.pdf?dl=0 %0.255 €/kg (IVA included)               
               1.1*0.497};  %http://www.aqualia.com/es/web/aqualia-almeria/atencion-al-cliente/tarifas-de-agua %0.497 €/m3 (Taxes: IVA 10% not included)            
EH.def.I.Imin={0,0,0,0,0,0,0,0};
EH.def.I.Imax={inf,inf,inf,inf,inf,inf,inf,inf};


%% Outputs
%Definition
EH.def.O.t=["E";"H";"G";"W";"E"]; %type of resource
EH.def.O.l=["o";"gh";"o";"o";"ip"]; %label
EH.def.O.d=[zeros(1,2) 0 0 4]; %dependance on active Device
EH.def.O.dO=[zeros(1,2) 0 0 1]; %dependance on active Device output
EH.def.O.ch=[zeros(1,2) 0 0 0]; %dependance on active charge sys.
EH.def.O.dis=[zeros(1,2) 0 0 0]; %dependance on active discharge sys.
EH.def.O.st=[1 ones(1,1) 1 1 0]; %Storage system
EH.def.O.ms=[0 zeros(1,1) 1 0 0]; %Market sales allowed
EH.def.O.n=1:length(EH.def.O.t); %check l and t have the same size
EH.def.O.N=length(EH.def.O.t); %check l and t have the same size
EH.def.O.colour=[...
                0.2 0.2 0.2;...
                0.6 0 0;...
                0.8 0.8 0.4;...
                0 0.4 0;...
                0.2 0.2 0.2;...
                ];
%Model (output block)
EH.def.O.dF=[0 0 0 0 0]; %dependance fixed (0) / porpotional (k)
EH.def.O.O={'Eo','Hgh','Go','Wo',4.5};
EH.def.O.price={0,... 
                0,0,0,0};  
EH.def.O.Mmin={0,0,0,0,0};
EH.def.O.Mmax={0,0,inf,0,0};

%Representation (output block)
EH.def.O.labels=[...
                {{"Electric power (kW)"}} {"Stored energy (kWh)"};...
                {{"Thermal power  (kW)"}} {"Stored energy (kWh)"};...                
                {{"Carbon dioxide (kg/h)"}} {"Stored mass (kg)"};...
                {{"Irrigation water (m^3/h)"}} {"Stored volume (m^3)"};... 
                {{"Pump power (kW)"}} {""};...
                ];
EH.def.O.factor=[ones(1,4) 1]*1.2; %axis representation factor
EH.def.O.ll=zeros(1,9); %lower limts for representation
EH.def.O.ul=[2 16 8 2 5]; %upper limts for representation
EH.def.O.st_factor=[ones(1,8) 1]*1; %axis representation factor
EH.def.O.st_ll=zeros(1,9); %lower limts for representation
EH.def.O.st_ul=[2 16 8 2 5]; %upper limts for representation


%Model (storage block)
EH.def.dev.S(1).l=["Eo"];
EH.def.dev.S(1).Qchmin={0};
EH.def.dev.S(1).Qchmax={3};
EH.def.dev.S(1).nch={0.7};
EH.def.dev.S(1).Qdismin={0};
EH.def.dev.S(1).Qdismax={3};
EH.def.dev.S(1).ndis={0.8};
EH.def.dev.S(1).Smin={0};
EH.def.dev.S(1).Smax={11};
EH.def.dev.S(1).ns={0.98};
EH.def.dev.S(1).S0={0};

EH.def.dev.S(2).l=["Hgh"];
EH.def.dev.S(2).Qchmin={0};
EH.def.dev.S(2).Qchmax={104.5};
EH.def.dev.S(2).nch={0.9};
EH.def.dev.S(2).Qdismin={0};
EH.def.dev.S(2).Qdismax={104.5};
EH.def.dev.S(2).ndis={0.9};
EH.def.dev.S(2).Smin={0};
EH.def.dev.S(2).Smax={116.1};
EH.def.dev.S(2).ns={0.94};
EH.def.dev.S(2).S0={0};

EH.def.dev.S(3).l=["Go"];
EH.def.dev.S(3).Qchmin={0};
EH.def.dev.S(3).Qchmax={51};
EH.def.dev.S(3).nch={1};
EH.def.dev.S(3).Qdismin={0};
EH.def.dev.S(3).Qdismax={51};
EH.def.dev.S(3).ndis={1};
EH.def.dev.S(3).Smin={0};
EH.def.dev.S(3).Smax={25.2};
EH.def.dev.S(3).ns={1};
EH.def.dev.S(3).S0={0};

EH.def.dev.S(4).l=["Wo"];
EH.def.dev.S(4).Qchmin={0};
EH.def.dev.S(4).Qchmax={3};
EH.def.dev.S(4).nch={1};
EH.def.dev.S(4).Qdismin={0};
EH.def.dev.S(4).Qdismax={3};
EH.def.dev.S(4).ndis={1};
EH.def.dev.S(4).Smin={0};
EH.def.dev.S(4).Smax={6};
EH.def.dev.S(4).ns={1};
EH.def.dev.S(4).S0={0};

%% Ddownstream links from labels (b: begining, e: end)
%Inputs 
EH.def.linkslabel.b=["Ei";"Ei";...
        "Rpk";...       
        "Pi";...
        "Bi";...
        "Wi";...
        ];
EH.def.linkslabel.e=["Eo";"Eip";...
        "PVpk";... 
        "PBgh";...
        "BBgh";...
        "IPgh";...
        ];
%Devices
EH.def.linkslabel.b=[EH.def.linkslabel.b;[...
        "PVpk";"PVpk";...
        "PBgh";...
        "BBgh_O1";...
        "BBgh_O2";...        
        "IPgh"]];
    
EH.def.linkslabel.e=[EH.def.linkslabel.e;[...
        "Eo";"Eip";...
        "Hgh";...
        "Hgh";...
        "Go";...
        "Wo"]];
end

