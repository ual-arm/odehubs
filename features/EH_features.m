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

function [EH] = EH_features(EH)
%Add automaticcally features (information required by other functions) of
%the energy hub from the definition.

% Features depending on the number of elements
EH.def.dev.N=length(EH.def.dev.D); 
EH.def.I.n=1:length(EH.def.I.t);
EH.def.I.N=length(EH.def.I.t);
EH.def.O.n=1:length(EH.def.O.t); 
EH.def.O.N=length(EH.def.O.t); 

% Features depending on the devices links and/or outputs
[EH.def.links.b,EH.def.links.e] = rename_links(EH);
[EH.feat.paths.N,EH.feat.paths.paths,EH.feat.paths.inv_paths]=P_paths(EH); 
[EH.feat.dev.N,EH.feat.dev.dev,EH.feat.dev.dev2]=D_count(EH);        

%Determine the number of simultaneous outputs in a single device
EH.feat.dev.Nnso=0; %total groups (N) of non-simultaneus outputs (nso), each group constitute an ineq. constraint
EH.feat.dev.Nso=0;  %total groups (N) of simultaneus paired outputs (so), each group constitute 2 eq. constraints
for i=1:EH.def.dev.N           
     EH.feat.dev.Nnso=EH.feat.dev.Nnso+...
               max(EH.def.dev.D(i).O.s)-1;
     EH.feat.dev.Nso=EH.feat.dev.Nso+...
               max(EH.def.dev.D(i).O.n)-...
               max(EH.def.dev.D(i).O.s);
end

%Determine the number of market sales which share a network with inputs
EH.feat.mssn=0; 
for j=1:EH.def.O.N
     if (EH.def.O.ms(j)==1)&&max(contains(EH.def.I.t,EH.def.O.t(j)))
         EH.feat.mssn=EH.feat.mssn+1;
     end
end

%auxiliary matrix generation for so binary variables
so=zeros(EH.feat.dev.Nso,EH.feat.dev.N);
socount=1;
for j=1:EH.def.dev.N
     if length(EH.def.dev.D(j).O.n)>1
        for j2=2:length(EH.def.dev.D(j).O.n)
         g1=EH.def.dev.D(j).O.s(j2-1);
         g2=EH.def.dev.D(j).O.s(j2);
         if g1==g2
           pos=find((EH.feat.dev.dev2(:,1)==j) &(EH.feat.dev.dev2(:,2)==j2-1));
           so(socount,pos)=1;
           so(socount,pos+1)=-1;
           socount=socount+1;
         end            
        end    
     end
end

EH.feat.dev.constraints_matrixes.so=so;

%auxiliary matrix generation for so P variables
soP=zeros(EH.feat.dev.Nso,EH.feat.paths.N);
aux=EH.feat.dev.dev(not(so==0));
for j=1:EH.feat.dev.Nso
    g1=strfind(EH.feat.paths.paths,aux(1)); %1st group of paths
    g1=not(cellfun(@isempty,g1))*1;
    g2=strfind(EH.feat.paths.paths,aux(2)); %2nd group of paths
    g2=not(cellfun(@isempty,g2))*(-1);    
    soP(j,:)=g1'+g2';
end
EH.feat.dev.constraints_matrixes.soP=soP;

%Non simultaneous sales/demand from grid
%Determine which market sale share a network with which inputs
mssn=zeros(EH.feat.mssn,EH.def.I.N+EH.def.O.N); %dI+dM columns
pos=1;
for j=1:EH.def.O.N
     if (EH.def.O.ms(j)==1)&&max(contains(EH.def.I.t,EH.def.O.t(j)))
         mssn(pos,max((1:EH.def.I.N)'.*...
             contains(EH.def.I.t,EH.def.O.t(j))))=1;  %dI+active dI
         mssn(pos,EH.def.I.N+j)=1;  %dI+active dM
         pos=pos+1;
     end
end
EH.feat.dev.constraints_matrixes.mssn=mssn;

%auxiliary matrix generation for nso
nso=zeros(EH.feat.dev.Nso,EH.feat.dev.N);
nsocount=1;
pos=[];
for j=1:EH.def.dev.N
     if max(EH.def.dev.D(j).O.s)>1
        for j2=1:length(EH.def.dev.D(j).O.s)
           aux=find((EH.feat.dev.dev2(:,1)==j) &(EH.feat.dev.dev2(:,3)==j2)); 
           pos=[pos aux(1)];                    
        end
        nso(nsocount,pos)=1;
        nsocount=nsocount+1;
     end
end
EH.feat.dev.constraints_matrixes.nso=nso;

%Optimization variables for Matlab             
%number of variables in a sample time  
%(M,P,Qc,Qd,S,int_I,int_M,int_Qc,int_Qd,int_devices)                
EH.feat.nx=sum([EH.def.O.N,EH.feat.paths.N,EH.def.O.N,EH.def.O.N,...
                EH.def.O.N,EH.def.I.N,EH.def.O.N,EH.def.O.N,EH.def.O.N,...
                EH.feat.dev.N]);
%(int_I,int_M,int_Qc,int_Qd,int_devices)             
EH.feat.nxint=sum([EH.def.I.N,EH.def.O.N,EH.def.O.N,EH.def.O.N,EH.feat.dev.N]);            

%number of eq. constraints in a sample time  
%(O-P,S,so)                
EH.feat.eqconN=sum([EH.def.O.N,EH.def.O.N,2*EH.feat.dev.Nso]);
%number of ineq. constraints in a sample time  
%(Qc,Qd,int_Qc-int_Qd,I,M,D,nso,mssn)                
EH.feat.ineqconN=sum([4*EH.def.O.N,EH.def.O.N,2*EH.def.I.N,...
                    2*EH.def.O.N,...
                    4*EH.feat.dev.N,EH.feat.dev.Nnso,...
                    EH.feat.mssn]);

end

