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

function [Aeq,beq] = eqcon(EH,H,tm,nx,C,Cdo,O,Cdis,Cch,Cs,S0)
%Generates Aeq and beq matrixes for MILP on Matlab using solver
%notation

%simulation and EH parameters, each row is a constraint
i=1; %from old code, now i counter always beggins by 1

% O-P equation for outputs: 0=C*P-Qch+Qdis-M-dO*O
%dO matrix generation as 3 different sub matrixes
dOd=zeros(EH.def.O.N,EH.feat.dev.N);
dOch=zeros(EH.def.O.N,EH.def.O.N);
dOdis=zeros(EH.def.O.N,EH.def.O.N);

for j=1:EH.def.O.N
    if EH.def.O.d(j)>0
        dev=find((EH.feat.dev.dev2(:,1)==EH.def.O.d(j))&...
            (EH.feat.dev.dev2(:,3)==EH.def.O.dO(j)));
        if EH.def.O.dF(j)>0
            dOd(j,min(dev):max(dev))=0;
            paths=find(contains(EH.feat.paths.paths,strcat("D",string(EH.def.O.d(j)))));
            C(j,paths,i)=-EH.def.O.dF(j)*Cdo(dev,paths,i);
        else
            dOd(j,min(dev):max(dev))=-O(j,i);
        end
    end
    if EH.def.O.ch(j)>0
        dOch(j,EH.def.O.ch(j))=-O(j,i);
    end
    if EH.def.O.dis(j)>0
        dOdis(j,EH.def.O.dis(j))=-O(j,i);
    end
end

AOPCD=[-eye(EH.def.O.N)... %M columns
    C(1:EH.def.O.N,:,i)... %P variables, whole C hor. size
    -eye(EH.def.O.N)... %Qch columns
    eye(EH.def.O.N)... %Qdis columns
    zeros(EH.def.O.N,EH.def.O.N)... %S columns
    zeros(EH.def.O.N,EH.def.I.N)... %dI columns
    zeros(EH.def.O.N,EH.def.O.N)... %dM columns
    dOch... %dQc columns
    dOdis... %dQd columns
    dOd...
    zeros(EH.def.O.N,nx-... %Remaining variables for o sample 1
    (EH.def.O.N+EH.feat.paths.N+EH.def.O.N*3+EH.def.I.N+...
    +EH.def.O.N+EH.def.O.N*2+...
    EH.feat.dev.N))... %M+P+Qc+Qd+S+dI+dM+dQc+dQd+int_devices columns
    zeros(EH.def.O.N,nx*(H-1))]; %Remaining variables for samples

if H>1
    % Storage equation S*L=-Qc+Qd+S(k+1)
    AS=[zeros(EH.def.O.N,EH.def.O.N+...
        EH.feat.paths.N)... %M+P columns, 1st sample
        -diag(tm/60*Cch(:,i))...   %Qc columns, 1st sample
        diag(tm/60*Cdis(:,i))...    %Qd columns, 1st sample
        eye(EH.def.O.N)...        %S columns, next sample
        zeros(EH.def.O.N,nx-(EH.def.O.N+...
        EH.feat.paths.N+EH.def.O.N*3))... %remaining variables, 1st sample
        zeros(EH.def.O.N,EH.def.O.N+EH.feat.paths.N+EH.def.O.N*3)...
        zeros(EH.def.O.N,nx-(EH.def.O.N+...
        EH.feat.paths.N+EH.def.O.N*3))...
        zeros(EH.def.O.N,nx*(H-2))];
else
    % Storage equation S*L=-Qc+Qd+S(k+1)
    AS=[zeros(EH.def.O.N,EH.def.O.N+...
        EH.feat.paths.N)... %M+P columns, 1st sample
        -diag(tm/60*Cch(:,i))...   %Qc columns, 1st sample
        diag(tm/60*Cdis(:,i))...    %Qd columns, 1st sample
        eye(EH.def.O.N)...        %S columns, next sample
        zeros(EH.def.O.N,nx-(EH.def.O.N+...
        EH.feat.paths.N+EH.def.O.N*3))... %remaining variables, 1st sample
        ];
end


% Simultaneous outputs binary variables
%(each pair of so constitutes a constraint)
so=EH.feat.dev.constraints_matrixes.so;
Aso=[zeros(EH.feat.dev.Nso,nx-EH.feat.dev.N)... %all columns but int_devices
    so...
    zeros(EH.feat.dev.Nso,nx*(H-1))];

% Simultaneous outputs P variables
%(each pair of so constitutes a constraint)
soP=EH.feat.dev.constraints_matrixes.soP;
AsoP=[zeros(EH.feat.dev.Nso,EH.def.O.N)... %M
    soP...
    zeros(EH.feat.dev.Nso,nx-(EH.def.O.N+...
    EH.feat.paths.N))... %remaining variables, 1st sample
    zeros(EH.feat.dev.Nso,nx*(H-1))];

for k=i:i+H-2
    
    % O-P equation for outputs: 0=C*P-Qch+Qdis-M-dO*O
    %dO matrix generation as 3 different sub matrixes
    dOd=zeros(EH.def.O.N,EH.feat.dev.N);
    dOch=zeros(EH.def.O.N,EH.def.O.N);
    dOdis=zeros(EH.def.O.N,EH.def.O.N);
    
    for j=1:EH.def.O.N
        if EH.def.O.d(j)>0
            dev=find((EH.feat.dev.dev2(:,1)==EH.def.O.d(j))&...
                (EH.feat.dev.dev2(:,3)==EH.def.O.dO(j)));
            if EH.def.O.dF(j)>0
                dOd(j,min(dev):max(dev))=0;
                paths=find(contains(EH.feat.paths.paths,strcat("D",string(EH.def.O.d(j)))));
                C(j,paths,k+1)=-EH.def.O.dF(j)*Cdo(dev,paths,k+1);
            else
                dOd(j,min(dev):max(dev))=-O(j,k+1);
            end
        end
        if EH.def.O.ch(j)>0
            dOch(j,EH.def.O.ch(j))=-O(j,k+1);
        end
        if EH.def.O.dis(j)>0
            dOdis(j,EH.def.O.dis(j))=-O(j,k+1);
        end
    end
    
    AOPCD=[AOPCD; zeros(EH.def.O.N,nx*(k-i+1)),...
        -eye(EH.def.O.N)... %M columns
        C(1:EH.def.O.N,:,k+1)... %P variables, whole C hor. size
        -eye(EH.def.O.N)... %Qch columns
        eye(EH.def.O.N)... %Qdis columns
        zeros(EH.def.O.N,EH.def.O.N)... %S columns
        zeros(EH.def.O.N,EH.def.I.N)... %dI columns
        zeros(EH.def.O.N,EH.def.O.N)... %dM columns
        dOch... %dQc columns
        dOdis... %dQd columns
        dOd...
        zeros(EH.def.O.N,nx-... %Remaining variables for o sample 1
        (EH.def.O.N+EH.feat.paths.N+EH.def.O.N*3+EH.def.I.N+...
        +EH.def.O.N+EH.def.O.N*2+...
        EH.feat.dev.N))... %I+M+P+Qc+Qd+S+dI+dM+dQc+dQd+int_devices columns
        zeros(EH.def.O.N,nx*(H-1-(k-i+1)))]; %Remaining variables for samples
    
    % Storage equation S*L=-Qc+Qd+S(k+1)
    AS=[AS; zeros(EH.def.O.N,nx*(k-i))...
        zeros(EH.def.O.N,EH.def.O.N+EH.feat.paths.N+...
        EH.def.O.N*2)...
        diag(Cs(:,k+1))...
        zeros(EH.def.O.N,nx-(EH.def.O.N+...
        EH.feat.paths.N+EH.def.O.N*3))...
        zeros(EH.def.O.N,EH.def.O.N+EH.feat.paths.N)...
        +diag(tm/60*Cch(:,k+1))...
        -diag(tm/60*Cdis(:,k+1))...
        -eye(EH.def.O.N)...
        zeros(EH.def.O.N,nx-(EH.def.O.N+EH.feat.paths.N+...
        EH.def.O.N*3))...
        zeros(EH.def.O.N,nx*(H-1-(k-i+1)))];
    
    % Simultaneous outputs (each pair of so constitutes a constraint)
    Aso=[Aso; zeros(EH.feat.dev.Nso,nx*(k-i+1))...
        zeros(EH.feat.dev.Nso,nx-EH.feat.dev.N)... %all columns but int_devices
        so...
        zeros(EH.feat.dev.Nso,nx*(H-1-(k-i+1)))];
    
    AsoP=[AsoP; zeros(EH.feat.dev.Nso,nx*(k-i+1))...
        zeros(EH.feat.dev.Nso,EH.def.O.N)... %M
        soP...
        zeros(EH.feat.dev.Nso,nx-(EH.def.O.N+...
        EH.feat.paths.N))... %remaining variables, 1st sample
        zeros(EH.feat.dev.Nso,nx*(H-1-(k-i+1)))];
end
O(EH.def.O.d>0,:)=0;
O(EH.def.O.ch>0,:)=0;
O(EH.def.O.dis>0,:)=0;
Aeq=[AOPCD;AS;Aso;AsoP];
beq=[reshape(O(:,i:i+H-1),...
    [EH.def.O.N*H,1]);...
    S0.*Cs(:,i);...
    zeros(EH.def.O.N*(H-1),1);...
    zeros(EH.feat.dev.Nso*H,1);...
    zeros(EH.feat.dev.Nso*H,1)];
end