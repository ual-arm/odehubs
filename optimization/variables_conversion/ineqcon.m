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

function [Aineq,bineq] = ineqcon(EH,H,tm,nx,Qc_max,Qc_min,Qd_max,...
    Qd_min,I_max,I_min,M_max,M_min,Di_max,Di_min,Do_max,Do_min,Ci,Cdi,Cdo)
%Generates Aineq and bineq matrixes for MILP on Matlab using solver
%notation

i=1; %from old code, now i counter always begins by 1

%storage limits
AQmax=[zeros(EH.def.O.N*2,EH.def.O.N+EH.feat.paths.N)... %M+A columns
    eye(EH.def.O.N*2)... %Qd, Qc columns
    zeros(EH.def.O.N*2,EH.def.O.N+EH.def.I.N+EH.def.O.N)... %S+dI+dM columns
    -[diag(Qc_max(:,i)),zeros(EH.def.O.N);...
    zeros(EH.def.O.N),diag(Qd_max(:,i))]...     dQc+dQd columns
    zeros(EH.def.O.N*2,nx-(EH.def.O.N+EH.feat.paths.N+...
    EH.def.O.N*2+... %M+A+Qc+Qd+S+dI+dM+dQc+dQd columns
    EH.def.O.N+EH.def.I.N+EH.def.O.N+EH.def.O.N*2))...
    zeros(EH.def.O.N*2,nx*(H-1))];
AQmin=[zeros(EH.def.O.N*2,EH.def.O.N+EH.feat.paths.N)... %M+P columns
    -eye(EH.def.O.N*2)... %Qd, Qc columns
    zeros(EH.def.O.N*2,EH.def.O.N+EH.def.I.N+EH.def.O.N)... %S+dI+dM columns
    [diag(Qc_min(:,i)),zeros(EH.def.O.N);...
    zeros(EH.def.O.N),diag(Qd_min(:,i))]...     dQc+dQd columns
    zeros(EH.def.O.N*2,nx-(EH.def.O.N+EH.feat.paths.N+...
    EH.def.O.N*2+EH.def.O.N+...
    EH.def.I.N+EH.def.O.N+EH.def.O.N*2))... %M+P+Qc+Qd+S+dI+dM+dQc+dQd columns
    zeros(EH.def.O.N*2,nx*(H-1))];

%non simultaneous charge-discharge
ACD=[zeros(EH.def.O.N,EH.def.O.N+EH.feat.paths.N+EH.def.O.N*2+...
    EH.def.O.N+EH.def.I.N+EH.def.O.N)... %M+P+Qc+Qd+S+dI+dM columns
    eye(EH.def.O.N) eye(EH.def.O.N)... %dQc+dQd columns
    zeros(EH.def.O.N,nx-(EH.def.O.N+EH.feat.paths.N+EH.def.O.N*2+...
    EH.def.O.N+EH.def.I.N+EH.def.O.N+EH.def.O.N*2))...
    zeros(EH.def.O.N,nx*(H-1))];

%input limits
AImax=[zeros(EH.def.I.N,EH.def.O.N)... %M columns
    Ci(:,:,i)... %P columns
    zeros(EH.def.I.N,EH.def.O.N*3)... %Qd+Qc+S columns
    -diag(I_max(:,i))... %dI columns
    zeros(EH.def.I.N,nx-...
    (EH.def.O.N+EH.feat.paths.N+EH.def.O.N*3+EH.def.I.N))...
    zeros(EH.def.I.N,nx*(H-1))];
AImin=[zeros(EH.def.I.N,EH.def.O.N)... %M columns
    -Ci(:,:,i)... %P columns
    zeros(EH.def.I.N,EH.def.O.N*3)... %Qd+Qc+S columns
    diag(I_min(:,i))... %dI columns
    zeros(EH.def.I.N,nx-...
    (EH.def.O.N+EH.feat.paths.N+EH.def.O.N*3+EH.def.I.N))...
    zeros(EH.def.I.N,nx*(H-1))];

%devices limits
ADimax=[ zeros(EH.feat.dev.N,EH.def.O.N)... %M columns
    Cdi(:,:,i)... %P columns
    zeros(EH.feat.dev.N,EH.def.O.N*3)... %Qc+Qd+S
    zeros(EH.feat.dev.N,EH.def.I.N+EH.def.O.N+EH.def.O.N*2)...%dI+dM+dQc+dQd
    -diag(Di_max(:,i))...
    zeros(EH.feat.dev.N,nx*(H-1))];
ADimin=[ zeros(EH.feat.dev.N,EH.def.O.N)... %M columns
    -Cdi(:,:,i)... %P columns
    zeros(EH.feat.dev.N,EH.def.O.N*3)... %Qc+Qd+S
    zeros(EH.feat.dev.N,EH.def.I.N+EH.def.O.N+EH.def.O.N*2)...%dI+dM+dQc+dQd
    diag(Di_min(:,i))...
    zeros(EH.feat.dev.N,nx*(H-1))];
ADomax=[ zeros(EH.feat.dev.N,EH.def.O.N)... %M columns
    Cdo(:,:,i)... %P columns
    zeros(EH.feat.dev.N,EH.def.O.N*3)... %Qc+Qd+S
    zeros(EH.feat.dev.N,EH.def.I.N+EH.def.O.N+EH.def.O.N*2)...%dI+dM+dQc+dQd
    -diag(Do_max(:,i))...
    zeros(EH.feat.dev.N,nx*(H-1))];
ADomin=[ zeros(EH.feat.dev.N,EH.def.O.N)... %M columns
    -Cdo(:,:,i)... %P columns
    zeros(EH.feat.dev.N,EH.def.O.N*3)... %Qc+Qd+S
    zeros(EH.feat.dev.N,EH.def.I.N+EH.def.O.N+EH.def.O.N*2)...%dI+dM+dQc+dQd
    diag(Do_min(:,i))...
    zeros(EH.feat.dev.N,nx*(H-1))];

nso=EH.feat.dev.constraints_matrixes.nso;
%non-simultaneous outputs
Anso=[zeros(EH.feat.dev.Nnso,nx-EH.feat.dev.N)... %all columns but int_devices
    nso...
    zeros(EH.feat.dev.Nnso,nx*(H-1))];

%Market sales limits
AMmax=[ eye(EH.def.O.N)... %M columns
    zeros(EH.def.O.N,EH.feat.paths.N)... %A columns
    zeros(EH.def.O.N,EH.def.O.N*3)... %Qc+Qd+S
    zeros(EH.def.O.N,EH.def.I.N)... %dI
    -diag(M_max(:,i))... %dM columns
    zeros(EH.def.O.N,nx-(EH.def.O.N+EH.feat.paths.N+...
    EH.def.O.N*2+EH.def.O.N+...
    EH.def.I.N+EH.def.O.N))... %I+M+A+Qc+Qd+S+dI+dM columns
    zeros(EH.def.O.N,nx*(H-1))];
AMmin=[ -eye(EH.def.O.N)... %M columns
    zeros(EH.def.O.N,EH.feat.paths.N)... %A columns
    zeros(EH.def.O.N,EH.def.O.N*3)... %Qc+Qd+S
    zeros(EH.def.O.N,EH.def.I.N)... %dI
    diag(M_min(:,i))... %dM columns
    zeros(EH.def.O.N,nx-(EH.def.O.N+EH.feat.paths.N+...
    EH.def.O.N*2+EH.def.O.N+...
    EH.def.I.N+EH.def.O.N))... %I+M+A+Qc+Qd+S+dI+dM columns
    zeros(EH.def.O.N,nx*(H-1))];

mssn=EH.feat.dev.constraints_matrixes.mssn;
%non-simultaneous sales/demand
Amssn=[zeros(EH.feat.mssn,EH.def.O.N+EH.feat.paths.N+...
    EH.def.O.N*2+EH.def.O.N)... %M+A+Qc+Qd+S columns
    mssn...                                          %dI+dM columns
    zeros(EH.feat.mssn,nx-(EH.def.O.N+EH.feat.paths.N+...
    EH.def.O.N*2+EH.def.O.N+...
    EH.def.I.N+EH.def.O.N))... %M+A+Qc+Qd+S+dI+dM columns
    zeros(EH.feat.mssn,nx*(H-1))];


for k=i:i+H-2
    %storage limits
    AQmax=[AQmax; zeros(EH.def.O.N*2,nx*(k-i+1))...
        zeros(EH.def.O.N*2,EH.def.O.N+EH.feat.paths.N)... %M+A columns
        eye(EH.def.O.N*2)... %Qd, Qc columns
        zeros(EH.def.O.N*2,EH.def.O.N+EH.def.I.N+EH.def.O.N)... %S+dI+dM columns
        -[diag(Qc_max(:,k+1)),zeros(EH.def.O.N);...
        zeros(EH.def.O.N),diag(Qd_max(:,k+1))]... %dQc+dQd columns
        zeros(EH.def.O.N*2,nx-(EH.def.O.N+EH.feat.paths.N+... %M+A columns
        EH.def.O.N*2+EH.def.O.N+... %Qc,Qd,S
        EH.def.I.N+EH.def.O.N+EH.def.O.N*2))... %dI+dM+dQc+dQd
        zeros(EH.def.O.N*2,nx*(H-1-(k-i+1)))];
    
    AQmin=[AQmin; zeros(EH.def.O.N*2,nx*(k-i+1))...
        zeros(EH.def.O.N*2,EH.def.O.N+EH.feat.paths.N)... %M+A columns
        -eye(EH.def.O.N*2)... %Qd, Qc columns
        zeros(EH.def.O.N*2,EH.def.O.N+EH.def.I.N+EH.def.O.N)... %S+dI+dM columns
        [diag(Qc_min(:,k+1)),zeros(EH.def.O.N);...
        zeros(EH.def.O.N),diag(Qd_min(:,k+1))]... %dQc+dQd columns
        zeros(EH.def.O.N*2,nx-(EH.def.O.N+EH.feat.paths.N+... %M+A columns
        EH.def.O.N*2+EH.def.O.N+...  %Qc,Qd,S
        EH.def.I.N+EH.def.O.N+EH.def.O.N*2))... %dI+dM+dQc+dQd
        zeros(EH.def.O.N*2,nx*(H-1-(k-i+1)))];
    
    %non simultaneous charge-discharge
    ACD=[ACD; zeros(EH.def.O.N,nx*(k-i+1))...
        zeros(EH.def.O.N,EH.def.O.N+EH.feat.paths.N+EH.def.O.N*2+...
        EH.def.O.N+EH.def.I.N+EH.def.O.N)... %M+A+Qc+Qd+S+dI+dM columns
        eye(EH.def.O.N) eye(EH.def.O.N)... %dQc+dQd columns
        zeros(EH.def.O.N,nx-(EH.def.O.N+EH.feat.paths.N+EH.def.O.N*2+...
        EH.def.O.N+EH.def.I.N+EH.def.O.N+EH.def.O.N*2))...
        zeros(EH.def.O.N,nx*(H-1-(k-i+1)))];
    
    %input limits
    AImax=[AImax; zeros(EH.def.I.N,nx*(k-i+1))...
        zeros(EH.def.I.N,EH.def.O.N)... %M columns
        Ci(:,:,k+1)... %P columns
        zeros(EH.def.I.N,EH.def.O.N*3)... %Qd+Qc+S columns
        -diag(I_max(:,k+1))... %dI
        zeros(EH.def.I.N,nx-...
        (EH.def.O.N+EH.feat.paths.N+EH.def.O.N*3+EH.def.I.N))...
        zeros(EH.def.I.N,nx*(H-1-(k-i+1)))];
    AImin=[AImin; zeros(EH.def.I.N,nx*(k-i+1))...
        zeros(EH.def.I.N,EH.def.O.N)... %M columns
        -Ci(:,:,k+1)... %P columns
        zeros(EH.def.I.N,EH.def.O.N*3)... %Qd+Qc+S columns
        diag(I_min(:,k+1))...
        zeros(EH.def.I.N,nx-...
        (EH.def.O.N+EH.feat.paths.N+EH.def.O.N*3+EH.def.I.N))...
        zeros(EH.def.I.N,nx*(H-1-(k-i+1)))];
    
    %devices limits
    ADimax=[ADimax; zeros(EH.feat.dev.N,nx*(k-i+1))...
        zeros(EH.feat.dev.N,EH.def.O.N)... %M columns
        Cdi(:,:,k+1)... %P columns
        zeros(EH.feat.dev.N,EH.def.O.N*3)... %Qc+Qd+S
        zeros(EH.feat.dev.N,EH.def.I.N+EH.def.O.N+...
        EH.def.O.N*2)...%dI+dM+dQc+dQd
        -diag(Di_max(:,k+1))...
        zeros(EH.feat.dev.N,nx*(H-1-(k-i+1)))];
    ADimin=[ADimin; zeros(EH.feat.dev.N,nx*(k-i+1))...
        zeros(EH.feat.dev.N,EH.def.O.N)... %M columns
        -Cdi(:,:,k+1)... %P columns
        zeros(EH.feat.dev.N,EH.def.O.N*3)... %Qc+Qd+S
        zeros(EH.feat.dev.N,EH.def.I.N+EH.def.O.N+...
        EH.def.O.N*2)...%dI+dM+dQc+dQd
        diag(Di_min(:,k+1))...
        zeros(EH.feat.dev.N,nx*(H-1-(k-i+1)))];
    ADomax=[ADomax; zeros(EH.feat.dev.N,nx*(k-i+1))...
        zeros(EH.feat.dev.N,EH.def.O.N)... %M columns
        Cdo(:,:,k+1)... %P columns
        zeros(EH.feat.dev.N,EH.def.O.N*3)... %Qc+Qd+S
        zeros(EH.feat.dev.N,EH.def.I.N+EH.def.O.N+...
        EH.def.O.N*2)...%dI+dM+dQc+dQd
        -diag(Do_max(:,k+1))...
        zeros(EH.feat.dev.N,nx*(H-1-(k-i+1)))];
    ADomin=[ADomin; zeros(EH.feat.dev.N,nx*(k-i+1))...
        zeros(EH.feat.dev.N,EH.def.O.N)... %M columns
        -Cdo(:,:,k+1)... %P columns
        zeros(EH.feat.dev.N,EH.def.O.N*3)... %Qc+Qd+S
        zeros(EH.feat.dev.N,EH.def.I.N+EH.def.O.N+...
        EH.def.O.N*2)...%dI+dM+dQc+dQd
        diag(Do_min(:,k+1))...
        zeros(EH.feat.dev.N,nx*(H-1-(k-i+1)))];
           
     %non-simultaneous outputs
    if EH.feat.dev.Nnso>=1
    Anso=[Anso; zeros(EH.feat.dev.Nnso,nx*(k-i+1))...
        zeros(EH.feat.dev.Nnso,nx-EH.feat.dev.N)... %all columns but int_devices
        nso...
        zeros(EH.feat.dev.Nnso,nx*(H-1-(k-i+1)))];      
    else
    Anso=[];
    end
    
    %Market sales limits
    AMmax=[AMmax;  zeros(EH.def.O.N,nx*(k-i+1))...
        eye(EH.def.O.N)... %M columns
        zeros(EH.def.O.N,EH.feat.paths.N)... %P columns
        zeros(EH.def.O.N,EH.def.O.N*3)... %Qc+Qd+S
        zeros(EH.def.O.N,EH.def.I.N)... %dI
        -diag(M_max(:,i))... %dM columns
        zeros(EH.def.O.N,nx-(EH.def.O.N+EH.feat.paths.N+...
        EH.def.O.N*2+EH.def.O.N+...
        EH.def.I.N+EH.def.O.N))... %M+A+Qc+Qd+S+dI+dM columns
        zeros(EH.def.O.N,nx*(H-1-(k-i+1)))];
    AMmin=[AMmin;  zeros(EH.def.O.N,nx*(k-i+1))...
        -eye(EH.def.O.N)... %M columns
        zeros(EH.def.O.N,EH.feat.paths.N)... %P columns
        zeros(EH.def.O.N,EH.def.O.N*3)... %Qc+Qd+S
        zeros(EH.def.O.N,EH.def.I.N)... %dI
        diag(M_min(:,i))... %dM columns
        zeros(EH.def.O.N,nx-(EH.def.O.N+EH.feat.paths.N+...
        EH.def.O.N*2+EH.def.O.N+...
        EH.def.I.N+EH.def.O.N))... %M+A+Qc+Qd+S+dI+dM columns
        zeros(EH.def.O.N,nx*(H-1-(k-i+1)))];
    
    %Non simultaneous sales/demand from grid
    Amssn=[Amssn; zeros(EH.feat.mssn,nx*(k-i+1))...
        zeros(EH.feat.mssn,EH.def.O.N+EH.feat.paths.N+...
        EH.def.O.N*2+EH.def.O.N)... %M+A+Qc+Qd+S columns
        mssn...                                          %dI+dM columns
        zeros(EH.feat.mssn,nx-(EH.def.O.N+EH.feat.paths.N+...
        EH.def.O.N*2+EH.def.O.N+...
        EH.def.I.N+EH.def.O.N))... %M+A+Qc+Qd+S+dI+dM columns
        zeros(EH.feat.mssn,nx*(H-1-(k-i+1)))];
end
Aineq=[AQmax;AQmin;ACD;...
    AImax;AImin;...
    ADimax;ADimin;ADomax;ADomin;...
    Anso;AMmax;AMmin;Amssn];
bineq=[zeros(EH.def.O.N*2*H*2,1);... %AQmax;AQmin;
    ones(EH.def.O.N*H,1);... %ACD;
    zeros(EH.def.I.N*H,1);... %AImax;
    zeros(EH.def.I.N*H,1);... %AImin;
    zeros(EH.feat.dev.N*2*H,1);... %ADimax;ADimin;
    zeros(EH.feat.dev.N*2*H,1);... %ADomax;ADomin;
    ones(EH.feat.dev.Nnso*H,1);... %Anso;
    zeros(EH.def.O.N*H,1);... %AMmax;
    zeros(EH.def.O.N*H,1);... %AMmin;
    ones(EH.feat.mssn*H,1)]; %Amssn
end