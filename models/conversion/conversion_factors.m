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

function [C,Ci,Cdis,Cch,...
    Cs,Cdi,Cdo] = conversion_factors(EH,data,date,samples,tm)
%Returns the coupling matrixes C and Ci for the Energy Hub
%

%Resample data according to tm
Time=downsample(data.Time,tm);
data=retime(data,Time,'mean');

%Select the suitable time range
TR=timerange(date,date+minutes(samples*tm));
data=data(TR,:);

%preset
C=zeros(EH.def.O.N,EH.feat.paths.N,samples);
Ci=zeros(EH.def.I.N,EH.feat.paths.N);
Cdi=zeros(EH.feat.dev.N,EH.feat.paths.N,samples);
Cdo=zeros(EH.feat.dev.N,EH.feat.paths.N,samples);
Cdis=NaN(EH.def.O.N,samples); %Discharge efficiency (Cdis)
Cch=NaN(EH.def.O.N,samples); %Charge efficiency (Cch)
Cs=NaN(EH.def.O.N,samples); %Degradation (Cs)

%% Coupling matrix Ci for matrixes P-I,
%summation of same input resource which goes to different outputs
if iscell(strfind(EH.feat.paths.paths,"-"))
    Ciaux=cellfun(@min,strfind(EH.feat.paths.paths,"-"));
else
    Ciaux=min(strfind(EH.feat.paths.paths,"-"));
end
Ciaux2=char(EH.feat.paths.paths);

devouts=EH.feat.dev.dev2;


%get groups of simoultaneous outputs for each device
%considering only the first output of each group to determinate the input 
%flow the paths containing the remaining outputs of that group must have 0
%coefficients in Ci matrix, so we discard  the first output of each group
[~,auxg]=unique(strcat("D",string(devouts(:,1)),...
                "G",string(devouts(:,3))),'row');
devouts=devouts(not(ismember(1:size(devouts,1),auxg)),:);
devouts=strcat("D",string(devouts(:,1)),"O",string(devouts(:,2)));

%discard paths containing labels in 'devouts'
discard=ones(length(Ciaux),1);
for i=1:length(devouts)
    discard=discard.*not(contains(EH.feat.paths.paths,devouts(i)));
end

%set up auxiliary vector
for i=1:length(Ciaux)
    Ciaux3(i)=str2num(Ciaux2(i,2:Ciaux(i)-1));
end

%matrix Ci building
for i=1:length(Ciaux3)
if (Ciaux3(i)>0)
   Ci(Ciaux3(i),i)=1*discard(i);
end  
end 

Ci=repmat(Ci,1,1,samples);

%% Conversion factors definition (devices)
%Create one variable from either coeficients or functions containing the
%efficiencies vector for the 'samples' horizon
for i=1:EH.def.dev.N    
    if isvarname(EH.def.dev.D(i).l)
        if EH.def.dev.D(i).O.n==1
            if  ischar(EH.def.dev.D(i).nd{:})
                if ismember(EH.def.dev.D(i).nd{:},data.Properties.VariableNames)                    
                    right_term=['transpose(data.',num2str(EH.def.dev.D(i).nd{:}),');'];                    
                else
                    right_term=[num2str(EH.def.dev.D(i).nd{:}),...
                        '(data,date,samples,tm);'];
                end
            elseif  isnumeric(EH.def.dev.D(i).nd{:})
                right_term=[num2str(EH.def.dev.D(i).nd{:}),...
                    '*ones(1,samples);'];
            else
                error('Invalid n efciciency for device %d: %s',i,EH.def.dev.D(i).l);
            end
            eval(strcat('n',EH.def.dev.D(i).l,'=',right_term));
        else           
            for j=1:max(EH.def.dev.D(i).O.n)
                if  ischar(EH.def.dev.D(i).nd{j})
                    if ismember(EH.def.dev.D(i).nd{j},data.Properties.VariableNames)
                        right_term=['transpose(data.',num2str(EH.def.dev.D(i).nd{j}),');'];
                    else
                        right_term=[num2str(EH.def.dev.D(i).nd{j}),...
                            '(data,date,samples,tm);'];
                    end
                elseif  isnumeric(EH.def.dev.D(i).nd{j})
                    right_term=[num2str(EH.def.dev.D(i).nd{j}),...
                        '*ones(1,samples);'];
                else
                    error('Invalid n efciciency for device %d: %s',i,EH.def.dev.D(i).l);
                end
                eval(strcat('n',EH.def.dev.D(i).l,'_O',...
                            num2str(j),'=',right_term));
            end
        end        
    else
        error('Invalid name or label for device %d',i);
    end
end

%% Coupling matrix C, for matrixes P-O
%Generate matrix C
for i=1:samples
   for j=1:EH.feat.paths.N
    pathdashes=strfind(EH.feat.paths.paths(j),"-"); %determine number of devices
    devices=strings(1,length(pathdashes)+1); %empty array to store devices
    pathdashes=[0 pathdashes length(char(EH.feat.paths.paths(j)))+1]; %include I-O
    for k=1:length(pathdashes)-1          %store devices
       aux=char(EH.feat.paths.paths(j));
       devices(k)=string(aux(pathdashes(k)+1:pathdashes(k+1)-1)); %devices, including I-O
    end
    if length(pathdashes)==3              %case A, no devices between I-O
       output=char(devices(2));
       C(str2num(output(2:length(output))),j,i)=1;               
    elseif length(pathdashes)>3           %case B, devices between I-O
       n=1;                         %conversion factor initial value
       output=char(devices(length(devices))); %output node
       for l=2:length(devices)-1    %production loop
          dev=char(devices(l));           %device to determine conversion factor
          %determine if the device has more than 1 output
          Noutdev=strfind(dev,"O");
          if not(isempty(Noutdev))==1
              outdev=strcat("_",dev(Noutdev:length(dev)));
              dev=dev(1:Noutdev-1);
          else
              outdev="";
          end
          %determine conversion factor of device l    
          ndev=eval(strcat("n",...
              EH.def.dev.D(str2double(dev(2:length(dev)))).l,outdev));       
          n=n*ndev(i);                 %conversion factor as production  
       end
       C(str2num(output(2:length(output))),j,i)=n;
    else                             %case C, error owing to size of Ndev
      error("Wrong I-O path");
    end
   end
end


%% Coupling matrix Cdi, for matrixes P-D
%Generate matrix Cdi for ineq. constraints, almost same process as C
for i=1:samples
   for j=1:EH.feat.paths.N
    pathdashes=strfind(EH.feat.paths.paths(j),"-"); %determine number of devices
    devices=strings(1,length(pathdashes)+1); %empty array to store devices
    pathdashes=[0 pathdashes length(char(EH.feat.paths.paths(j)))+1]; %include I-O
    for k=1:length(pathdashes)-1          %store devices
       aux=char(EH.feat.paths.paths(j));
       devices(k)=string(aux(pathdashes(k)+1:pathdashes(k+1)-1)); %devices, including I-O
    end    
    if length(pathdashes)==3              %case A, no devices between I-O
       %no efect on Cdev         
    elseif length(pathdashes)>3           %case B, devices between I-O
       n=1;                         %conversion factor initial value
       for l=2:length(devices)-1    %production loop
          dev=char(devices(l));     %device to determine conversion factor
          %determine if the device has more than 1 output
          Noutdev=strfind(dev,"O");
          if not(isempty(Noutdev))==1
              outdev=strcat("_",dev(Noutdev:length(dev)));
              dev=dev(1:Noutdev-1);
          else
              outdev="";
          end
          %determine conversion factor of device l          
          ndev=eval(strcat("n",...
              EH.def.dev.D(str2double(dev(2:length(dev)))).l,outdev));   
          n=n*ndev(i);                 %conversion factor as production  
          devnumber=find(EH.feat.dev.dev==...
              strcat(dev,strrep(outdev,"_","")));
          Cdi(devnumber,j,i)=n;
          %Comment for outputs contraints on devices
          if ndev(i)~=0 
          Cdi(devnumber,j,i)=Cdi(devnumber,j,i)/ndev(i); 
          else
          Cdi(devnumber,j,i)=0;     
          end
          %end comment
       end       
    else                             %case C, error owing to size of Ndev
      error("Wrong I-O path");
    end
   end
end

%% Coupling matrix Cdo, for matrixes P-D
%Generate matrix Cdo for ineq. constraints, almost same process as C
for i=1:samples
   for j=1:EH.feat.paths.N
    pathdashes=strfind(EH.feat.paths.paths(j),"-"); %determine number of devices
    devices=strings(1,length(pathdashes)+1); %empty array to store devices
    pathdashes=[0 pathdashes length(char(EH.feat.paths.paths(j)))+1]; %include I-O
    for k=1:length(pathdashes)-1          %store devices
       aux=char(EH.feat.paths.paths(j));
       devices(k)=string(aux(pathdashes(k)+1:pathdashes(k+1)-1)); %devices, including I-O
    end    
    if length(pathdashes)==3              %case A, no devices between I-O
       %no efect on Cdev         
    elseif length(pathdashes)>3           %case B, devices between I-O
       n=1;                         %conversion factor initial value
       for l=2:length(devices)-1    %production loop
          dev=char(devices(l));     %device to determine conversion factor
          %determine if the device has more than 1 output
          Noutdev=strfind(dev,"O");
          if not(isempty(Noutdev))==1
              outdev=strcat("_",dev(Noutdev:length(dev)));
              dev=dev(1:Noutdev-1);
          else
              outdev="";
          end
          %determine conversion factor of device l          
          ndev=eval(strcat("n",... 
              EH.def.dev.D(str2double(dev(2:length(dev)))).l,outdev));   
          n=n*ndev(i);                 %conversion factor as production  
          devnumber=find(EH.feat.dev.dev==...
              strcat(dev,strrep(outdev,"_","")));
          Cdo(devnumber,j,i)=n;
       end       
    else                             %case C, error owing to size of Ndev
     error("Wrong I-O path");
    end
   end
end


%% Conversion factors definition (storage)
%Create one variable from either coeficients or functions containing the
%efficiencies vector for the 'samples' horizon
for i=1:length(EH.def.dev.S)
    if isvarname(EH.def.dev.S(i).l)
        if  ischar(EH.def.dev.S(i).nch{:}) %charge
            if ismember(EH.def.dev.S(i).nch{:},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.dev.S(i).nch{:}),');'];
            else
                right_term=[num2str(EH.def.dev.S(i).nch{:}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.dev.S(i).nch{:})
            right_term=[num2str(EH.def.dev.S(i).nch{:}),...
                '*ones(1,samples);'];
        else
            error('Invalid n charge efciciency for device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_c=',right_term));
        
        if  ischar(EH.def.dev.S(i).ndis{:}) %discharge
            if ismember(EH.def.dev.S(i).ndis{:},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.dev.S(i).ndis{:}),');'];
            else
                right_term=[num2str(EH.def.dev.S(i).ndis{:}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.dev.S(i).ndis{:})
            right_term=[num2str(EH.def.dev.S(i).ndis{:}),...
                '*ones(1,samples);'];
        else
            error('Invalid n discharge efciciency for device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_d=1./',right_term)); %the value is inverted
        
        if  ischar(EH.def.dev.S(i).ns{:}) %degradation
            if ismember(EH.def.dev.S(i).ns{:},data.Properties.VariableNames)
                right_term=['transpose(data.',num2str(EH.def.dev.S(i).ns{:}),');'];
            else
                right_term=[num2str(EH.def.dev.S(i).ns{:}),...
                    '(data,date,samples,tm);'];
            end
        elseif  isnumeric(EH.def.dev.S(i).ns{:})
            right_term=[num2str(EH.def.dev.S(i).ns{:}),...
                '*ones(1,samples);'];
        else
            error('Invalid n degradation efciciency for device %d: %s',i,EH.def.dev.S(i).l);
        end
        eval(strcat(EH.def.dev.S(i).l,'_s=',right_term));
        
    else
        error('Invalid name or label for device %d',i)
    end
end

%Inexistent storage systems
for i=1:EH.def.O.N
    if EH.def.O.st(i)==0
        eval(strcat(EH.def.O.t(i),EH.def.O.l(i),'_c=ones(samples,1);'))
        eval(strcat(EH.def.O.t(i),EH.def.O.l(i),'_d=ones(samples,1);'))
        eval(strcat(EH.def.O.t(i),EH.def.O.l(i),'_s=ones(samples,1);'))
    end
end

%% Build matrixes
for i=1:EH.def.O.N  
if EH.def.O.st(i)>0 %Storage system
Cdis(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_d")));
Cch(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_c")));
Cs(i,:)=eval(char(strcat(EH.def.O.t{i},EH.def.O.l{i},"_s")));
else
Cdis(i,:)=1;
Cch(i,:)=1;
Cs(i,:)=0.01;
end
end

Cdis=mat2cell(Cdis',ones(samples,1),EH.def.O.N);
Cch=mat2cell(Cch',ones(samples,1),EH.def.O.N);
Cs=mat2cell(Cs',ones(samples,1),EH.def.O.N);
C=reshape(mat2cell(C,EH.def.O.N,EH.feat.paths.N,ones(samples,1)),...
                   [samples,1]);
Ci=reshape(mat2cell(Ci,EH.def.I.N,EH.feat.paths.N,ones(samples,1)),...
                   [samples,1]);
Cdi=reshape(mat2cell(Cdi,EH.feat.dev.N,EH.feat.paths.N,ones(samples,1)),...
                   [samples,1]);
Cdo=reshape(mat2cell(Cdo,EH.feat.dev.N,EH.feat.paths.N,ones(samples,1)),...
                   [samples,1]);    
end
