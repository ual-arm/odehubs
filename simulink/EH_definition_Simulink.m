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

function [EH] = EH_definition_Simulink(EH)
%Read from Simulink the technical characteristics of the energy hub: number
%of inputs, outputs, conversion devices, storage devices, and either their
%models or static paramteres.



%% Set Simulink numeration
%Get the subsys blocks of the file
blocks=find_system(EH.def.definition,...
    'LookUnderMasks','off',...
    'FollowLinks','on','BlockType','SubSystem');

%Filter ODEHubs components
blocks=blocks(cellfun(@(x) isfield(x,'type'),...
    get_param(blocks,'UserData'))); %get blocks with field 'type' in UD
CN=blocks(cellfun(@(x) isequal(x.type,'CN'),...
    get_param(blocks,'UserData'))); %get convergence blocks
DN=blocks(cellfun(@(x) isequal(x.type,'DN'),...
    get_param(blocks,'UserData'))); %get divergence blocks
S=blocks(cellfun(@(x) isequal(x.type,'S'),...
    get_param(blocks,'UserData'))); %get storage blocks
D=blocks(cellfun(@(x) isequal(x.type,'D'),...
    get_param(blocks,'UserData'))); %get device blocks
I=blocks(cellfun(@(x) isequal(x.type,'I'),...
    get_param(blocks,'UserData'))); %get input blocks
O=blocks(cellfun(@(x) isequal(x.type,'O'),...
    get_param(blocks,'UserData'))); %get output blocks

%Set input numbers
n=get_param(I,'n');
[~,order]=sort(cellfun(@str2double,n)); %remove zeros and order numbers
numbers=num2cell(1:length(n));  %remove zeros and order numbers
n(order)=cellfun(@num2str,numbers,'UniformOutput',false);  %remove zeros and order numbers
for i=1:length(n)
    set_param(I{i},'n',n{i});
end

%Set output numbers
n=get_param(O,'n');
[~,order]=sort(cellfun(@str2double,n)); %remove zeros and order numbers
numbers=num2cell(1:length(n));  %remove zeros and order numbers
n(order)=cellfun(@num2str,numbers,'UniformOutput',false);  %remove zeros and order numbers
for i=1:length(n)
    set_param(O{i},'n',n{i});
end

%Set device numbers
n=get_param(D,'n');
[~,order]=sort(cellfun(@str2double,n)); %remove zeros and order numbers
numbers=num2cell(1:length(n));  %remove zeros and order numbers
n(order)=cellfun(@num2str,numbers,'UniformOutput',false);  %remove zeros and order numbers
for i=1:length(n)
    set_param(D{i},'n',n{i});
end

%Set DN numbers
n=get_param(DN,'n');
[~,order]=sort(cellfun(@str2double,n)); %remove zeros and order numbers
numbers=num2cell(1:length(n));  %remove zeros and order numbers
n(order)=cellfun(@num2str,numbers,'UniformOutput',false);  %remove zeros and order numbers
for i=1:length(n)
    set_param(DN{i},'n',n{i});
end

%Set CN numbers
n=get_param(CN,'n');
[~,order]=sort(cellfun(@str2double,n)); %remove zeros and order numbers
numbers=num2cell(1:length(n));  %remove zeros and order numbers
n(order)=cellfun(@num2str,numbers,'UniformOutput',false);  %remove zeros and order numbers
for i=1:length(n)
    set_param(CN{i},'n',n{i});
end

%Set storage numbers and labels
for i=1:length(S)
    %get port handles
    ph=get_param(S{i},'PortHandles');
    %get the signal line associated with the output port of the inport
    line = get_param(ph.Outport,'Line');
    if line==-1 %Not conected to any block
        subsys=find_system(EH.def.definition,...
            'LookUnderMasks','off',...
            'FollowLinks','on','BlockType','SubSystem');
        UD=get_param(subsys,'UserData');
        devs={};
        for i=1:length(UD)
            if isfield(UD{i},'type')
                if UD{i}.type=='S'
                    devs{end+1}=subsys{i}; %filter storage devices (S)
                end
            end
        end
        
        names=get_param(devs,'Name'); %all dev names
        names=names(~cellfun(@isempty,erase(names,get_param(S{i},'Name')))); %remove this block
        labels=matlab.lang.makeValidName(names); %all, but this block, dev labels
        set_param(S{i},'label',matlab.lang.makeUniqueStrings(... %ensure uniqueness
            matlab.lang.makeValidName(get_param(S{i},'Name')),...
            labels))
        set_param(S{i},'n','0')
    else %Conected to any block
        %get the destination port handles for the signal
        dstport = get_param(line, 'Dstporthandle');
        %get the properties from the handle
        port = get(dstport);
        %get the block type
        UD=get_param(port.Parent,'UserData');
        if isfield(UD,'type')
            if UD.type=='O'
                set_param(S{i},'label',... %rename label equal to output block's label
                    get_param(port.Parent,'label'))
                set_param(S{i},'n',get_param(port.Parent,'n')) %set number equal to output block's number
            else
                error('The storage system is not conected to any valid Output')
            end
        else
            error('The storage system is not conected to any valid Output')
        end
        
    end
end

%% Read parameters from Simulink
%Devices
for i=1:length(D)
    EH.def.dev.D(i).n=eval(get_param(D{i},'n'));
    EH.def.dev.D(i).I.n=eval(get_param(D{i},'nI'));
    EH.def.dev.D(i).I.t=eval(get_param(D{i},'tI'));
    EH.def.dev.D(i).O.n=eval(get_param(D{i},'nO'));
    EH.def.dev.D(i).O.t=eval(get_param(D{i},'tO'));
    EH.def.dev.D(i).O.s=eval(get_param(D{i},'sO'));
    EH.def.dev.D(i).l=convertCharsToStrings(get_param(D{i},'label'));
    EH.def.dev.D(i).Dimin=read_param(D{i},'Dimin');
    EH.def.dev.D(i).Dimax=read_param(D{i},'Dimax');
    EH.def.dev.D(i).Domin=read_param(D{i},'Domin');
    EH.def.dev.D(i).Domax=read_param(D{i},'Domax');
    EH.def.dev.D(i).nd=read_param(D{i},'nd');
end
%sort elements
EH.def.dev.D=table2struct(sortrows(struct2table(EH.def.dev.D),'n'));

%Inputs
for i=1:length(I)
    EH.def.I.t(i,:)=convertCharsToStrings(get_param(I{i},'t'));
    EH.def.I.l(i,:)=convertCharsToStrings(eval(get_param(I{i},'l')));
    EH.def.I.colour(i,:)=eval(get_param(I{i},'colour'));
    EH.def.I.cost(:,i)=read_param(I{i},'c');
    EH.def.I.Imin(:,i)=read_param(I{i},'I_min');
    EH.def.I.Imax(:,i)=read_param(I{i},'I_max');
end


%Outputs
for i=1:length(O)
    EH.def.O.t(i,:)=convertCharsToStrings(get_param(O{i},'t'));
    EH.def.O.l(i,:)=convertCharsToStrings(eval(get_param(O{i},'l')));
    EH.def.O.colour(i,:)=eval(get_param(O{i},'colour'));
    if strcmp(get_param(O{i},'dependance'),'device')
        dev_name=get_param(O{i},'dev');
        if dev_name=='0'
            error('Dependance for output %d needs to be redefined.',i)
        end
        EH.def.O.d(i)=eval(get_param(D{~cellfun(@isempty,...
            (strfind(get_param(D,'Name'),dev_name)))},'n'));
        if get_param(O{i},'devout')==0
            error('\nNo device output selected for output %d',i);
        else
            EH.def.O.dO(i)=eval(get_param(O{i},'devout'));
        end
        EH.def.O.ch(i)=0;
        EH.def.O.dis(i)=0;
    elseif strcmp(get_param(O{i},'dependance'),'charge')
        dev_name=get_param(O{i},'dev');
        if dev_name=='0'
            error('Dependance for output %d needs to be redefined.',i)
        end
        EH.def.O.d(i)=0;
        EH.def.O.dO(i)=0;
        EH.def.O.ch(i)=eval(get_param(S{~cellfun(@isempty,...
            (strfind(get_param(S,'Name'),dev_name)))},'n'));
        EH.def.O.dis(i)=0;
    elseif strcmp(get_param(O{i},'dependance'),'discharge')
        dev_name=get_param(O{i},'dev');
        if dev_name=='0'
            error('Dependance for output %d needs to be redefined.',i)
        end
        EH.def.O.d(i)=0;
        EH.def.O.dO(i)=0;
        EH.def.O.ch(i)=0;
        EH.def.O.dis(i)=eval(get_param(S{~cellfun(@isempty,...
            (strfind(get_param(S,'Name'),dev_name)))},'n'));
    else
        EH.def.O.d(i)=0;
        EH.def.O.dO(i)=0;
        EH.def.O.ch(i)=0;
        EH.def.O.dis(i)=0;
    end
    
    if max(~cellfun(@isempty,...
            (strfind(get_param(S,'label'),get_param(O{i},'label')))))==1
        EH.def.O.st(i)=1;
    else
        EH.def.O.st(i)=0;
    end
    
    if strcmp(get_param(O{i},'ms'),'on')
        EH.def.O.ms(i)=1;
        EH.def.O.price{i}=eval(get_param(O{i},'s'));
        EH.def.O.Mmin{i}=eval(get_param(O{i},'M_min'));
        EH.def.O.Mmax{i}=eval(get_param(O{i},'M_max'));
    else
        EH.def.O.ms(i)=0;
        EH.def.O.price{i}=0;
        EH.def.O.Mmin{i}=0;
        EH.def.O.Mmax{i}=0;
    end
        
    if strcmp(get_param(O{i},'prop'),'on')&&...
            ~strcmp(get_param(O{i},'dependance'),'none')
        EH.def.O.O{i}=0;
        EH.def.O.dF(i)=eval(get_param(O{i},'k'));
    else
        EH.def.O.O{i}=eval(get_param(O{i},'O'));
        EH.def.O.dF(i)=0;
    end   
    
    EH.def.O.labels(i,:)=...
        [eval(get_param(O{i},'f_label')) eval(get_param(O{i},'st_label'))];
    EH.def.O.factor(i)=eval(get_param(O{i},'factor'));
    EH.def.O.ll(i)=eval(get_param(O{i},'lb'));
    EH.def.O.ul(i)=eval(get_param(O{i},'ub'));
    EH.def.O.st_factor(i)=eval(get_param(O{i},'st_factor'));
    EH.def.O.st_ll(i)=eval(get_param(O{i},'st_lb'));
    EH.def.O.st_ul(i)=eval(get_param(O{i},'st_ub'));
    
end

%Storage
for i=1:length(S)
    EH.def.dev.S(i).l=convertCharsToStrings(get_param(S{i},'label'));
    EH.def.dev.S(i).Qchmin=read_param(S{i},'Qchmin');
    EH.def.dev.S(i).Qchmax=read_param(S{i},'Qchmax');
    EH.def.dev.S(i).nch=read_param(S{i},'nch');
    EH.def.dev.S(i).Qdismin=read_param(S{i},'Qdismin');
    EH.def.dev.S(i).Qdismax=read_param(S{i},'Qdismax');
    EH.def.dev.S(i).ndis=read_param(S{i},'ndis');
    EH.def.dev.S(i).Smin=read_param(S{i},'Smin');
    EH.def.dev.S(i).Smax=read_param(S{i},'Smax');
    EH.def.dev.S(i).ns=read_param(S{i},'ns');
    EH.def.dev.S(i).S0=read_param(S{i},'S0');
end

%Nodes
%To be used in future releases

%% Ddownstream links from labels (b: begining, e: end)

%Get all the lines on the diagram
lines=find_system(EH.def.definition,'FindAll','on','type','line');

%Get source and destination blocks and ports
src_b_handle = cell2mat(get_param(lines, 'SrcBlockHandle'));
dst_b_handle = cell2mat(get_param(lines, 'DstBlockHandle'));

%Get User Data
src_b_UD = get_param(src_b_handle, 'UserData');
dst_b_UD = get_param(dst_b_handle, 'UserData');

%Get index for filtering
ind=and(cellfun(@(x) isfield(x,'type'),src_b_UD),...
    cellfun(@(x) isfield(x,'type'),dst_b_UD));
valid_types={'D','S','I','O','CN','DN'};
for i=1:length(ind)
    if ind
        if ~(any(strcmp(src_b_UD{i}.type,valid_types))&&...
                any(strcmp(dst_b_UD{i}.type,valid_types)))
            ind=false;
        end
    end
end

%Filter the above variables to consider only ODEHubs blocks
lines = lines(ind);
src_b_handle = src_b_handle(ind);
dst_b_handle = dst_b_handle(ind);

%Preset empty arrays
links=[strings(0) strings(0)];

%Omit these blocks
omit={'I','S','CN','DN'};

%Inputs
for i=1:length(I)
b = convertCharsToStrings(get_param(I{i},'label'));
hb = getSimulinkBlockHandle(I{i},true);
h0 = hb; %first time recursive count_links function is called
[links] = count_links(h0,0,hb,src_b_handle,dst_b_handle,lines,links,omit);
end

%Devices
for i=1:length(D)
b = convertCharsToStrings(get_param(D{i},'label'));
hb = getSimulinkBlockHandle(D{i},true);
h0 = hb; %first time recursive count_links function is called
[links] = count_links(h0,0,hb,src_b_handle,dst_b_handle,lines,links,omit);
end

EH.def.linkslabel.b=links(:,1);
EH.def.linkslabel.e=links(:,2);

end

function [out] = read_param(block,param)
out=eval(get_param(block,param));
if ~iscell(out)
    out={out};
end

end

function [links] = count_links(h0,p0,hb,src,dst,lines,links,omit)
%recursive function to find links from I to O based on Simulink connectors


%b: source link
%e: destination link

%h0: block source handle
%p0: port source handle
%hb: beggining block handle to start search (for recursivity)
%omit: omit these intermediate blocks during search

%Find beggining handle in source column
ind=find(src==hb);

%Start recursive search in src dst vectors
for i=1:length(ind)
    UD_dst=get_param(dst(ind(i)),'UserData');
    UD_src=get_param(src(ind(i)),'UserData');
    if  any(strcmp(UD_src.type,{'I','D'}))
        p0=get_param(lines(ind(i)),'SrcportHandle');    
    end
    if ~any(strcmp(UD_dst.type,omit))
        b=convertCharsToStrings(get_param(h0,'label'));
        e=convertCharsToStrings(get_param(dst(ind(i)),'label'));
        in_ports=get_param(dst(ind(i)),'Ports');
        out_ports=get_param(h0,'Ports');
        if out_ports(2)>1            
            n=get_param(p0,'PortNumber');
            b=strcat(b,"_O",num2str(n));
        end
        if in_ports(1)>1
            p=get_param(lines(ind(i)),'DstportHandle');
            n=get_param(p,'PortNumber');
            e=strcat(e,"_I",num2str(n));
        end        
        links(end+1,:)=[b;e];
    else
        [links] = count_links(h0,p0,dst(ind(i)),src,dst,lines,links,omit);
    end
end


end