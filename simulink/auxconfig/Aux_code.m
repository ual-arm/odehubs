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


%% Update popup menu (devices)
% p=Simulink.Mask.get(gcb);
% subsys=find_system(gcs,...
%     'LookUnderMasks','on',...
%     'FollowLinks','on','BlockType','SubSystem');
% UD=get_param(subsys,'UserData');
% devs={};
% for i=1:length(UD)
%     if isfield(UD{i},'type')
%         if UD{i}.type=='D'
%             devs{end+1}=subsys{i}; %filter devices (D) or storage (S)
%         end
%     end
% end
% p.Parameters(strcmp(get_param(gcb,'MaskNames'),...
%                     'dev')).TypeOptions=... %path to popout menu
%                                         get_param(devs,'Name'); %dev names
 
%% Convergence node 
% gcb='Libreria/Convergence Node';
% Inputs=3;
% 
% blocks=find_system(gcb,...
%         'LookUnderMasks','on',...
%         'FollowLinks','on','BlockType','Inport');
% lines=find_system(gcb,...
%         'LookUnderMasks','on',...
%         'FollowLinks','on','FindAll','On','type','line'); 
% %Delete all input ports
% delete_block(blocks)
% 
% %Delete all the lines
% delete_line(lines)
% 
% %Set sum block inputs
% set_param(strcat(gcb,'/Sum'),'Inputs',num2str(Inputs))
% 
% %Build new inputs and lines            
% for k=1:Inputs
%         newblock=[gcb,char(strcat('/In',num2str(k)))];
%         add_block('built-in/Inport',newblock);
%         P0=get_param(newblock,'position');
%         P=P0;
%         P(1)=P0(1)+8; %left
%         P(2)=P0(2)+18+k*20-100; %top
%         P(3)=P0(3)-8; %right
%         P(4)=P0(4)-18+k*20-100; %bottom
%         set_param(newblock,'position',P);
%         add_line(gcb,...
%                 strcat('In',num2str(k),'/1'),... %out
%                 strcat('Sum/',num2str(k)),...%in
%                 'autorouting','on')
% end
% add_line(gcb,...
%                 'Sum/1',... %out
%                 'Out1/1',...%in
%                 'autorouting','on')
            
%% Divergence node
% gcb='Libreria/Divergence Node';
% Outputs=5;
% blocks1=find_system(gcb,...
%         'LookUnderMasks','on',...
%         'FollowLinks','on','BlockType','Gain');
% blocks2=find_system(gcb,...
%         'LookUnderMasks','on',...
%         'FollowLinks','on','BlockType','Outport');
% lines=find_system(gcb,...
%         'LookUnderMasks','on',...
%         'FollowLinks','on','FindAll','On','type','line'); 
% 
% 
% %Delete all gain ports
% delete_block(blocks1)    
%     
% %Delete all output ports
% delete_block(blocks2)
% 
% %Delete all the lines
% delete_line(lines)
% 
% %Set mux and demux ports
% set_param(strcat(gcb,'/Mux'),'Inputs',num2str(Outputs))
% set_param(strcat(gcb,'/Demux'),'Outputs',num2str(Outputs))
% 
% %Build new inputs and lines            
% for k=1:Outputs
%         
%         %Add output ports
%         newblock=[gcb,char(strcat('/Out',num2str(k)))];
%         add_block('built-in/Outport',newblock);
%         P0=get_param(newblock,'position');
%         P=P0;
%         P(1)=P0(1)+8; %left
%         P(2)=P0(2)+18-80+k*20; %top
%         P(3)=P0(3)-8; %right
%         P(4)=P0(4)-18-80+k*20; %bottom
%         set_param(newblock,'position',P);
%         
%         %Add gains
%         newblock=[gcb,char(strcat('/Gain',num2str(k)))];
%         add_block('built-in/Gain',newblock);
%         P0=get_param(newblock,'position');
%         P=P0;
%         P(1)=P0(1)+8-40; %left
%         P(2)=P0(2)+18-80+k*20; %top
%         P(3)=P0(3)-8-40; %right
%         P(4)=P0(4)-18-80+k*20; %bottom
%         set_param(newblock,'position',P);
%         %set equal default gains
%         set_param(strcat(gcb,'/Gain',num2str(k)),'Gain',num2str(1/Outputs))
%         
%         %link in to mux ports
%         add_line(gcb,...
%                 'In1/1',... %out
%                 strcat('Mux/',num2str(k)),...%in
%                 'autorouting','on')
%         %link demux ports to gains
%         add_line(gcb,...
%                 strcat('Demux/',num2str(k)),... %out
%                 strcat('Gain',num2str(k),'/1'),...%in
%                 'autorouting','on')
%         %link gains to output ports    
%         add_line(gcb,...
%                 strcat('Gain',num2str(k),'/1'),... %out
%                 strcat('Out',num2str(k),'/1'),...%in
%                 'autorouting','on')
% end
% %link mux to demux
% add_line(gcb,...
%                 'Mux/1',... %in
%                 'Demux/1',...%out
%                 'autorouting','on')  

%% Block UD data
            
% blocks = find_system(gcs,'Selected','on');
% for i = 1 : length(blocks)
% UD{i}=get_param(blocks{i},'UserData');
% % UD{i}.type='CN';
% try
%     UD{i}=rmfield(UD{i},'ID');
% catch
% end
% set_param(blocks{i},'UserData',UD{i});
% end

%% Copy mask
% pSource = Simulink.Mask.get('Libreria/Input')
% pDest = Simulink.Mask.create('Libreria/Output')
% pDest.copy(pSource)
% 
% 
%% Get all Tab Names returned in cell array
% TabNames = get_param(gcb,'MaskTabNames');
% % Get visibility of all parameters
% TabVis = get_param(gcbh,'MaskVisibilities');
% % Find the index of Tab in which, you want to disable parameters
% DisableTabIdx = find(strcmp(TabNames,'MS'));
% % Turn of the visibility of parameters present in the Tab
% for x=1:length(DisableTabIdx)
%     TabVis{DisableTabIdx(x)}='off';
% end
% % Pass the modify visibility values to block
% set_param(gcbh,'MaskVisibilities',TabVis)

% %% Delte selected masks
% blocks = find_system(gcs,'Selected','on');
% for i=1:length(blocks)
% p = Simulink.Mask.get(blocks{i})
% p.delete
% end
% 
% %% Copy selected masks
% blocks = find_system(gcs,'Selected','on');
% for i=1:length(blocks)
% pSource = Simulink.Mask.get('ODEHUBS_components/Absorption Chiller')
% pDest = Simulink.Mask.create(blocks{i})
% pDest.copy(pSource)     
% end


%% Update labels
% subsys=find_system(gcs,...
%     'LookUnderMasks','on',...
%     'FollowLinks','on','BlockType','SubSystem');
% UD=get_param(subsys,'UserData');
% devs={};
% for i=1:length(UD)
%     if isfield(UD{i},'type')
%         if UD{i}.type=='D'
%             devs{end+1}=subsys{i}; %filter devices (D)
%         end
%     end
% end
% 
% names=get_param(devs,'Name'); %all dev names
% names=names(~cellfun(@isempty,erase(names,get_param(gcb,'Name')))); %remove this block
% labels=matlab.lang.makeValidName(names); %all, but this, dev labels
% set_param(gcb,'label',matlab.lang.makeUniqueStrings(... %ensure uniqueness
%                       matlab.lang.makeValidName(get_param(gcb,'Name')),...
%                                                 labels))

%% Storage output conection
% %get port handles
% ph=get_param(gcb,'PortHandles');
% %get the signal line associated with the output port of the inport
% line = get_param(ph.Outport,'Line');
% if line==-1 %Not conected to any block
%     subsys=find_system(gcs,...
%         'LookUnderMasks','on',...
%         'FollowLinks','on','BlockType','SubSystem');
%     UD=get_param(subsys,'UserData');
%     devs={};
%     for i=1:length(UD)
%         if isfield(UD{i},'type')
%             if UD{i}.type=='S'
%                 devs{end+1}=subsys{i}; %filter storage devices (S)
%             end
%         end
%     end
%     
%     names=get_param(devs,'Name'); %all dev names
%     names=names(~cellfun(@isempty,erase(names,get_param(gcb,'Name')))); %remove this block
%     labels=matlab.lang.makeValidName(names); %all, but this block, dev labels
%     set_param(gcb,'label',matlab.lang.makeUniqueStrings(... %ensure uniqueness
%         matlab.lang.makeValidName(get_param(gcb,'Name')),...
%         labels))
% else %Conected to any block
%     %get the destination port handles for the signal
%     dstport = get_param(line, 'Dstporthandle');
%     %get the properties from the handle
%     port = get(dstport);
%     %get the block type
%     UD=get_param(port.Parent,'UserData');
%     if isfield(UD,'type')
%         if UD.type=='O'
%             set_param(gcb,'label',... %rename label equal to output block's label
%                 strcat(get_param(port.Parent,'t'),get_param(port.Parent,'l')))
%         else
%             error('The storage system is not conected to any valid Output')
%         end
%     else
%         error('The storage system is not conected to any valid Output')
%     end
%     
% end

%% Update icon
blocks = find_system(gcs,'Selected','on');
UD=get_param(blocks{:},'UserData');
UD.icon=imread('chp.jpg');
set_param(blocks{:},'UserData', UD);
set_param(blocks{:},'UserDataPersistent', 'on');

%% Set size
blocks = find_system(gcs,'Selected','on');
for i=1:length(blocks)
    p=get_param(blocks{i},'Position')    
    set_param(blocks{i},'Position',[p(1) p(2) p(1)+40 p(2)+40])
end


