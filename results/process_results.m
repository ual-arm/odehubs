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

function [EH] = process_results(EH,showmodel)
%Show the simulation results in figures

%% Results
if nargin == 0 % Plot old simulations (by default the most recent)
    % Determine where your m-file's folder is
    folder = fileparts(which(mfilename));
    cd(folder);
    files=dir('*.mat');
    aux=struct2cell(files);
    aux=cell2mat(aux(size(aux,1),:));
    [~,aux]=max(aux);
    load(files(aux).name);
    showmodel=true;
end

%figures parameters
aaF=0.25;
bbF=0.25;
ccF=0.25;
ddF=0.25;
FF=12;

if showmodel
    %% Preset (model)
    t=EH.model.data.Time(1:EH.simparam.nm/EH.simparam.tm)';
    tm=EH.simparam.tm;
    S(:,1)=EH.model.data.S0{1};
    for k=1:EH.simparam.nm/EH.simparam.tm
        O(:,k)=EH.model.param.O{k};
        C(:,:,k)=EH.model.param.C{k};
        Ci(:,:,k)=EH.model.param.Ci{k};
        Cdo(:,:,k)=EH.model.param.Cdo{k};
        c(:,k)=EH.model.param.c{k};
        s(:,k)=EH.model.param.s{k};
        M(:,k)=EH.model.data.M{k};
        P(:,k)=EH.model.data.P{k};
        S(:,k+1)=EH.model.data.S{k};
        dI(:,k)=EH.model.data.dI{k};
        dM(:,k)=EH.model.data.dM{k};
        dD(:,k)=EH.model.data.dD{k};
        dQch(:,k)=EH.model.data.dQch{k};
        dQdis(:,k)=EH.model.data.dQdis{k};
        I(:,k)=Ci(:,:,k)*P(:,k);
    end
else
    if EH.simparam.MPCmode
        %% Preset (MPC)
        t=EH.control.MPC.model.Time';
        S(:,1)=EH.model.data.S0{1};
        tm=EH.simparam.tm_MPC;
        for k=1:EH.simparam.nm/tm
            O(:,k)=EH.control.MPC.model.O{k}(:,1);
            C(:,:,k)=EH.control.MPC.model.C{k}(:,:,1);
            Ci(:,:,k)=EH.control.MPC.model.Ci{k}(:,:,1);
            Cdo(:,:,k)=EH.control.MPC.model.Cdo{k}(:,:,1);
            c(:,k)=EH.control.MPC.model.c{k}(:,1);
            s(:,k)=EH.control.MPC.model.s{k}(:,1);
            M(:,k)=EH.control.MPC.results.M{k}(:,1);
            P(:,k)=EH.control.MPC.results.P{k}(:,1);
            S(:,k+1)=EH.control.MPC.results.S{k}(:,1);
            dI(:,k)=EH.control.MPC.results.dI{k}(:,1);
            dM(:,k)=EH.control.MPC.results.dM{k}(:,1);
            dD(:,k)=EH.control.MPC.results.dD{k}(:,1);
            dQch(:,k)=EH.control.MPC.results.dQch{k}(:,1);
            dQdis(:,k)=EH.control.MPC.results.dQdis{k}(:,1);
            I(:,k)=Ci(:,:,k)*P(:,k);
        end
    else
        %% Preset (Scheduling)
        t=EH.control.MPC.model.Time:minutes(EH.simparam.tm_MPC):...
          EH.control.MPC.model.Time+...
          minutes(EH.control.MPC.model.H{:}-1);
        tm=EH.simparam.tm_MPC;
        O=EH.control.MPC.model.O{1,1};
        C=EH.control.MPC.model.C{1,1,1};
        Ci=EH.control.MPC.model.Ci{1,1,1};
        Cdo=EH.control.MPC.model.Cdo{1,1,1};
        c=EH.control.MPC.model.c{1,1};
        s=EH.control.MPC.model.s{1,1};
        M=EH.control.MPC.results.M{1,1};
        P=EH.control.MPC.results.P{1,1};
        S=[EH.model.data.S0{1} EH.control.MPC.results.S{1,1}];
        dI=EH.control.MPC.results.dI{1,1};
        dM=EH.control.MPC.results.dM{1,1};
        dD=EH.control.MPC.results.dD{1,1};
        dQch=EH.control.MPC.results.dQch{1,1};
        dQdis=EH.control.MPC.results.dQdis{1,1};
        for k=1:EH.control.MPC.model.samples{:}
            I(:,k)=Ci(:,:,k)*P(:,k);
        end
    end
end

%Build dO matrix
dO=zeros(EH.def.O.N,size(dD,2));
for j=1:EH.def.O.N
    if EH.def.O.d(j)>0
        dev=find((EH.feat.dev.dev2(:,1)==EH.def.O.d(j))&...
            (EH.feat.dev.dev2(:,3)==EH.def.O.dO(j)));
        dO(j,:)=dD(dev,:);
    elseif EH.def.O.ch(j)>0&&~EH.def.O.dis(j)>0
        dO(j,:)=dQch(EH.def.O.ch(j),:)*(EH.def.O.ch(j)>0);
    elseif EH.def.O.dis(j)>0&&~EH.def.O.ch(j)
        dO(j,:)=dQdis(EH.def.O.dis(j),:)*(EH.def.O.dis(j)>0);
    elseif EH.def.O.ch(j)>0&&EH.def.O.dis(j)>0
        dO(j,:)=dQch(EH.def.O.ch(j),:)*(EH.def.O.ch(j)>0)+...
            dQdis(EH.def.O.dis(j),:)*(EH.def.O.dis(j)>0);
    else
        dO(j,:)=ones(1,size(dD,2));
    end
end

%% Tables
EH.results.O_Total=sum(O,2)*tm/60;
EH.results.I_Total=sum(I,2)*tm/60;
EH.results.M_Total=sum(M,2)*tm/60;
EH.results.c_Total=sum(I.*c,2)*tm/60;
EH.results.s_Total=sum(M.*s,2)*tm/60;

%% Figures
for j=1:EH.def.O.N
    set(figure,'Units','normalized','Position',...
        [aaF,bbF,ccF,ddF],'Color', [0.8 0.8 0.8]);
    hold on
    box on
    %Find paths corresponding to oputput j
    aux=strfind(EH.feat.paths.inv_paths,strcat("O",string(j)));
    Pvar=[];
    if iscell(aux)
    for k=1:length(aux)
        if aux{k}==1
            Pvar=[Pvar k];
            
        end
    end
    else       
        if aux==1
            Pvar=[Pvar 1];            
        end
    end
    clearvars aux k
    hmatrix=[];
    for k=1:length(Pvar)
        hmatrix=[hmatrix;
            P(Pvar(k),:).*reshape(C(j,Pvar(k),:),[1,size(P,2)])];
    end
    clearvars aux k
    
    %Find paths with the same input
    paths=char(EH.feat.paths.paths(Pvar));
    aux=strfind(EH.feat.paths.paths(Pvar),"-");
    try
        inputs=str2double(string(paths(:,2:cellfun(@min,aux)-1)));
    catch
        inputs=str2double(string(paths(:,2:min(aux)-1)));
    end
    
    %Add paths with the same input
    [input,~,idx] = unique(inputs);
    nu=numel(input);
    hmatrix2=zeros(nu,size(hmatrix,2));
    for k=1:nu
        hmatrix2(k,:)=sum(hmatrix(idx==k,:),1);
    end
    hmatrix=hmatrix2';   
    h=bar(t+hours(tm/120),hmatrix,...
        'stacked','BarWidth',1,'LineStyle',':');
    
    labels=cell(nu,1);
    for k=1:nu
        colour=EH.def.I.colour(input(k),:);
        labels{k}=char(strcat(EH.def.I.t(input(k)),EH.def.I.l(input(k))));
        labels{k}=char(strcat("I_",string(input(k))));
        h(k).FaceColor=colour;
    end
    clearvars aux aux2 k inputs colour
    
    %variable dependent outputs
    if EH.def.O.dF(j)>0
            dev=find((EH.feat.dev.dev2(:,1)==EH.def.O.d(j))&...
            (EH.feat.dev.dev2(:,3)==EH.def.O.dO(j)));
            paths=find(contains(EH.feat.paths.paths,strcat("D",string(EH.def.O.d(j)))));
            Caux=C;
            Caux(j,paths,:)=-EH.def.O.dF(j)*Cdo(dev,paths,:);
            Caux(Caux>0)=0;
            for i=1:size(O,2)
            O(j,i)=-Caux(j,:,i)*P(:,i);
            end
    end
    
    
    %stairs depending on on/off states given by dO
    stairs([t t(end)+hours(tm/60)],[O(j,:) O(j,end)].*[dO(j,:) dO(j,end)],...
        'Color',EH.def.O.colour(j,:),...
        'LineWidth',4);
    hold on
    
    if EH.def.O.ms(j)==1
        stairs([t t(end)+hours(tm/60)],[M(j,:) M(j,end)]+...
            [O(j,:) O(j,end)].*[dO(j,:) dO(j,end)],...
            'Color',EH.def.O.colour(j,:),...
            'LineWidth',2);
    end
    
    set(gca,'FontName','Palatino Linotype','FontSize', FF);
    xlim([t(1) t(end)+hours(tm/60)])
    if EH.def.O.factor(j)>0
        ylim([0 (round(max([max(sum(hmatrix,2))...
            max([M(j,:) M(j,length(M(j,:)))]+...
            [O(j,:) O(j,length(O(j,:)))])]),0)+...
            (round(max([max(sum(hmatrix,2))... %zero demand condition
            max([M(j,:) M(j,length(M(j,:)))]+...%this is to avoid errors
            [O(j,:) O(j,length(O(j,:)))])]),0)==0))*EH.def.O.factor(j)]);
    else
        ylim([EH.def.O.ll(j) EH.def.O.ul(j)])
    end
    datetick('x','HH:MM')
    xlabel('Coordinated Universal Time +1 (HH:MM)','FontName','Palatino Linotype','FontSize', FF);
    
    p=get(gca,'InnerPosition');
    p(1)=0.12;
    p(3)=0.74;
    set(gca,'InnerPosition',p);
    
    yleft =ylabel(...
        EH.def.O.labels{j,1},...
        'FontName','Palatino Linotype','FontSize', FF);
    
    pl = get(yleft,'position'); % get the current position property
    pl(1) = -0.08;              % distance
    set(yleft,'Position', pl);   % set the new position
    set(yleft,'VerticalAlignment','bottom');   % set the aligment
    set(yleft,'HorizontalAlignment','center');   % set the aligment
    
    if EH.def.O.st(j)~=0
        yyaxis right
        plot([t t(end)+hours(tm/60)],S(j,:),...
            'Color',EH.def.O.colour(j,:),...
            'LineWidth',4,'LineStyle','--')
        set(gca,'YColor',[0,0,0]);

        if EH.def.O.st_factor(j)>0
            ylim([0 (EH.model.param.S_max{1}(j)+...
                (EH.model.param.S_max{1}(j)==0))*EH.def.O.st_factor(j)]);
        else
            ylim([EH.def.O.st_ll(j) EH.def.O.st_ul(j)])
        end


        if  EH.def.O.st(j)==0
            ylim([0 1])
            yticks([0 1])
            yticklabels({' ',' '})
        end

        yright =ylabel(...
            EH.def.O.labels{j,2},...
            'FontName','Palatino Linotype','FontSize', FF);

        pr = get(yright,'position'); % get the current position property
        pr(1) = 1.18;              % distance
        set(yright,'Position', pr);   % set the new position
        set(yright,'VerticalAlignment','bottom');   % set the aligment
        set(yright,'HorizontalAlignment','center');   % set the aligment
    end

        labels{length(labels)+1,1}=char(strcat("O_",string(j)));
        if EH.def.O.ms(j)==1
            labels{length(labels)+1,1}=char(strcat("M_",string(j)));
        end
        if EH.def.O.st(j)==1
            labels{length(labels)+1,1}=char(strcat("S_",string(j)));
        end    
    hleg=legend(...
        labels,...
        'Location','northeast',...
        'Orientation','vertical');
    set(hleg,'FontSize', FF);
    
    if EH.results.exportgraphics
        cd(strcat(EH.def.folder,"\results\"));
        exportgraphics(gca,strcat("Sim_",string(j),'.pdf'),...
            'ContentType','vector','BackgroundColor','none');
        cd(strcat(EH.def.folder))
    end
end
end
