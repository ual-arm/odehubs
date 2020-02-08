%% Creación de bloques
%Hay que crear el blque en la librería antes de ejecutar el código
sys='Libreria'; %nombre del archivo que contiene la librería
open_system(sys); %abrir en Simulink
set_param(sys,'Lock','off') %Desbloquear librería

clear Ud %Borra el contenido de esta variable, por precaución
Ud.SFunction=1;
Ud.icono=imread('desaladora_icon.bmp'); %lee la imagen que se inserta
name='/Desaladora'; %Nombre del bloque, que se ha de crear previamente con
                    % la barra inclinada delante
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');


origin='/Bomba_Riego'; %Nombre del bloque que se tomado como referencia con
                      % la barra inclinada delante

%Copia la máscara                      
pSource = Simulink.Mask.get(strcat(sys,origin));
pDest = Simulink.Mask.create(strcat(sys,name));
pDest.copy(pSource)
