%% Creaci�n de bloques
%Hay que crear el blque en la librer�a antes de ejecutar el c�digo
sys='Libreria'; %nombre del archivo que contiene la librer�a
open_system(sys); %abrir en Simulink
set_param(sys,'Lock','off') %Desbloquear librer�a

clear Ud %Borra el contenido de esta variable, por precauci�n
Ud.SFunction=1;
Ud.icono=imread('desaladora_icon.bmp'); %lee la imagen que se inserta
name='/Desaladora'; %Nombre del bloque, que se ha de crear previamente con
                    % la barra inclinada delante
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');


origin='/Bomba_Riego'; %Nombre del bloque que se tomado como referencia con
                      % la barra inclinada delante

%Copia la m�scara                      
pSource = Simulink.Mask.get(strcat(sys,origin));
pDest = Simulink.Mask.create(strcat(sys,name));
pDest.copy(pSource)
