%% INICIALIZACION LIBRERIA

sys='Libreria';
open_system(sys);%se podria poner solo un load_system
set_param(sys,'Lock','off');


clear Ud
Ud.SFunction=1;
Ud.icono=imread('Libreria_Imagenes/celula_fotovoltaica.jpg')
name='/Celula_Fotovoltaica';
Libreria/Celula_Fotovoltaica
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');

name='/Coeficiente_Reparto';
Ud.SFunction=1;
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');

name='/Almacenamiento';
clear Ud;
Ud.Store_system=1;
Ud.icono=imread('Libreria_Imagenes/almacenamiento.jpg')
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');


clear Ud
Ud.SFunction=1;
name='/Caldera_tipo1';
Ud.icono=imread('Libreria_Imagenes/caldera.jpg')
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');

clear Ud
Ud.SFunction=1;
name='/Caldera_tipo2';
Ud.icono=imread('Libreria_Imagenes/caldera.jpg')
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');

clear Ud
Ud.SFunction=1;
name='/Absorption_chiller';
Ud.icono=imread('Libreria_Imagenes/ab_chiller.jpg')
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');

clear Ud
Ud.SFunction=1;
Ud.Bloque_binario=1;
name='/Reversible_heat_pump';
Ud.icono=imread('Libreria_Imagenes/reversible_heat_pump.jpg')
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');

clear Ud
Ud.SFunction=1;
name='/Bomba_de_calor';
Ud.icono=imread('Libreria_Imagenes/reversible_heat_pump.jpg')
set_param(strcat(sys,name),'UserData',Ud);
set_param(strcat(sys,name),'UserDataPersistent','on');

sys='Libreria';


%% GUARDAR CAMBIOS
%sale blq por la mascara de cada bloque
% set_param('EH_prueba','Lock','on');
% save_system(sys)







