%Fija las variables de cada bloque almacen
function [] = fijo_variables_bloque_almacen(k,bloque_almacen,T_sample)
% k: contador del bloque almacén
% bloque_almacen: bloque almacén
% T_sample: tiempo de muestreo

system=strcat(bloque_almacen,'/Qch');
set_param(system,'VariableName',strcat('Qchp',string(k)));
system=strcat(bloque_almacen,'/ef');
set_param(system,'VariableName',strcat('Ech',string(k)))
system=strcat(bloque_almacen,'/Qdis');
set_param(system,'VariableName',strcat('Qdisp',string(k)));
system=strcat(bloque_almacen,'/eff');
set_param(system,'VariableName',strcat('Edis',string(k)));
system=strcat(bloque_almacen,'/In1');
set_param(system,'SampleTime',string(T_sample));
end