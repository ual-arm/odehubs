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

function c_Ei = e_cost_func(data,date,samples,tm)
%User defined function for ODEHub Examples
%

%Electricity
%info: http://www.idae.es/sites/default/files/estudios_informes_y_estadisticas/tarifas_reguladas_julio_2017.pdf
%Period   Price(€/kWh)      Winter             Summmer
% P1:       0.018762        18-22h              11-15h
% P2:       0.012575      8-18h/22-24h      8-11h/15-24h  
% P3:       0.004670         0-8h                0-8h

%endesa fixed prices: 
%https://www.endesaclientes.com/articulos/tarifas-reguladas-luz-gas.html
%Period   Price(€/kWh)      Winter             Summmer
% P1:       0.168899        18-22h              11-15h
% P2:       0.093162      8-18h/22-24h      8-11h/15-24h  
% P3:       0.073738         0-8h                0-8h


%winter and summer coincide with DST changes
% https://es.support.somenergia.coop/article/176-que-horarios-tienen-los-periodos-de-la-tarifa-3-0a

%Taxes (not included in prices): 21 % IVA

dates=data.Time;
[h,m,s]=hms(dates);
time=h+m/60+s/3600;
priceE=0.168899*not(isdst(dates)).*and((time>=18),(time<22))+... %winter P1
      0.093162*not(isdst(dates)).*and((time>=8),(time<18))+... %winter P2.1
      0.093162*not(isdst(dates)).*and((time>=22),(time<24))+... %winter P2.2
      0.073738*not(isdst(dates)).*and((time>=0),(time<8))+... %winter P3
      0.168899*isdst(dates).*and((time>=11),(time<15))+... %summer P1
      0.093162*isdst(dates).*and((time>=8),(time<11))+... %summer P2.1
      0.093162*isdst(dates).*and((time>=15),(time<24))+... %summer P2.2
      0.073738*isdst(dates).*and((time>=0),(time<8)); %summer P3

c_Ei=1.21*priceE'; % €/kWh
end

