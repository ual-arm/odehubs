function [Crrp] = crear_restricciones_dispatch_factor(Rb,R)

u=size(Rb,2);
Crrp=[];
for z=1:u
    Crrp=[Crrp Rb(z).p]
end
u=size(R,2);
for z=1:u
    uu=size(R(z).p,2)
    for zz=1:uu
        Crrp=[Crrp R(z).p(zz).p];
    end
end