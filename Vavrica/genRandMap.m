%mapa je definovana ako 100 x 100 mriezka - jedno policko = 10 cm;
% 0 - volne, 1 - prekazka
%
%
%
function [map] = genRandMap(rows,columns,prekazok)
    
global map;
    map = zeros(rows,columns);
    
    for i=1:1:prekazok
        x = round((rand()*0.8+0.1)*columns);
        y = round((rand()*0.8+0.1)*rows);
        
        map(x,y) = 1;
    end;
end


