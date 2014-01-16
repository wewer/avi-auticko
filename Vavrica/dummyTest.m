clc
clear
close all
map = genRandMap(100,100,50);

h=figure();
%for i=1:1:500
%    viewMap(map,i,i,45,h);
%end;


for (i=1:0.01:2*pi)
    viewMap(map,cos(i)*200+500,sin(i)*200+500,rad2deg(i)+90,h);
end;