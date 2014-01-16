clc
clear
close all;
map = genRandMap(100,100,50);
fuzcar =  readfis('fuzcar');
fuzcar2 =  readfis('fuzcar2');
%vsetky prekazky
    [rows,columns] = size(map);
    for i=1:1:rows
        for j =1:1:columns
            if (map(i,j) == 1)
                patch([i*10-5 i*10+5 i*10+5 i*10-5],[j*10-5 j*10-5 j*10+5 j*10+5], 'black'); hold on;
            end;
        end;
    end;
    axis([-100 1100 -100 1100]);
   
    % tazisko auticka
    posx =50;
    posy = 50;
    
    %rozmery
    xSize=20;
    ySize=20;
    sensRange = 100;
    
    angle = 45;
    left = -1;
    center =-1;
    right = -1;
   
   gcf();
   %vytvorenie objektovych handlov
   p1 = [posx+cos(deg2rad(0+45))*sensRange, posy+sin(deg2rad(0+45))*sensRange];
   p2 = [posx+cos(deg2rad(0+15))*sensRange, posy+sin(deg2rad(0+15))*sensRange];
   p3 = [posx+cos(deg2rad(0+-15))*sensRange, posy+sin(deg2rad(0-15))*sensRange];
   p4 = [posx+cos(deg2rad(0+-45))*sensRange, posy+sin(deg2rad(0-45))*sensRange];
  
   %pociatocne suradnice auticka
   global refAuto;
   refAuto=patch([posx-xSize/2 posx+xSize/2 posx+xSize/2 posx-xSize/2], [posy-ySize/2 posy-ySize/2 posy+ySize/2 posy+ySize/2], 'green');
   set(refAuto,'Visible','on');
   
   %objekt ktory sa bude hybat
   global auto;
   auto = patch([posx-xSize/2 posx+xSize/2 posx+xSize/2 posx-xSize/2], [posy-ySize/2 posy-ySize/2 posy+ySize/2 posy+ySize/2], 'green');
  
   
   %kuzely
   global k1;
   k1 = patch ([posx p1(1) p2(1) posx],[posy p1(2) p2(2) posy],'blue');
   global refK1;
   refK1 = patch ([posx p1(1) p2(1) posx],[posy p1(2) p2(2) posy],'blue');
   set(refK1,'Visible','off');
   
   global k2;
   k2= patch ([posx p2(1) p3(1) posx],[posy p2(2) p3(2) posy],'blue');
   global refK2;
   refK2= patch ([posx p2(1) p3(1) posx],[posy p2(2) p3(2) posy],'blue');
    set(refK2,'Visible','off');
   
   global k3;
   k3 = patch ([posx p3(1) p4(1) posx],[posy p3(2) p4(2) posy],'blue');
   global refK3;
   refK3 = patch ([posx p3(1) p4(1) posx],[posy p3(2) p4(2) posy],'blue');
    set(refK3,'Visible','off');
   
 goalX = 900;
 goalY = 900;   
 sim('cesta')