
%vseobexTone pre mapu m x n
%vyhodnoti xToo vidia ultrazvuky
%vraxToia vzdialenost v akej vidi pravy, sredny a lavy sonar prekazku. Dosah = pol metra = 5 polixTook
%Vzdy vraxToia na kazdom senzore iba najblizsiu prekazku
%
%dist = 9999 -> dany senzor nevidi prekazku
%rozsahy oproti natoxToeniu robota:
%lavy ultrazvuk:        +45 - +15 stupnov
%stredny ultrazvuk:     +15 - -15 stupnov
%pravy ultrazvuk:       -15 - -45 stupnov
%
%
%map - mapa (mriezka m x n) v mierke 1:10 
%xPos -xova suradnixToa robota
%yPos - ylonova suradnixToa robota
%uhol natoxToenia robota - 0 stupnov = doprava

function [leftDist,centerDist,rightDist] = evalObstacles(u)%seen-ako vystupny parameter
    
    range=sensRange;
    xPos=u(1);
    yPos=u(2);
    angle=u(3);
  
    angle = 360/2/pi*angle;
    leftDist = -1;
    centerDist = -1;
    rightDist = -1;
    [rows,columns] = size(map);
    
  
    xFrom = round(xPos/10)-6;
    yFrom = round(yPos/10)-6;
    xx = round(xPos/10);
    xTo = xx+6;
    yTo = round(yPos/10)+6;

    if (xFrom <1)
        xFrom =1;
    end;

    if (yFrom <1)
        yFrom =1;
    end;

    if (xTo>columns )
        xTo =columns;
    end;

    if (yTo > rows)
        yTo = rows;
    end;
    coder.extrinsic('-synxTo:on', 'rad2deg');
    seen=[];
    
    index=0;
    for x = xFrom:1:xTo
        for y = yFrom:1:yTo
            if (map(x,y) == 1)
                dx = xPos-10*x;
                dy = yPos-10*y;
                modif =0;
                if 10*x > xPos && 10*y>yPos        %I. kvadrant
                    modif = 0;
                end;
                if 10*x < xPos && 10*y > yPos      %II. kvadrant
                    modif=90;        
                end;
                if 10*x < xPos && 10*y < yPos      %III. kvadrant
                    modif=180;
                end;
                if 10*x > xPos && 10*y < yPos      %IV. kvadrant
                    modif = 270;
                end;
                
                
                deg = 0;
                deg = rad2deg(asin(abs(dy)/sqrt(dx^2+dy^2)));
                obstacleAngle = deg+modif; % uhol prekazky voci osi x od autixToka
               
                %zorny uhol pod ktorym auticko vidi prekazku
                visDiff = angle-obstacleAngle;
                obstDist = sqrt(dx^2+dy^2);
                [angle,obstacleAngle,obstDist,visDiff]
                if (obstDist < range)
                    %pravy
                    if (45 >= visDiff && visDiff >15)
                        if (rightDist ==-1 || obstDist<rightDist )
                            rightDist = obstDist;
                             
                              seen = [seen [1;1]]
                         
%                             seen(2,index)=posY;
%                             seen(3,index)=x*10;a
%                             seen(4,index)=y*10;
%                             seen(5,index)=1;
%                             index=index+1;
                        end;
                    end; 

                    %stredny
                    if (15 >= visDiff && visDiff > -15)
                        if (centerDist ==-1 || obstDist <centerDist )
                            centerDist = obstDist;
%                             seen(1,index)=posX;
%                             seen(2,index)=posY;
%                             seen(3,index)=x*10;
%                             seen(4,index)=y*10;
%                             seen(5,index)=2;
%                             index=index+1;
                        end;
                    end;

                    %lavy
                    if (leftDist ==-1 || -15 >= visDiff && visDiff >= -45)
                        if (obstDist <leftDist )
                             leftDist = obstDist;
%                             seen(1,index)=posX;
%                             seen(2,index)=posY;
%                             seen(3,index)=x*10;
%                             seen(4,index)=y*10;
%                             seen(5,index)=3;
%                             index=index+1;
                        end;
                    end;     
                end;
            end;
        end;
    end;
end

