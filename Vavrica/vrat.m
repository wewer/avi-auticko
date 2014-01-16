%vseobecne pre mapu m x n
%vyhodnoti co vidia ultrazvuky
%vracia vzdialenost v akej vidi pravy, sredny a lavy sonar prekazku. Dosah = pol metra = 5 policok
%Vzdy vracia na kazdom senzore iba najblizsiu prekazku
%
%dist = 9999 -> dany senzor nevidi prekazku
%rozsahy oproti natoceniu robota:
%lavy ultrazvuk:        +45 - +15 stupnov
%stredny ultrazvuk:     +15 - -15 stupnov
%pravy ultrazvuk:       -15 - -45 stupnov
%
%
%map - mapa (mriezka m x n) v mierke 1:10 
%xPos -xova suradnica robota
%yPos - ylonova suradnica robota
%uhol natocenia robota - 0 stupnov = doprava

function [leftDist,centerDist,rightDist] = vrat(map,xPos,yPos,angle )%seen-ako vystupny parameter
%     seen=[];
    leftDist = 9999;
    centerDist = 9999;
    rightDist = 9999;
    [rows,columns] = size(map);
    %visDiff = 0;

    xFrom = round(xPos/10)-6;
    yFrom = round(yPos/10)-6;
%     xx = round(xPos/10);
%     C = xx+6;
%     yTo = round(yPos/10)+6;
    xx = (xPos/10)+6;
    C = xx;
   
    C =7;
     C = C;
    yTo = round(yPos/10)+6;

    if (xFrom <1)
        xFrom =1;
    end;

    if (yFrom <1)
        yFrom =1;
    end;

    if (C>columns )
        C =columns;
    end;

    if (yTo > rows)
        yTo = rows;
    end;
    coder.extrinsic('-sync:on', 'rad2deg');

    for x = xFrom:1:C
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
                deg = rad2deg(asin(abs(dy)/sqrt(dx^2+dy^2)))
               obstacleAngle = deg+modif; % uhol prekazky voci osi x od auticka
               
                %zorny uhol pod ktorym auticko vidi prekazku
                visDiff = angle-obstacleAngle;
                %visDiff = visAngle-angle;
                
                obstDist = sqrt(dx^2+dy^2);
                obstSeen=0;
                
                if (obstDist < 50)
                    %pravy
                    if (45 >= visDiff && visDiff >15)
                        if (obstDist <rightDist )
                            rightDist = obstDist;
                            
                        end;
                         obstSeen=1;
                    end; 

                    %stredny
                    if (15 >= visDiff && visDiff > -15)
                        if (obstDist <centerDist )
                            centerDist = obstDist;
                        end;
                         obstSeen=1;
                    end;

                    %lavy
                    if (-15 >= visDiff && visDiff >= -45)
                        if (obstDist <leftDist )
                            leftDist = obstDist;
                        end;
                        obstSeen=1;
                    end;
                
%                     if (obstSeen ==1)
%                         [xPos,yPos,x*10,y*10,angle,obstacleAngle,visDiff];
%                         [r,c] = size(seen);
%                         seen(1,c+1) = x;
%                         seen(2,c+1) = y;
%                     end;
                    
                end;
            end;
        end;
    end;
end

