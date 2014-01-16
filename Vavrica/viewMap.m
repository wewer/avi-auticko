function viewMap(u)
       
    %pociatocne podmienky
    gcf();
    absX = u(1); 
    absY = u(2); 
    dFi =  u(3); % [rad] natocenie auticka okolo jeho taziska
    
    left   =    u(4);
    center =    u(5);
    right  =    u(6);

    refAuto     = u(7);
    auto        = u(8);
    refK1       = u(9);
    k1          = u(10);
    refK2       = u(11);
    k2          = u(12);
    refK3       = u(13);
    k3          = u(14);
   
    %[absX absY rad2deg(dFi)]
   
   gcf();
   %kuzely
   if left > -1 
    set(k1,'FaceColor','r'); 
   else
    set(k1,'FaceColor','b'); 
   end;
   
   if center > -1
       set(k2,'FaceColor','r');   
   else
      set(k2,'FaceColor','b');    
   end;
   
   if right > -1
      set(k3,'FaceColor','r');    
   else
       set(k3,'FaceColor','b'); 
   end;
   
   
   
   
   Tx_old = sum(get(auto,'XData'))/length(get(auto,'XData'));
   Ty_old = sum(get(auto,'YData'))/length(get(auto,'YData'));
   %rotacia
   objects = [auto k1 k2 k3];
   refObjects = [refAuto refK1 refK2 refK3];
   
   for i=1:1:length(objects)
    xData = get(refObjects(i),'XData');
    yData = get(refObjects(i),'YData');
    
    %translacia
    set(objects(i),'XData',xData +absX);
    set(objects(i),'YData',yData +absY);
    %pohybujuce sa ref objekty-nie je dobry napad 
%      set(refObjects(i),'XData',xData +absX);
%      set(refObjects(i),'YData',yData +absY);
%     pause(1);
   end;
  
   
   %vyznacenie trasy
   Tx = sum(get(auto,'XData'))/length(get(auto,'XData'));
   Ty = sum(get(auto,'YData'))/length(get(auto,'YData'));
   plot([Tx_old Tx],[Ty_old Ty]);
 
   %rotacia
   for i=1:1:length(objects)
       xdata = get(objects(i),'XData');
       ydata = get(objects(i),'YData');
       
       for j=1:1:length(xdata)
            x = xdata(j);
            y = ydata(j);
            xNew(j) = ((x -Tx)*cos(dFi) - (y -Ty)*sin(dFi)) +Tx;
            yNew(j) = ((x -Tx)*sin(dFi) + (y -Ty)*cos(dFi)) +Ty;
        end;
        set(objects(i),'XData',xNew);
        set(objects(i),'YData',yNew);
   end; 
end

