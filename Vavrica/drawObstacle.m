function [ fig ] = drawObstacle( fig,x,y,color)
    figure(fig); 
    axis([0 1000 0 1000]);
    patch([x-5 x+5 x+5 x-5],[y-5 y-5 y+5 y+5],color); hold on;
end

