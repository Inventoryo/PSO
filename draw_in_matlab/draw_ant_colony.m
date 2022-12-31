clc;
clear;


while(1)
    cla;
    
    ant_colony=load('data/ants.txt');
    ant_num =(length(ant_colony(1,:))-2)/2;
    
    figure(2);
    title("uav position and target position")%БъЬт
    hold on;
    color ='rgbcmykrgbcmykrgbcmyk';
    start_j =1;
    for j =10000:length(ant_colony(:,1))
        plot (60,60);
        axis equal;
        hold on;
        plot (0,0); 
        if(ant_colony(j,1)==ant_colony(j,ant_num*2+1)&&ant_colony(j,2)==ant_colony(j,ant_num*2+2))
            start_j =j;
        end
        
        for i =1:ant_num
            plot(ant_colony(start_j:j,i*2-1),ant_colony(start_j:j,i*2),color(i));
        end
        plot(ant_colony(j,ant_num*2+1),ant_colony(j,ant_num*2+2),'*r');
        
        
        hold off;
        pause(0.0000005); 
    end
    
    
    hold off;
    pause(10);
end
