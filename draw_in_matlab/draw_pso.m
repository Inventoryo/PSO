
clc;
clear;
cla;

figure(1);
hold on;
axis equal;
plot(0,0);
plot (45000,45000);
particle_state = load('../draw_data/1102_6V0_Hybrid/particle_state.txt');
uav_state      = load('../draw_data/1102_6V0_Hybrid/uav_state.txt');
best_state     = load('../draw_data/1102_6V0_Hybrid/best_state.txt');

rectangle('Position',[30000-5000,30000-5000,10000,10000],'Curvature',[1,1],'FaceColor',[0.41176,0.41176,0.41176]),axis equal;
rectangle('Position',[15000,15000,5000,5000],'FaceColor',[0,0,0]);

last_row = 1;
search_r = 3000;
p2 = plot(particle_state(last_row,1:2:100),particle_state(last_row,2:2:100),'g*');
p1 = plot(best_state(last_row,1),best_state(last_row,2),'ko');
p3 = plot(best_state(last_row,3),best_state(last_row,4),'k*');
text(best_state(last_row,1)+1000,best_state(last_row,2)+2000,'global best','color','r');
plot(uav_state(last_row,1),uav_state(last_row,2),'ro'); 
text(uav_state(last_row,1)-2000,uav_state(last_row,2)+1000,'uav pose','color','b');

rectangle('Position',[uav_state(last_row,1)-search_r,uav_state(last_row,2) - search_r,10000,10000],'Curvature',[1,1]),axis equal;


%legend([p1,p2],'Global Best', 'SWARM');

hold off;

while(1)
    cla;
    particle_state = load('../draw_data/1102_6V0_Hybrid/particle_state.txt');
    uav_state      = load('../draw_data/1102_6V0_Hybrid/uav_state.txt');
    best_state     = load('../draw_data/1102_6V0_Hybrid/best_state.txt');
    figure(1);
    hold on
    for j =1:1:length(particle_state(:,1))
        plot(uav_state(1:j,1),uav_state(1:j,2),'ro');  
        
        hold on; 
        grid on;
        text(30000,30000,'rader','color','g');
        rectangle('Position',[30000-5000,30000-5000,10000,10000],'Curvature',[1,1]),axis equal;
        rectangle('Position',[15000,10000,5000,5000]);
        rectangle('Position',[uav_state(j,1)-search_r,uav_state(j,2)-search_r,2 * search_r, 2 * search_r],'Curvature',[1,1]),axis equal;
        plot(best_state(j,1),best_state(j,2),'ko');
        plot(best_state(j,3),best_state(j,4),'k*');
        text(best_state(j,1),best_state(j,2)+100,'current global best','color','r');
        text(uav_state(j,1)+100,uav_state(j,2),'current uav pose','color','b');
        for k=1:2:length(particle_state(j,:))
            plot(particle_state(j,k),particle_state(j,k+1),'g*');
        end
        
        hold off;
        pause(0.05);
    end
    
    title("particle position and target position")%БъЬт
    hold off;
    pause(1);
end