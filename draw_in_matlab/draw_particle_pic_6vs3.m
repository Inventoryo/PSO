
clc;
clear;
cla;  

while(1)
%     cla;
    data_floder = "./mingw_build/draw_data/20230214_11VS6_1/";
    target         = load(data_floder + "target.txt");
    uav            = load(data_floder + "uav.txt");
    Gbest_position = load(data_floder + "traj_Point.txt");
    area_Point     = load(data_floder + "area_Point.txt");

    total_uav_num = length(uav(1, :)) / 3;
    total_target_num = length(target(1, :)) / 2;
    
    colors = ["#000000"; "#FF0000"; "#FF00FF"; "#0000FF"; "#00FFFF"; ...,
              "#00FF00"; "#9C66EF"; "#802A2A"; "#A020F0"; "#FF9912"; "#DA70D6"];
    t_z = ones(length(uav(:,1))) * 0;
    u_low_z = ones(length(uav(:,1))) * 10000;
    u_high_z = ones(length(uav(:,1))) * 40000;
    
    for j = 420:length(uav(:,1))
        plot3(0,0,0);hold on;grid on;
        rectangle('Position',[15000-2000,15000-2000,4000,4000],'Curvature',[1,1],'FaceColor', [0.41176,0.41176,0.41176]),axis equal;
        rectangle('Position',[30000-3000,30000-3000,6000,6000],'Curvature',[1,1],'FaceColor', [0.41176,0.41176,0.41176]),axis equal;
        rectangle('Position',[20000-3000,40000-3000,6000,6000],'Curvature',[1,1],'FaceColor', [0.41176,0.41176,0.41176]),axis equal;
        rectangle('Position',[40000-4000,20000-4000,8000,8000],'Curvature',[1,1],'FaceColor', [0.41176,0.41176,0.41176]),axis equal;
        xlabel('X, [km]');
        xlim([0 50000]);xticks([0 10000 20000 30000 40000 50000]);
        xticklabels('manual');xticklabels({'0','10','20','30','40','50'});
        ylabel('Y, [km]');
        ylim([0 50000]);yticks([0 10000 20000 30000 40000 50000]);
        yticklabels('manual');yticklabels({'0','10','20','30','40','50'});
        zlabel('Z, [m]');
        zlim([0 20000]);zticks([0 10000 20000]);
        zticklabels('manual');zticklabels({'0','400','800'});
        for i=1:total_uav_num
            end_x = (i-1)*3+1;
            end_y = (i-1)*3+2;
            search_r = uav(j,(i-1)*3+3);
            
            if search_r==3000
                scale = 1;
            else
                scale = 2;
            end

            plot3(uav(1:j,end_x), uav(1:j,end_y), u_low_z(1:j) * scale, 'color', colors(i), 'LineWidth', 2);
            text(uav(j,end_x), uav(j,end_y)-1000, u_low_z(1) * scale , ['U',num2str(i),''], 'color', colors(i));

        end     
        for i=1:total_target_num
            end_x = (i-1)*2+1;
            end_y = (i-1)*2+2;
            
            plot3(target(1:j,end_x), target(1:j,end_y), t_z(1:j), 'color', colors(i), 'LineStyle', '--', 'LineWidth', 1);
            text(target(j,end_x), target(j,end_y)+1000, 0, ['T',num2str(i),''], 'color',colors(i));
        end    
        hold off;
        pause(0.0000005); 
    end
    
    
    hold off;
    pause(10);
end




    
