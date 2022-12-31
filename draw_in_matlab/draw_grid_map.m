clc;
clear;

grid_map = load("data/global_map.txt");
%grid_map = uint8(grid_map);
figure(2);
x=0:3:500;
y=0:3:500;
mesh(x,y,grid_map);
uav = load('data/uav.txt');
uav_num=length(uav(1,:))/3;
hold on;
color ='rgbcmykrgbcmykrgbcmyk';
imshow(grid_map);
hold on;
for i=1:uav_num
    end_x = (i-1)*3+1;
    end_y = (i-1)*3+2;
    j=150;
    search_r = uav(j,(i-1)*3+3);
    plot(uav(1:j,end_x),uav(1:j,end_y),color(i));
    rectangle('Position',[uav(j,end_x)-search_r,uav(j,end_y)-search_r,2*search_r,2*search_r],'Curvature',[1,1],'EdgeColor',color(i)),axis equal;
    text(uav(j,end_x),uav(j,end_y)-1000,['uav',num2str(i),''],'color',color(i));
    hold on;
end



