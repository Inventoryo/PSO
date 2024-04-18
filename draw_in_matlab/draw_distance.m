clc;
clear;
cla;  

colors = ["#000000"; "#FF0000"; "#FF00FF"; "#0000FF"; "#00FFFF"; ...,
          "#00FF00"; "#9C66EF"; "#802A2A"; "#A020F0"; "#FF9912"; "#DA70D6"];

data_floder = "D:\Workspace\PSO\data\20230228_3v2/";
target         = load(data_floder + "target.txt");
uav            = load(data_floder + "uav.txt");

last_idx = 1000;

distance1 = uav(1:last_idx, 1:2) - target(1:last_idx, 1:2);
distance1 = sqrt(sum(distance1.* distance1, 2));

distance2 = uav(1:last_idx, 4:5) - target(1:last_idx, 1:2);
distance2 = sqrt(sum(distance2.* distance2, 2));

distance3 = uav(1:last_idx, 7:8) - target(1:last_idx, 1:2);
distance3 = sqrt(sum(distance3.* distance3, 2));

figure(1);
handle = []
p = plot(1:1:last_idx, distance1(1:last_idx), 'color', colors(1), 'LineWidth', 1.5);
handle = [handle, p];
hold on;

p = plot(1:1:last_idx, distance2(1:last_idx), 'color', colors(2), 'LineWidth', 1.5);
handle = [handle, p];

p = plot(1:1:last_idx, distance3(1:last_idx), 'color', colors(3), 'LineWidth', 1.5);
handle = [handle, p];

legend(handle, {'U1','U2', 'U3'}, 'location', 'best', 'Orientation', 'vertical', 'NumColumns', 1, 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('T, [sec]', 'FontName', 'Times New Roman', 'FontSize', 12);
ylabel('Distance between UAV and target, [m]', 'FontName', 'Times New Roman', 'FontSize', 12);
