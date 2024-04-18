clc;
clear;
cla;

xlabel('T, [sec]','FontName', 'Times New Roman', 'FontSize', 12);
ylabel('UAV ID', 'FontName', 'Times New Roman', 'FontSize', 12);
xlim([0 2000]);

%% 20230311_10vs6_1
% ylim([0 11]);yticks(1:11);
% yticklabels('manual');yticklabels({'UAV1','UAV2','UAV3','UAV4','UAV5','UAV6','UAV7','UAV8','UAV9','UAV10'});
% 
% uav_ID    = [   1,    2,   3,   4,   5,    6,    7,    8,    9,   10];
% target_ID = [   4,    2,   3,   1,   6,    4,    5,    1,    3,    6];
% start_T   = [ 200,  250, 250, 250, 250, 1500,  300,  650   500,  450];
% duration  = [1300, 2000, 250, 400, 200, 2000, 2000, 2000, 2000, 2000];

%% 20230311_11vs6_1
ylim([0 12]);yticks(1:12);
yticklabels('manual');yticklabels({'UAV1','UAV2','UAV3','UAV4','UAV5','UAV6','UAV7','UAV8','UAV9','UAV10','UAV11'});

uav_ID    = [  1,    2,   3,   4,   5,     6,    7,    8,    9,   10,   11];
target_ID = [  4,    2,   3,   1,   6,     4,    5,    1,    3,    6,    2];
start_T   = [200,  250, 250, 2100, 250,  700,  300,  500   500,  450, 1450];
duration  = [500, 1200, 250, 2000, 200, 2000, 2000, 2000, 2000, 2000, 2000];

%% draw
n_task_nb = length(uav_ID);

rec = [0, 0, 0, 0]; %temp data space for every rectangle
colors = ["#000000" "#FF0000" "#FF00FF" "#0000FF" "#00FFFF" ...,
        "#00FF00" "#9C66EF" "#802A2A" "#A020F0" "#FF9912" "#DA70D6"];

rec_with = 0.6;
for i = 1:n_task_nb
    rec(1) = start_T(i); %矩形的横坐标
    rec(2) = uav_ID(i) - rec_with / 2.0; %矩形的纵坐标
    rec(3) = duration(i); %矩形的x轴方向的长度
    rec(4) = rec_with;

    rectangle('Position', rec, 'LineWidth', 0.5, 'LineStyle', '-', 'FaceColor', colors(target_ID(i)+1)); %draw every rectangle
    txt = sprintf('T%d', target_ID(i)); %
    text(start_T(i) + 1, uav_ID(i), txt, 'FontWeight', 'Bold', 'FontSize', 12, 'FontName', 'Times New Roman'); %label the id of every task  ，字体的坐标和其它特性
end
