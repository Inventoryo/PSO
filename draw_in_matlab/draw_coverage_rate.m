clc;
clear;
cla;  

colors = ["#000000"; "#FF0000"; "#FF00FF"; "#0000FF"; "#00FFFF"; ...,
          "#00FF00"; "#9C66EF"; "#802A2A"; "#A020F0"; "#FF9912"; "#DA70D6"];

% cove_rate_dacmp  = load('D:\Workspace\PSO\data\20230514_6vs15\DACMP_data/cove_rate_PSO.txt'); 
% cove_rate_greedy = load('D:\Workspace\PSO\data\20230514_6vs15\greedy_data/cove_rate_PSO.txt'); 
cove_rate_dacmp  = load('D:\Workspace\PSO\data\20230311_10and11vs6\DACMP_data_10vs6_1/cove_rate_PSO.txt'); 
cove_rate_greedy = load('D:\Workspace\PSO\data\20230311_10and11vs6\DACMP_data_11vs6_1/cove_rate_PSO.txt'); 

x_idx = 1:10000;
figure(1);
first_row = 1;
last_row  = 2000;


handle = []
p = plot(x_idx(first_row:last_row), cove_rate_dacmp(first_row:last_row), 'color', colors(1), 'LineWidth', 1.5);
hold on
xlabel('T, [sec]', 'FontName', 'Times New Roman', 'FontSize', 12);
ylabel('Coverage rate, [%]', 'FontName', 'Times New Roman', 'FontSize', 12);
handle = [handle, p];
p = plot(x_idx(first_row:last_row), cove_rate_greedy(first_row:last_row), 'color', colors(2), 'LineWidth', 1.5);
handle = [handle, p];

legend(handle, {'10UAV','11UAV'}, 'location', 'best', 'Orientation', 'vertical', 'NumColumns', 1, 'FontName', 'Times New Roman', 'FontSize', 12);
% legend(handle, {'DACMP','GREEDY'}, 'location', 'best', 'Orientation', 'vertical', 'NumColumns', 1, 'FontName', 'Times New Roman', 'FontSize', 12);

hold off;

%% ---30 times--- %%
figure(2);
handle = [];

%% dacmp
cove_rate_dacmp_2000 = zeros(1, 30);
for i=0:1:29
    temp = load("D:\Workspace\PSO\data\20230313_30times\DACMP_data_" + i + "\cove_rate_PSO.txt");
    cove_rate_dacmp_2000(i+1) = temp(2000);
end
p = plot(x_idx(1:30), cove_rate_dacmp_2000(1:30), 'color', colors(1), 'LineWidth', 1.5);
hold on;
handle = [handle, p];

average = sum(cove_rate_dacmp_2000) / 30;
average_V = linspace(average, average, 30);
p = plot(x_idx(1:30), average_V(1:30), 'color', colors(1), 'LineWidth', 1.5);
handle = [handle, p];

%% greedy
cove_rate_greedy_2000 = zeros(1, 30);
for i=0:1:29
    temp = load("D:\Workspace\PSO\data\20230313_30times\greedy_data_" + i + "\cove_rate_PSO.txt");
    cove_rate_greedy_2000(i+1) = temp(2000);
end
p = plot(x_idx(1:30), cove_rate_greedy_2000(1:30), 'color', colors(2), 'LineWidth', 1.5);
handle = [handle, p];

average = sum(cove_rate_greedy_2000) / 30;
average_V = linspace(average, average, 30);
p = plot(x_idx(1:30), average_V(1:30), 'color', colors(2), 'LineWidth', 1.5);
handle = [handle, p];

legend(handle, {'DACMP','Mean of DACMP','GREEDY','Mean of GREEDY'}, 'location', 'best', 'Orientation', 'vertical', 'NumColumns', 1, 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('T, [sec]', 'FontName', 'Times New Roman', 'FontSize', 12);
ylabel('Coverage rate, [%]', 'FontName', 'Times New Roman', 'FontSize', 12);
hold off;
