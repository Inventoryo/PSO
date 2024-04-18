clc;
clear;
cla;  

colors = ["#000000"; "#FF0000"; "#FF00FF"; "#0000FF"; "#00FFFF"; ...,
          "#00FF00"; "#9C66EF"; "#802A2A"; "#A020F0"; "#FF9912"; "#DA70D6"];

tracked_num_dacmp  = load("D:\Workspace\PSO\data\20230514_6vs15\DACMP_data/destroyed_target_num.txt");
tracked_num_greedy = load("D:\Workspace\PSO\data\20230514_6vs15\greedy_data/destroyed_target_num.txt");

sim_steps = 2000;
x_idx = 1:10000;%length(cunt);

figure(1);
handle = []
p = plot(x_idx(1:sim_steps), tracked_num_dacmp(1:sim_steps), 'color', colors(1), 'LineWidth', 1.5);
handle = [handle, p];
hold on;
p = plot(x_idx(1:sim_steps), tracked_num_greedy(1:sim_steps), 'color', colors(2), 'LineWidth', 1.5);
handle = [handle, p];

legend(handle, {'DACMP','GREEDY'}, 'location', 'best', 'Orientation', 'vertical', 'NumColumns', 1, 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('T, [sec]', 'FontName', 'Times New Roman', 'FontSize', 12);
ylabel('Number of target searched', 'FontName', 'Times New Roman', 'FontSize', 12);
hold off;

%%---30 times---%%
figure(2);
handle = [];

%% dacmp
tracked_num_dacmp_2000 = zeros(1, 30);
for i=0:1:29
    temp = load("D:\Workspace\PSO\data\20230313_30times\DACMP_data_" + i + "\destroyed_target_num.txt");
    tracked_num_dacmp_2000(i+1) = temp(2000);
end
p = plot(x_idx(1:30), tracked_num_dacmp_2000(1:30), 'color', colors(1), 'LineWidth', 1.5);
hold on;
handle = [handle, p];

average = sum(tracked_num_dacmp_2000) / 30;
average_V = linspace(average, average, 30);
p = plot(x_idx(1:30), average_V(1:30), 'color', colors(1), 'LineWidth', 1.5);
handle = [handle, p];

%% greedy
tracked_num_greedy_2000 = zeros(1, 30);
for i=0:1:29
    temp = load("D:\Workspace\PSO\data\20230313_30times\greedy_data_" + i + "\destroyed_target_num.txt");
    tracked_num_greedy_2000(i+1) = temp(2000);
end
p = plot(x_idx(1:30), tracked_num_greedy_2000(1:30), 'color', colors(2), 'LineWidth', 1.5);
handle = [handle, p];

average = sum(tracked_num_greedy_2000) / 30;
average_V = linspace(average, average, 30);
p = plot(x_idx(1:30), average_V(1:30), 'color', colors(2), 'LineWidth', 1.5);
handle = [handle, p];

legend(handle, {'DACMP','Mean of DACMP','GREEDY','Mean of GREEDY'}, 'location', 'best', 'Orientation', 'vertical', 'NumColumns', 1, 'FontName', 'Times New Roman', 'FontSize', 10);
xlabel('T, [sec]', 'FontName', 'Times New Roman', 'FontSize', 12);
ylabel('Number of target searched', 'FontName', 'Times New Roman', 'FontSize', 12);
hold off;

