clc;
clear;


pso = load("../draw_data/时间分析/pso.txt");
topdown = load("../draw_data/时间分析/top_down.txt");
bottomup = load("../draw_data/时间分析/bottom_up.txt");
hybrid = load("../draw_data/时间分析/hybrid.txt");
aco = load("../draw_data/时间分析/aco.txt");

first_row = 1;
last_row = 30;
sim_steps = 30;

x_idx = 1:30;

figure(1);

average = sum(pso(1:30)) / 30;
average_V1 = linspace(average, average, last_row);
average = sum(topdown(1:30)) / 30;
average_V2 = linspace(average, average, last_row);
average = sum(bottomup(1:30)) / 30;
average_V3 = linspace(average, average, last_row);
average = sum(hybrid(1:30)) / 30;
average_V4 = linspace(average, average, last_row);
average = sum(aco(1:30)) / 30;
average_V5 = linspace(average, average, last_row);


p1 = plot(x_idx, pso(1:sim_steps, 1),'c');
hold on;
p2 = plot(x_idx, topdown(1:sim_steps, 1),'k');
p3 = plot(x_idx, bottomup(1:sim_steps, 1),'b');
p4 = plot(x_idx, hybrid(1:sim_steps, 1),'y');
p5 = plot(x_idx, aco(1:sim_steps, 1),'r');

p11 = plot(x_idx(first_row:last_row), average_V1(first_row:last_row),'c-.');
p21 = plot(x_idx(first_row:last_row), average_V2(first_row:last_row),'k-.');
p31 = plot(x_idx(first_row:last_row), average_V3(first_row:last_row),'b-.');
p41 = plot(x_idx(first_row:last_row), average_V4(first_row:last_row),'y-.');
p51 = plot(x_idx(first_row:last_row), average_V5(first_row:last_row),'r-.');

% xlable('simulation times');
% ylable('simulation steps');
% h1 = legend([p1, p2, p3, p4, p5], "PSO",  "TopDown_PSO", "BottomUP_PSO","DACMP_PSO","ACO");
legend([p1, p11, p2, p21, p3, p31, p4, p41, p5, p51], 'PSO','Average of PSO', 'PSO-TopDown', 'Average of PSO-TopDown','PSO-BottomUp', 'Average of PSO-BottomUp','DACMP', 'Average of DACMP','ACO', 'Average of ACO');