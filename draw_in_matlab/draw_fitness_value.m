clc;
clear;

% fitness_value_aco = load("../fitness_value/fitness_value_aco.txt");%自己换路径
fitness_value_pso = load("../draw_data/最终实验数据/搜索/fitness/pso/fitness_value.txt");
% fitness_value_bottom_up = load("../draw_data/最终实验数据/time_consume/bottom_up.txt");
% fitness_value_top_down = load("../draw_data/最终实验数据/time_consume/top_down.txt");
fitness_value_dacmp = load("../draw_data/最终实验数据/搜索/fitness/dacmp/fitness_value.txt");


sim_steps = 100;%length(time_aco(:,1));

x_idx = 1:sim_steps;

figure(1);

p1 = plot(x_idx, fitness_value_dacmp(1:sim_steps, 1),'r');
hold on;
p2 = plot(x_idx, fitness_value_pso(1:sim_steps, 1),'g');
% p3 = plot(x_idx, fitness_value_bottom_up(1:sim_steps, 1),'b');
% p4 = plot(x_idx, fitness_value_top_down(1:sim_steps, 1),'y');
% p5 = plot(x_idx, fitness_value_dacmp(1:sim_steps, 1),'m');

xlable("simulation steps");
ylable("fitness value");
% h1 = legend([p1, p2, p3, p4, p5], "ACO", "PSO", "BottomUP_PSO", "TopDown_PSO", "DACMP_PSO");