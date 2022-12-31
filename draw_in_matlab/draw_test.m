clc;
clear;
cla;  

sim_times = 30;
first_row = 1200;
last_row  = 4000;
x_idx = 1:sim_times;
file_addr = '/cove_rate_PSO.txt';

aco_dir_path = 'C:/Users/win-pc/Desktop/假设检验/1109_6v1_aco/';
aco_average = zeros(1,sim_times);
for i = 1:sim_times
    file_path = [aco_dir_path,num2str(i),file_addr];
    cove_rate_data = load(file_path); 
    
    aco_average(i) = sum(cove_rate_data(first_row:last_row)) / (last_row - first_row);
end
%aco_average = sort(aco_average);
average_aco_average = linspace(sum(aco_average(1:sim_times)) / sim_times, sum(aco_average(1:sim_times)) / sim_times, sim_times);
p1 = plot(x_idx(1:sim_times), aco_average(1:sim_times),'b-');
p11 = plot(x_idx(1:sim_times), average_aco_average(1:sim_times),'b-.');
hold on;

pso_dir_path = 'C:/Users/win-pc/Desktop/假设检验/1109_6v1_pso/';
pso_average = zeros(1,sim_times);
for i = 1:sim_times
    file_path = [pso_dir_path,num2str(i),file_addr];
    cove_rate_data = load(file_path); 
    pso_average(i) = sum(cove_rate_data(first_row:last_row)) / (last_row - first_row);
end
%pso_average = sort(pso_average);
average_pso_average = linspace(sum(pso_average(1:sim_times)) / sim_times, sum(pso_average(1:sim_times)) / sim_times, sim_times);
p2 = plot(x_idx(1:sim_times), pso_average(1:sim_times),'c-');
p21 = plot(x_idx(1:sim_times), average_pso_average(1:sim_times),'c-.');

bottomup_dir_path = 'C:/Users/win-pc/Desktop/假设检验/1109_6v1_bottomup/';
bottomup_average = zeros(1,sim_times);
for i = 1:sim_times
    file_path = [bottomup_dir_path,num2str(i),file_addr];
    cove_rate_data = load(file_path); 
    bottomup_average(i) = sum(cove_rate_data(first_row:last_row)) / (last_row - first_row);
end
%bottomup_average = sort(bottomup_average);
average_bottomup_average = linspace(sum(bottomup_average(1:sim_times)) / sim_times, sum(bottomup_average(1:sim_times)) / sim_times, sim_times);
p3 = plot(x_idx(1:sim_times), bottomup_average(1:sim_times),'r-');
p31 = plot(x_idx(1:sim_times), average_bottomup_average(1:sim_times),'r-.');

topdown_dir_path = 'C:/Users/win-pc/Desktop/假设检验/1109_6v1_topdown/';
topdown_average = zeros(1,sim_times);
for i = 1:sim_times
    file_path = [topdown_dir_path,num2str(i),file_addr];
    cove_rate_data = load(file_path); 
    topdown_average(i) = sum(cove_rate_data(first_row:last_row)) / (last_row - first_row);
end
%topdown_average = sort(topdown_average);
average_topdown_average = linspace(sum(topdown_average(1:sim_times)) / sim_times, sum(topdown_average(1:sim_times)) / sim_times, sim_times);
p4 = plot(x_idx(1:sim_times), topdown_average(1:sim_times),'y-');
p41 = plot(x_idx(1:sim_times), average_topdown_average(1:sim_times),'y-.');

hybrid_dir_path = 'C:/Users/win-pc/Desktop/假设检验/1109_6v1_hybrid/';
hybrid_average = zeros(1,sim_times);
for i = 1:sim_times
    file_path = [hybrid_dir_path,num2str(i),file_addr];
    cove_rate_data = load(file_path); 
    hybrid_average(i) = sum(cove_rate_data(first_row:last_row)) / (last_row - first_row);
end
%hybrid_average = sort(hybrid_average);
average_hybrid_average = linspace(sum(hybrid_average(1:sim_times)) / sim_times, sum(hybrid_average(1:sim_times)) / sim_times, sim_times);
p5 = plot(x_idx(1:sim_times), hybrid_average(1:sim_times),'k-');
p51 = plot(x_idx(1:sim_times), average_hybrid_average(1:sim_times),'k-.');

xlabel('simulation times');
ylabel('coverage rate(%)');
legend([p3, p31, p1, p11, p2, p21, p5, p51, p4, p41], 'ACO', 'Average of ACO','PSO', 'Average of PSO','PSO-BottomUp', 'Average of PSO-BottomUp', 'PSO-TopDown', 'Average of PSO-TopDown','DACMP', 'Average of DACMP');


