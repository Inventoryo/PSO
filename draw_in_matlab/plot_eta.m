clc;
clear;

steps_wanted=2000;

coverage_rate_ds = getDataset(steps_wanted, "cove_rate_PSO.txt");
coverage_rate_ds(:,[1,3])=coverage_rate_ds(:,[3,1]);
coverage_rate_ds(:,[2,4])=coverage_rate_ds(:,[4,2]);
coverage_rate_ds(:,[1,2])=coverage_rate_ds(:,[2,1]);
coverage_rate_ds(:,[1,5])=coverage_rate_ds(:,[5,1]);

destroyed_target_num_ds = getDataset(steps_wanted, "destroyed_target_num.txt");
destroyed_target_num_ds(:,[2,3])=destroyed_target_num_ds(:,[3,2]);
plotDataset(coverage_rate_ds, "覆盖率,[%]");
plotDataset(destroyed_target_num_ds, "发现目标数量");


function [dataset] = getDataset(steps_wanted, data_name)
    dataset=zeros(9,30);
    for i=1:1:9
        eta_data_path="..\mingw_build\eta_20230911\DACMP_data_eta_0"+string(i)+"\";
        for j=1:1:30
            data_path = eta_data_path + string(j) + "\" + data_name;
            temp = load(data_path);
            dataset(10-i,j)=temp(steps_wanted);
        end
    end
    dataset=dataset';
end

function [] = plotDataset(dataset, data_name)
    figure;
    fillcolor1=[100, 85, 30]/255;
    boxplot(dataset,'Colors',fillcolor1,'width',0.3,'Whisker',100);

    xlabel("\eta");
    xticks(1:1:9);
    xticklabels({'0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9'});

    ylabel(data_name);

    boxobj = findobj(gca,'Tag','Box');
    for j=1:9
        patch(get(boxobj(j),'XData'),get(boxobj(j),'YData'),'g','FaceAlpha',0.5);
    end

end
