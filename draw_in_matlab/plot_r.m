clc;
clear;

steps_wanted=2000;

coverage_rate_ds = getDataset(steps_wanted, "cove_rate_PSO.txt");
destroyed_target_num_ds = getDataset(steps_wanted, "destroyed_target_num.txt");

plotDataset(coverage_rate_ds, "覆盖率,[%]");
plotDataset(destroyed_target_num_ds, "发现目标数量");


function [dataset] = getDataset(steps_wanted, data_name)
    dataset=zeros(3,30);
    for i=1:1:3
        eta_data_path="..\mingw_build\r_20231106\DACMP_data_r_"+string(i)+"\";
        for j=1:1:30
            data_path = eta_data_path + string(j) + "\" + data_name;
            temp = load(data_path);
            dataset(i,j)=temp(steps_wanted);
        end
        mean(dataset(i,:))
    end
    dataset=dataset';
end

function [] = plotDataset(dataset, data_name)
    figure;
    fillcolor1=[100, 85, 30]/255;
    boxplot(dataset,'Colors',fillcolor1,'width',0.3,'Whisker',100);

    xlabel("探测半径，[km]");
    xticks(1:1:3);
    xticklabels({'0.5','1','2'});

    ylabel(data_name);

    boxobj = findobj(gca,'Tag','Box');
    for j=1:3
        patch(get(boxobj(j),'XData'),get(boxobj(j),'YData'),'g','FaceAlpha',0.5);
    end

end
