clear

set(0,'defaultfigurecolor','w') 
fid=fopen('fushion_map.txt','rb');%读取'r'
map = [];

tline = fgetl(fid);
strsplit(tline, ',');
while ischar(tline)
    % disp(tline);
    tmp = strsplit(tline, ' ');
    tmp_line = zeros(1, size(tmp, 2) -1 );
    for i = 1 : (size(tmp, 2) - 1)
        tmp_line(i) = str2num(tmp{i});
    end      
    map = [map; tmp_line];
    tline = fgetl(fid);
end
fclose(fid);

prob_map = map;
% prob_map = impyramid(prob_map, 'reduce'); 
% prob_map = impyramid(prob_map, 'reduce'); 

L = size(prob_map, 1);
[xx, yy] = meshgrid(1 : 1 : L, 1 : 1 : L);
figure;
pcolor(xx, yy, prob_map); shading interp
h = gca;
h.YTickLabel={};
h.XTickLabel={};
