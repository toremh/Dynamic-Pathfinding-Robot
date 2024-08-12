clear;

x1=0;
x2=1200;
y1=0;
y2=2000;
x = [x1, x2, x2, x1, x1];
y = [y1, y1, y2, y2, y1];
debugData = zeros(1,50000);
A = plot(x, y, 'LineWidth', 3);
axis equal;
hold on;
data_array = [1 2 3];
duration = 0.39;

device = serialport("COM15",115200);
for a = 1:50000
    try
        data = readline(device);
    catch 
        warning('No Data');
    end
    
    disp(data);
    if (isempty(data) == 0)
        data_array = split(data, ' ');
        data_array = str2double(data_array);
    end

% %     data_array(1) = data_array(1) + 1;
% %     data_array(2) = data_array(2) + 1;
    set(A,'XData', [get(A, 'XData') data_array(2)], 'YData', [get(A, 'YData') data_array(1)]);
    debugData(a) = data_array(3);
    drawnow;
    java.lang.Thread.sleep(duration*1000)
end
