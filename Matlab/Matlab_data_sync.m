% ********************************************************************************************************************
% ********************************************************************************************************************
%包含当前文件夹子文件夹到工作区
addpath(genpath(pwd));

close all;
clear variables;
clear global;

%删除所有已经打开的串口，这条很重要，防止之前运行没有关闭串口
delete(instrfindall);

%打开串口COM1，波特率38400，8位数据位，1位停止位，无奇偶校验，无流控制
s = serialport("COM9", 38400, "DataBits", 8, "StopBits", 1);
fopen(s);

fig = figure(1);

% ********************************************************************************************************************
AxisMax =  65536;    %坐标轴最大值
AxisMin = -65536;    %坐标轴最小值
window_width = 800;  %窗口宽度

g_Count =0;          %接收到的数据计数
SOF = 0;             %同步帧标志
AxisValue = 1;       %坐标值
RecDataDisp = zeros(1,100000); %开辟100000个数据单元，用于存储接收到的数据。
RecData = zeros(1,100);        %开辟100个数据单元，用于数据处理。
Axis = zeros(1,100000);        %开辟100000个数据单元，用于X轴。

window = window_width * (-0.9); %窗口X轴起始坐标
axis([window, window + window_width, -inf, inf]); %设置窗口坐标范围
%子图1显示串口上传的数据
subplot(2,1,1); 
grid on;
title('串口数据接收');
xlabel('时间');
ylabel('数据');

%子图2显示波形的幅频响应
subplot(2,1,2);
grid on;
title( 'FFT');
xlabel( '频率');
ylabel( '幅度');

Fs = 100;        % 采样率
N = 50;         % 采样点数
n = 0:N-1;      % 采样序列
f = n * Fs / N; %真实的频率

% ********************************************************************************************************************

while ishandle(fig)
    %设置同步信号标志， = 1表示接收到下位机发送的同步帧
    SOF = 0;  
    %发送同步帧
    encoded_str = unicode2native('$','UTF-8');
    fwrite(s,encoded_str);
    
    %获取是否有数据
    bytes = get(s, 'BytesAvailable');
    if bytes == 0
         bytes = 1;
    end
    
    %读取下位机返回的所有数据
    RecData = fread(s, bytes, 'uint8');
   
    %检索下位机返回的数据中是否有字符$
    StartData = find(char(RecData) == '$');
    
    %如果检索到$，读取10个字节的数据，也就是5个uint16的数据
    if(StartData >= 1)
        RecData = fread(s, 5, 'uint16');
        SOF =1;
        StartData = 0;
    end
    
%更新接收到的数据波形
if(SOF == 1)
	%更新数据
	RecDataDisp(AxisValue) =  RecData(1);
	RecDataDisp(AxisValue + 1) =  RecData(2);
	RecDataDisp(AxisValue + 2) =  RecData(3);
	RecDataDisp(AxisValue + 3) =  RecData(4);
	RecDataDisp(AxisValue + 4) =  RecData(5);
	
	%更新X轴
	Axis(AxisValue) = AxisValue;
	Axis(AxisValue + 1) = AxisValue + 1;
	Axis(AxisValue + 2) = AxisValue + 2;
	Axis(AxisValue + 3) = AxisValue + 3;
	Axis(AxisValue + 4) = AxisValue + 4;

	%更新变量
	AxisValue = AxisValue + 5;
	g_Count = g_Count + 5;
	
	%绘制波形
	subplot(2,1,1);
	plot(Axis(1:AxisValue-1),  RecDataDisp(1:AxisValue-1), 'r');
	window = window + 5;
	axis([window, window + window_width, -inf, inf]);
	grid on;
	title('串口数据接收');
	xlabel('时间');
	ylabel('数据');
	drawnow
end
    
if(g_Count== 50)
   subplot(2,1,2); 
   %对原始信号做 FFT 变换
   y = fft(RecDataDisp(AxisValue-50:AxisValue-1), 50); 
   
   %求 FFT 转换结果的模值
   Mag = abs(y)*2/N;  
   
   %绘制幅频相应曲线
   plot(f, Mag, 'r'); 
   grid on;
   title( 'FFT');
   xlabel( '频率');
   ylabel( '幅度');
   g_Count = 0;
   drawnow
end
    %同步延迟
    pause(130/1000);
end

fclose(s);
delete(s);

% ********************************************************************************************************************
