% ********************************************************************************************************************
% ********************************************************************************************************************
%包含当前文件夹子文件夹到工作区
addpath(genpath(pwd));

close all;
clear variables;
clear global;

%删除所有已经打开的串口，这条很重要，防止之前运行没有关闭串口
delete(instrfindall);

%打开串口COM，波特率38400，8位数据位，1位停止位，无奇偶校验，无流控制
s = serialport("COM4", 115200, "DataBits", 8, "StopBits", 1);

fig = figure(1);

% ********************************************************************************************************************
AxisMax =  65536;    %坐标轴最大值
AxisMin = -65536;    %坐标轴最小值
window_width = 800;  %窗口宽度

g_Count = 0;          %接收到的数据计数
SOF = 0;             %同步帧标志
AxisValue = 1;       %坐标值
Data_len = 0;       %本次应收数据长度
Data_CRC16 = 0;
Data_temp = zeros(1,100);   %Data读取缓冲区，一次取Data_len长做CRC
RecDataDisp = zeros(1,100000); %开辟100000个数据单元，用于存储接收到的数据。
RecData = zeros(1,100);        %开辟100个数据单元，用于数据处理。
Axis = zeros(1,100000);        %开辟100000个数据单元，用于X轴。

window = window_width * (-0.9); %窗口X轴起始坐标
axis([window, window + window_width, -inf, inf]); %设置窗口坐标范围
%子图1显示串口上传的数据
subplot(1,1,1); 
grid on;
title('串口数据接收');
xlabel('时间');
ylabel('数据');


Fs = 100;        % 采样率
N = 5;         % 采样点数
n = 0:N-1;      % 采样序列
f = n * Fs / N; %真实的频率

% ********************************************************************************************************************

while ishandle(fig)
    %设置同步信号标志， = 1表示接收到下位机发送的同步帧
    SOF = 0;  
    %发送同步帧
    encoded_str = unicode2native('$','UTF-8');
    write(s,encoded_str,'uint8');
    
    %获取是否有数据
    bytes = get(s, 'BytesAvailable');
    if bytes == 0
         bytes = 1;
    end
    
    %读取下位机返回的所有数据
    RecData = read(s, bytes, 'uint8');
   
    %检索下位机返回的数据中是否有字符$
    StartData = find(char(RecData) == '$');
    
    %如果检索到$，读取数据包字节数，为同步字符后一字节，同时将数据段暂存并进行CRC16校验，与数据包CRC比对，如有误立刻重传
    if(StartData >= 1)
        Data_len = read(s, 1, 'uint8');
        if(bytes >1 && Data_len > bytes)
            RecData = zeros(1,100);
            flush(s);
            continue
        end
        Data_temp = read(s, Data_len, 'uint8');
        Data_CRC16 = read(s, 1, 'uint16');
        [CRC6_cal,return_value] = CRC16_Verify(Data_temp,Data_CRC16);
        StartData = 0;
        if return_value == 0
            RecData = zeros(1,100);
            flush(s);
            continue
        end
    end

    %如果CRC16校验通过，读取帧头后数据段，也就是5个uint16的数据
    if(return_value == 1)
        RecData = typecast(uint8(Data_temp),'uint16');
        SOF =1;
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
	subplot(1,1,1);
	plot(Axis(1:AxisValue-1),  RecDataDisp(1:AxisValue-1), 'r');
	window = window + 5;
	axis([window, window + window_width, -inf, inf]);
	grid on;
	title('串口数据接收');
	xlabel('时间');
	ylabel('数据');
	drawnow
end
    
    %同步延迟
    pause(130/1000);
end

delete(s);
delete(instrfindall);

% ********************************************************************************************************************
