% ********************************************************************************************************************
% ********************************************************************************************************************
%包含当前文件夹子文件夹到工作区
addpath(genpath(pwd));

close all;
clear variables;
clear global;
clear workspace;

%删除所有已经打开的串口，这条很重要，防止之前运行没有关闭串口
delete(instrfindall);
global getdataflag;
%打开串口COM，波特率38400，8位数据位，1位停止位，无奇偶校验，无流控制
s = serialport("COM4", 115200, "DataBits", 8, "StopBits", 1);
configureCallback(s,"byte",124,@SerialDataAvailable);

% fig = figure(1);

% ********************************************************************************************************************
% AxisMax =  65536;    %坐标轴最大值
% AxisMin = -65536;    %坐标轴最小值
% window_width = 800;  %窗口宽度

g_Count = 0;          %接收到的数据计数
SOF = 0;%同步帧标志
return_value = 0;
bytes =0;
StartData = 0;
i=0;
AxisValue = 1;       %坐标值
Data_len = 0;       %本次应收数据长度
Data_CRC16 = 0;
Mag_X=double.empty;
Mag_Y=double.empty;
Mag_Z=double.empty;
% Data_temp = zeros(1,100000);   %Data读取缓冲区，一次取Data_len长做CRC
% RecDataDisp = zeros(1,100000); %开辟100000个数据单元，用于存储接收到的数据。
% RecDataRaw = zeros(1,100000);        %开辟100个数据单元，用于数据处理。
% Axis = zeros(1,100000);        %开辟100000个数据单元，用于X轴。


Fs = 100;        % 采样率
N = 5;         % 采样点数
n = 0:N-1;      % 采样序列
f = n * Fs / N; %真实的频率

% ********************************************************************************************************************
getdataflag =0;
while 1
    %设置同步信号标志， = 1表示接收到下位机发送的同步帧
    clearvars SOF return_value bytes StartData Data_len i RecData RecDataRaw Data_temp PackNum TrueNum u i j;
    SOF = 0;
    return_value = 0;
    bytes =0;
    StartData = 0;
    Data_len =0;
    i=0;
    TrueNum=int32.empty;
    %发送同步帧
    encoded_str = unicode2native('$','UTF-8');
    write(s,encoded_str,'uint8');
    %获取是否有数据
    while (getdataflag == 0)
        i = i+1;
        pause(1/1000);
        if(i>15)
            encoded_str = unicode2native('$','UTF-8');
            write(s,encoded_str,'uint8');
            i =0;
        end
    end
    getdataflag =0;
    bytes = get(s, 'BytesAvailable');
    if(bytes<124)
        continue
    end
    %读取下位机返回的所有数据
    RecDataRaw = read(s, bytes, 'uint8');
    %检索下位机返回的数据中是否有字符$
    StartData = find(char(RecDataRaw) == '$');
    PackNum=size(StartData,2);
    if((fix(bytes/124)<=PackNum) && (PackNum>2))
        if (size(StartData,2)>2)
            for i = 1:size(StartData,2)
                for j = i:size(StartData,2)
                    if(abs(StartData(i)-StartData(j))>= 123)
                        TrueNum(end+1)=i;
                        if(abs(StartData(i)-StartData(j))==124)
                            TrueNum(end+1)=j;
                        end
                    end
                end
            end
            if(isempty(TrueNum)&&bytes==124)
                TrueNum(1)=1;
            end
            TrueNum=unique(TrueNum);
            StartData=StartData(TrueNum);
        end
    end
    for u = size(StartData,2):-1:1
        if(abs(StartData(u)-bytes)<123)
            StartData(u)=[];
        end
    end
    if(~isempty(StartData))
        for n = size(StartData,2):-1:1
            if(RecDataRaw(StartData(n)+1)~=120)
                StartData(n)=[];
            end
        end
    end
    if(isempty(StartData))
        RecDataRaw = zeros(1,100000);
        flush(s);
        continue
    end
    PackNum=size(StartData,2);

    for PackIndex = 1:PackNum
        SOF =0;
        %如果检索到$，读取数据包字节数，为同步字符后一字节，同时将数据段暂存并进行CRC16校验，与数据包CRC比对，如有误立刻重传
        if(StartData(PackIndex) >= 1)
            Data_len = RecDataRaw(StartData(PackIndex)+1);
            if(bytes >1 && Data_len > bytes)
                if(PackNum ==1)
                RecDataRaw = zeros(1,100000);
                flush(s);
                end
                continue
            end
            Data_temp = RecDataRaw(StartData(PackIndex)+2:StartData(PackIndex)+1+Data_len);
            Data_CRC16 = typecast(uint8(RecDataRaw(StartData(PackIndex)+1+Data_len+1:StartData(PackIndex)+1+Data_len+2)),'uint16');
            [CRC16_cal,return_value] = CRC16_Verify(Data_temp,Data_CRC16);
            if return_value == 0
                if(PackNum ==1)
                RecDataRaw = zeros(1,100000);
                flush(s);
                end
                continue
            end
        end

        %如果CRC16校验通过，读取帧头后数据段，也就是3*10个float_32的数据
        if(return_value == 1)
            RecData = typecast(uint8(Data_temp),'single');
            SOF =1;
        end

        %更新接收到的数据波形
        if(SOF == 1)
            %更新数据
            Mag_X=[Mag_X RecData(1:3:end-2)];
            Mag_Y=[Mag_Y RecData(2:3:end-1)];
            Mag_Z=[Mag_Z RecData(3:3:end)];
            scatter3(RecData(1:3:end-2),RecData(2:3:end-1),RecData(3:3:end));
            hold on;
            if(PackIndex==1)
            %同步延迟
            pause(25/1000);
            end
        end
    end
end

delete(s);
delete(instrfindall);

% ********************************************************************************************************************
function SerialDataAvailable (s,~)
global getdataflag;
getdataflag = 1;
end