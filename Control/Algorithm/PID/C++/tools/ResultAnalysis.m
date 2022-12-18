close all;
clear;
clc;

[ReadFileName,ReadFilePathName,ReadFilterIndex] = uigetfile('*.txt;*.log','获取txt或者log','');
str = [ReadFilePathName ReadFileName];
fid = fopen(str,'r');

ctrlIndex = 1;
RefTra_Index = 1;
RefPoi_Index = 1;
TraErr_Index = 1;
ConCom_Index = 1;
StaRob_Index = 1;

RefTra_Ide = '%s %s %f %s %f %s %f %s %f %s %f %s %f %s %f';
TraErr_Ide = '%s %s %f %s %f %s %f %s %f %s %f';
ConCom_Ide = '%s %s %s %s %f %s %f %s %f';
StaRob_Ide = '%s %s %f %s %f %s %f %s %f %s %f %s %f %s %f';

i=0;
while ~feof(fid)
    i = i+1;

    t1 = fgetl(fid);

    id_MOTOR = strfind(t1,'[reference_trajectory]');
    if ~isempty(id_MOTOR)
        data = textscan(t1,RefTra_Ide);
        RefTra(RefTra_Index,(1:12)) = data(1,(4:15));
        RefTra_Index = RefTra_Index + 1;
    end

    id_MOTOR = strfind(t1,'[tracking_error]');
    if ~isempty(id_MOTOR)
        data = textscan(t1,TraErr_Ide);
        TraErr(TraErr_Index,(1:8)) = data(1,(4:11));
        TraErr_Index = TraErr_Index + 1;
    end

    id_MOTOR = strfind(t1,'[control_command]');
    if ~isempty(id_MOTOR)
        data = textscan(t1,ConCom_Ide);
        ConCom(ConCom_Index,(1:6)) = data(1,(4:9));
        ConCom_Index = ConCom_Index + 1;
    end

    id_MOTOR = strfind(t1,'[state_of_robot]');
    if ~isempty(id_MOTOR)
        data = textscan(t1,StaRob_Ide);
        StaRob(StaRob_Index,(1:12)) = data(1,(4:15));
        StaRob_Index = StaRob_Index + 1;
    end
end

fprintf('%s 读取完成\n',ReadFileName);
fclose(fid);

figure('name','仿真结果');
subplot(3,2,1);
plot(cell2mat(RefTra(:,2)),cell2mat(RefTra(:,4)),'r',cell2mat(StaRob(:,2)),cell2mat(StaRob(:,4)),'b'); grid on;
xlabel('横向位置(米)'); ylabel('纵向位置(米)'); title('运动轨迹'); legend('规划轨迹','真实轨迹');
subplot(3,2,2);
plot(cell2mat(RefTra(:,12)),cell2mat(RefTra(:,8)),'r',cell2mat(StaRob(:,12)),cell2mat(StaRob(:,8)),'b'); grid on;
xlabel('时间(秒)'); ylabel('速度(米/秒)'); title('速度-时间曲线'); legend('规划速度','真实速度');
subplot(3,2,3);
plot(cell2mat(TraErr(:,8)),cell2mat(TraErr(:,2))*1000); grid on;
xlabel('时间(秒)'); ylabel('轨迹系纵向误差(毫米)'); title('轨迹系纵向误差-时间曲线');
subplot(3,2,4);
plot(cell2mat(TraErr(:,8)),cell2mat(TraErr(:,4))*1000); grid on;
xlabel('时间(秒)'); ylabel('轨迹系横向误差(毫米)'); title('轨迹系横向误差-时间曲线');
subplot(3,2,5);
plot(cell2mat(ConCom(:,6)),cell2mat(ConCom(:,2)),'r',cell2mat(RefTra(:,12)),cell2mat(RefTra(:,8)),'b',cell2mat(StaRob(:,12)),cell2mat(StaRob(:,8)),'g'); grid on;
xlabel('时间(秒)'); ylabel('速度(米/秒)'); title('速度-时间曲线'); legend('速度指令','规划速度','真实速度');
subplot(3,2,6);
plot(cell2mat(ConCom(:,6)),cell2mat(ConCom(:,4))*57.3,'r',cell2mat(RefTra(:,12)),cell2mat(RefTra(:,10))*57.3,'b',cell2mat(StaRob(:,12)),cell2mat(StaRob(:,10))*57.3,'g'); grid on;
xlabel('时间(秒)'); ylabel('横摆角速度(度/秒)'); title('横摆角速度-时间曲线'); legend('横摆角速度指令','规划横摆角速度','真实横摆角速度');