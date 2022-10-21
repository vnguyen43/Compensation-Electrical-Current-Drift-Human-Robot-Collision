clear
close all
clc

cd('C:\Users\vtn2\Documents\MATLAB - Local\Safety Impact Feedback');


IP_UR = '169.254.152.46';
Port_Send = 30002; Port_Get = 30003;
Force_ATI.fx = 0;Force_ATI.fy = 0;Force_ATI.fz = 0;
Force_ATI.tx = 0;Force_ATI.ty = 0;Force_ATI.tz = 0;

COM_Port = 'COM12';
baud_rate = 115200;
s = serialport(COM_Port,baud_rate);
configureTerminator(s,"CR/LF");
configureCallback(s,"terminator",@readSerialData);

curr_thresh = 1.75; %current threshold in amps
blend_r = 0.1;
% accel = 15000;
accel = 1000;
alpha = 20;
spike_ind = 0;

[UR_Send, UR_Get] = Connect_UR(IP_UR,Port_Send,Port_Get); %Connect to UR

NewAxes.q1 =-120; NewAxes.q2 =-162.73; NewAxes.q3=55.77; NewAxes.q4=-166.63; NewAxes.q5=90; NewAxes.q6 = 0;
% NewAxes.q1 =-120; NewAxes.q2 =-160.00; NewAxes.q3=55.77; NewAxes.q4=-166.63; NewAxes.q5=90; NewAxes.q6 = 0;

MoveAxisBlockingUR(UR_Send,UR_Get,NewAxes,10,alpha,blend_r,0.1);

[CurAxes, CurPoses, curr, Force_Rob, Torque_Rob, Din, Dout] = GetDataUR(UR_Get);
timer = tic;ind = 1; curr_drift = curr.q1;time = 0;speed_diff=0;speed_ant=0;speed_sum = 0;

load('Model_NN');
load('Model_Targ');
win = zeros(1,40);

w1 = 4;
targ = w1;
w = [w1, 0, 0, 0, 0, 0]; %Joint velocities
% drift = 00*1e-6; %rate of drift
% drift = 03*1e-5; %rate of drift
% drift = 06*1e-5; %rate of drift
drift = 09*1e-5; %rate of drift

MoveVelAxisUR(UR_Send, w, alpha, 45);
% while (curr_drift < (curr_thresh )) && (CurAxes.q1 < -95)

while (curr_drift < (curr_thresh ))

    time = toc(timer)*1e3;%Time in ms
    [CurAxes, CurPoses, curr, Force_Rob, Torque_Rob, Din, Dout] = GetDataUR(UR_Get);
    
    curr_drift = curr.q1 - drift*time;
    
    win = circshift(win,1);
    win(1) = curr_drift;
    curr_filt = median(win);
        
    if time > 900
        speed_ant = predict(mdl,[(time*1e-3),curr_filt,CurAxes.q1]);
        if spike_ind > 150 %Drift 0, 30
            speed_diff = targ-speed_ant*1; %Drift 0, 30
            w1 = targ + speed_diff*5 + 0.005*speed_sum %Drift 0, 30
%         if spike_ind > 0 %Drift 60, 90
%             speed_diff = targ-speed_ant*.5; %Drift 90
%             speed_diff = targ-speed_ant*.6; %Drift 60
%             w1 = targ + speed_diff*1 + 0.005*speed_sum %Drift 60, 90

            speed_sum = speed_diff+speed_sum;
        end
        spike_ind = spike_ind + 1;
    end
    
%     w = [w1, 0, 0, 0, 0, 0];
    
    safety_lim = 8;
    if abs(w(1)) > safety_lim || w(1)<0
       w = [safety_lim, 0, 0, 0, 0, 0];
    end
    
    MoveVelAxisUR(UR_Send, w, alpha, 45);
        
    sample(ind,:) = [time,CurAxes.q1,CurAxes.q2,CurAxes.q3,CurAxes.q4,CurAxes.q5,CurAxes.q6,...
        CurPoses.px,CurPoses.py,CurPoses.pz,CurPoses.rx,CurPoses.ry,CurPoses.rz,...
        curr.q1,curr.q2,curr.q3,curr.q4,curr.q5,curr.q6...
        Force_Rob.fx,Force_Rob.fy,Force_Rob.fz,Torque_Rob.tx,Torque_Rob.ty,Torque_Rob.tz,...
        Force_ATI.fx,Force_ATI.fy,Force_ATI.fz,Force_ATI.tx,Force_ATI.ty,Force_ATI.tz,...
        curr_filt,curr_drift,w1,drift*time,speed_diff,speed_ant,s.UserData,[strjoin(string(Din),'')],[strjoin(string(Din),'')]];
    ind = ind + 1;
end

StopUR(UR_Send,accel);

% pause(1)

for k=1:1:200
        time = toc(timer)*1e3;%Time in ms
        [CurAxes, CurPoses, curr, Force_Rob, Torque_Rob, Din, Dout] = GetDataUR(UR_Get);

        sample(ind,:) = [time,CurAxes.q1,CurAxes.q2,CurAxes.q3,CurAxes.q4,CurAxes.q5,CurAxes.q6,...
        CurPoses.px,CurPoses.py,CurPoses.pz,CurPoses.rx,CurPoses.ry,CurPoses.rz,...
        curr.q1,curr.q2,curr.q3,curr.q4,curr.q5,curr.q6...
        Force_Rob.fx,Force_Rob.fy,Force_Rob.fz,Torque_Rob.tx,Torque_Rob.ty,Torque_Rob.tz,...
        Force_ATI.fx,Force_ATI.fy,Force_ATI.fz,Force_ATI.tx,Force_ATI.ty,Force_ATI.tz,...
        curr_filt,curr_drift,w1,drift*time,speed_diff,speed_ant,s.UserData,[strjoin(string(Din),'')],[strjoin(string(Din),'')]];
    ind = ind + 1;
    pause(0.01);
end


writematrix(sample,'data.csv');
clear UR_Send UR_Get s;

sample(end,2)
abs(mean(str2double(sample(end-100:end,38)))-mean(str2double(sample(1:500,38))))
abs(mean(str2double(sample(end-100:end,38))))

figure;
subplot(231);plot(str2double(sample(:,1))*1e-3,str2double(sample(:,33)),'linewidth',2);title('curr drift');
set(gca,'fontsize',20,'fontname','Arial','Linewidth',2);
subplot(232);plot(str2double(sample(:,1))*1e-3,str2double(sample(:,32)),'linewidth',2);title('curr filtered')
set(gca,'fontsize',20,'fontname','Arial','Linewidth',2);
subplot(233);plot(str2double(sample(:,1))*1e-3,str2double(sample(:,14)),'linewidth',2);title('raw');
set(gca,'fontsize',20,'fontname','Arial','Linewidth',2);
subplot(234);plot(str2double(sample(:,1))*1e-3,str2double(sample(:,34)),'linewidth',2);title('command');
set(gca,'fontsize',20,'fontname','Arial','Linewidth',2);
subplot(235);plot(str2double(sample(:,1))*1e-3,str2double(sample(:,36)),'linewidth',2);title('error');
set(gca,'fontsize',20,'fontname','Arial','Linewidth',2);
subplot(236);plot(str2double(sample(:,1))*1e-3,str2double(sample(:,37)),'linewidth',2);title('anticipated error');
set(gca,'fontsize',20,'fontname','Arial','Linewidth',2);

figure;plot(str2double(sample(:,38)))


function readSerialData(src,~)
    data = readline(src);
    src.UserData = str2double(data);
end