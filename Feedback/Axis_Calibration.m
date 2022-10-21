
clear
close all
clc

cd('C:\Users\vtn2\Documents\MATLAB - Local\Safety Impact Feedback');


IP_UR = '169.254.152.46';
Port_Send = 30002; Port_Get = 30003;
Force_ATI.fx = 0;Force_ATI.fy = 0;Force_ATI.fz = 0;
Force_ATI.tx = 0;Force_ATI.ty = 0;Force_ATI.tz = 0;


drift = 0.0; %amount of drift
curr_thresh = 2; %current threshold in amps
blend_r = 0.1;
accel = 1000;
alpha = 20;

[UR_Send, UR_Get] = Connect_UR(IP_UR,Port_Send,Port_Get); %Connect to UR

NewAxes.q1 =-120; NewAxes.q2 =-158.77; NewAxes.q3=55.77; NewAxes.q4=-166.63; NewAxes.q5=90; NewAxes.q6 = 0;

MoveAxisBlockingUR(UR_Send,UR_Get,NewAxes,10,alpha,blend_r,0.1);

[CurAxes, CurPoses, curr, Force_Rob, Torque_Rob, Din, Dout] = GetDataUR(UR_Get);
timer = tic;ind = 1;
win = zeros(1,40);

w = [3, 0, 0, 0, 0, 0]; %Joint velocities
MoveVelAxisUR(UR_Send,w, alpha, 120);

% while (curr.q1 < (curr_thresh + drift)) && (CurAxes.q1 < -95)
while (curr.q1 < (curr_thresh + drift))
    time = toc(timer)*1e3;%Time in ms
    [CurAxes, CurPoses, curr, Force_Rob, Torque_Rob, Din, Dout] = GetDataUR(UR_Get);
    
    win = circshift(win,1);
    win(1) = curr.q1;
    curr_filt = median(win);
    
    sample(ind,:) = [time,CurAxes.q1,CurAxes.q2,CurAxes.q3,CurAxes.q4,CurAxes.q5,CurAxes.q6,...
        CurPoses.px,CurPoses.py,CurPoses.pz,CurPoses.rx,CurPoses.ry,CurPoses.rz,...
        curr.q1,curr.q2,curr.q3,curr.q4,curr.q5,curr.q6...
        Force_Rob.fx,Force_Rob.fy,Force_Rob.fz,Torque_Rob.tx,Torque_Rob.ty,Torque_Rob.tz,...
        Force_ATI.fx,Force_ATI.fy,Force_ATI.fz,Force_ATI.tx,Force_ATI.ty,Force_ATI.tz,...
        curr_filt,[strjoin(string(Din),'')],[strjoin(string(Din),'')]];
%     pause(0.01);
    ind = ind + 1;
end

StopUR(UR_Send,accel);
writematrix(sample,['Axis_',num2str(w(1)), '_.csv']);
% writematrix(sample,['data.csv']);

clear UR_Send UR_Get;

