% a1=-89.576;%对中台RPY旋转A值
% b1=-29.967;%对中台RPY旋转B值
% c1=41.113;%对中台RPY旋转C值
%% 添加路径
% addpath(genpath('./Core'));%添加搜索路径
% addpath(genpath('Examples/FromARTE'));%添加搜索路径
regul=input('Using regulation? [y/n] ','s');
prob.rob = RobViz('ER80_2600');

ps=PseudoOptimal;
ps.npts=12;
ps.nS=12;
ps.nU=6;
maxiter=10000;dispLev=5;
switch regul
    case 'y'
        ps=ps.timeOptimalInit(@(Xc,Uc,D,nS,nU,scale,P)conEq(Xc,Uc,D,nS,nU,scale,P,prob.rob),...
            @(Xc,Uc,tc,D,nS,nU,ww,scale)costFcn(Xc,Uc,tc,D,nS,nU,ww,scale,prob.rob),...
            @(Xc,Uc,D,scale)conIneq(Xc,Uc,D,scale),maxiter,dispLev);
    case 'n'
        ps=ps.timeOptimalInit(@(Xc,Uc,D,nS,nU,scale,P)conEq(Xc,Uc,D,nS,nU,scale,P,prob.rob),...
            @(Xc,Uc,tc,D,nS,nU,ww,scale)costFcn_topt(Xc,Uc,tc,D,nS,nU,ww,scale,prob.rob),...
            @(Xc,Uc,D,scale)conIneq(Xc,Uc,D,scale),maxiter,dispLev);
    otherwise
        disp('Wrong option. Please try again');
        return;
end
disp('Initialization finished.');
disp('  ');
%% 建立原始数据
% 原始
prob.tf=0.485;
prob.dt=0.005;
prob.time = 0:prob.dt:prob.tf;
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,zeros(1,6),zeros(1,6),prob.rob);
velbnd=prob.bnds(:,3);
trqbnd=prob.bnds(:,4);
dtrqbnd=prob.bnds(:,5);
rotx=@(x)[1,0,0;0,cos(x),sin(x);0,-sin(x),cos(x)].';
rotz=@(x)[cos(x),sin(x),0;-sin(x),cos(x),0;0,0,1].';
roty=@(x)[cos(x),0,-sin(x);0,1,0;sin(x),0,cos(x)].';
state_record=[];control_record=[];time_record=[];

%% Case 1
% pause(1);
% R=[0 1 0;1 0 0;0 0 -1]; 
% P=[1.35293;1.82454;0.09928];    
% prob.init=[0.90109     -1.0175    -0.43049           0      1.4479     -0.6697];
% prob.target=prob.rob.kinv_ER80_2600(R,P);% joint space target position
% 
% disp(['Initial pos [',num2str(prob.init),']']);
% disp(['Target pos [',num2str(prob.target),']']);
% tmp=input('Press any key to run ');
% prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
% prob.time = 0:prob.dt:prob.tf;
% % generate initial motion
% ps.sGuess=nan(length(prob.time),ps.nS);
% sAcc=nan(length(prob.time),ps.nS/2);
% prob.u_init=nan(length(prob.time),ps.nU);
% for i=1:ps.nS/2
%     [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
%     ps.sGuess(:,i) = pos(:);
%     ps.sGuess(:,6+i) = vel(:);
%     sAcc(:,i) = acc(:);
% end
% for j=1:size(sAcc,1)
%     prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
% end
% [prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.target,prob.rob);
% % solve optimal control
% [opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
%     ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% % animation
% figure(1);clf;
% prob.rob.InitPlot([150,30]);
% for j=1:5:length(opt.Topt)
%     prob.rob.draw(opt.Xopt(j,1:6));
%     pause(0.02);
% end
% % record data
% state_record = [state_record;opt.Xopt];
% control_record = [control_record;opt.Uopt];
% if isempty(time_record)
%     time_record = [time_record;opt.Topt(:)];
% else
%     time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
% end
%% Case 2
% pause(1);
% R=[0 1 0;1 0 0;0 0 -1];
% *rotx(pi/2)*rotz(-pi/6);
% P=[1.35293;1.82454;0.15];
prob.init=[51.63 -56.70 -22.63 0 79.33 -38.37]*pi/180;
prob.target=[11.42 -74.76 -12.46 -38.41 52.79 13.75]*pi/180;

disp(['Initial pos [',num2str(prob.init),']']);
disp(['Target pos [',num2str(prob.target),']']);
tmp=input('Press any key to run ');
prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
prob.time = 0:prob.dt:prob.tf;
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
sAcc=nan(length(prob.time),ps.nS/2);
prob.u_init=nan(length(prob.time),ps.nU);
for i=1:ps.nS/2
    [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    sAcc(:,i) = acc(:);
end
for j=1:size(sAcc,1)
    prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
end
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.target,prob.rob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% animation
figure(1);clf;
prob.rob.InitPlot([150,30]);
for j=1:5:length(opt.Topt)
    prob.rob.draw(opt.Xopt(j,1:6));
    pause(0.02);
end
% record data
state_record = [state_record;opt.Xopt];
control_record = [control_record;opt.Uopt];
if isempty(time_record)
    time_record = [time_record;opt.Topt(:)];
else
    time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
end
%% example 1 
% pause(1);
% R=[0.750963054007751 0.00641082928101983 0.660313102083611;0.334011437095799 -0.866289518647558 -0.371455017156293;0.569640984660120 0.499501122270323 -0.652692712879604];
% P=[2.391508328945806;0.305524040253640;0.550523478258853];
% prob.init= [0.90109    -0.98968    -0.39492           0      1.3846     -0.6697];
% prob.target=prob.rob.kinv_ER80_2600(R,P);
% 
% disp(['Initial pos [',num2str(prob.init),']']);
% disp(['Target pos [',num2str(prob.target),']']);
% tmp=input('Press any key to run ');
% prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
% prob.time = 0:prob.dt:prob.tf;
% % generate initial motion
% ps.sGuess=nan(length(prob.time),ps.nS);
% sAcc=nan(length(prob.time),ps.nS/2);
% prob.u_init=nan(length(prob.time),ps.nU);
% for i=1:ps.nS/2
%     [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
%     ps.sGuess(:,i) = pos(:);
%     ps.sGuess(:,6+i) = vel(:);
%     sAcc(:,i) = acc(:);
% end
% for j=1:size(sAcc,1)
%     prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
% end
% [prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.target,prob.rob);
% % solve optimal control
% [opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
%     ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% % animation
% figure(1);clf;
% prob.rob.InitPlot([150,30]);
% for j=1:5:length(opt.Topt)
%     prob.rob.draw(opt.Xopt(j,1:6));
%     pause(0.02);
% end
% % record data
% state_record = [state_record;opt.Xopt];
% control_record = [control_record;opt.Uopt];
% if isempty(time_record)
%     time_record = [time_record;opt.Topt(:)];
% else
%     time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
% end
%% example 2
% pause(1);
% R=[0.750963054007751 0.00641082928101983 0.660313102083611;0.334011437095799 -0.866289518647558 -0.371455017156293;0.569640984660120 0.499501122270323 -0.652692712879604];
% P=[2.490555294258348;0.249805787680196;0.452619571326912];
% prob.init=prob.target;
% prob.target=prob.rob.kinv_ER80_2600(R,P);
% 
% disp(['Initial pos [',num2str(prob.init),']']);
% disp(['Target pos [',num2str(prob.target),']']);
% tmp=input('Press any key to run ');
% prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
% prob.time = 0:prob.dt:prob.tf;
% % generate initial motion
% ps.sGuess=nan(length(prob.time),ps.nS);
% sAcc=nan(length(prob.time),ps.nS/2);
% prob.u_init=nan(length(prob.time),ps.nU);
% for i=1:ps.nS/2
%     [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
%     ps.sGuess(:,i) = pos(:);
%     ps.sGuess(:,6+i) = vel(:);
%     sAcc(:,i) = acc(:);
% end
% for j=1:size(sAcc,1)
%     prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
% end
% [prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.target,prob.rob);
% % solve optimal control
% [opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
%     ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% % animation
% figure(1);clf;
% prob.rob.InitPlot([150,30]);
% for j=1:5:length(opt.Topt)
%     prob.rob.draw(opt.Xopt(j,1:6));
%     pause(0.02);
% end
% % record data
% state_record = [state_record;opt.Xopt];
% control_record = [control_record;opt.Uopt];
% if isempty(time_record)
%     time_record = [time_record;opt.Topt(:)];
% else
%     time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
% end

%% plot recorded results
% plot result

for j=1:6
    Vel(:,j)=state_record(:,6+j)*180/pi;
    Acc(:,j)=(gradient(Vel(:,j))./gradient(time_record(:)))/2;
%     /velbnd(j);
    Jerk(:,j)=gradient(Acc(:,j))./gradient(time_record(:));
    Trq(:,j)=control_record(:,j);
%     /trqbnd(j);
%     dtrq(:,j)=gradient(control_record(:,j))./gradient(time_record(:));
%     ratioDtrq(:,j)=dtrq(:,j)/dtrqbnd(j);

end
time_record(:,1)=time_record(:,1)*1.43555284;
figure(2);
plot(time_record,state_record(:,1:6)*180/pi,'linewidth',1.5);
grid on;xlim([time_record(1),time_record(end)]);
ylim([-215,215]);
title('Angle of each joint');
xlabel('time(s)');ylabel('pos(°)');
legend('joint1','joint2','joint3','joint4','joint5','joint6');

figure(3);
% clf;
% subplot(3,1,1);
plot(time_record,Vel,'linewidth',1.5);
grid on;xlim([time_record(1),time_record(end)]);
ylim([-200,200]);
title('Angle velocity of each joint');
xlabel('time(s)');ylabel('vel(°/s)');
legend('joint1','joint2','joint3','joint4','joint5','joint6');
% subplot(3,1,2);
figure(4)
plot(time_record,Acc(:,1:6),'linewidth',1.5);
grid on;xlim([time_record(1),time_record(end)]);
ylim([-430,430]);
title('Angle acceleration of each joint');
xlabel('time(s)');ylabel('Acc(°/s^2)');
legend('joint1','joint2','joint3','joint4','joint5','joint6');

figure(5)
plot(time_record,Jerk(:,1:6),'linewidth',1.5);
grid on;xlim([time_record(1),time_record(end)]);
ylim([-4300,4300]);
title('Angle jerk of each joint');
xlabel('time(s)');ylabel('Jerk(°/s^3)');
legend('joint1','joint2','joint3','joint4','joint5','joint6');

% subplot(3,1,3);
figure(6)
plot(time_record,Trq,'linewidth',1.5);
ylim([-4000,4000]);
grid on;xlim([time_record(1),time_record(end)]);
title('Torque of each joint');
xlabel('time(s)');ylabel('trq(Nm)');
legend('joint1','joint2','joint3','joint4','joint5','joint6');