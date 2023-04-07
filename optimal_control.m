%%
%Controle de um manipulador robótico de 6 juntas rotativas via MPC
%AUTOR: LUCCA GARCIA LEÃO
%DATA: 07/09/2021

%definiçao do MPC
nx = 6;
ny = 6;
nu = 6;
nlobj = nlmpc(nx,ny,nu);

Ts = 0.1;
nlobj.Ts = Ts

nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;

%%
%definicao da planta
%mdl_ur10
mdl_ur5

%posição inicial e desejada
x_center = 0.1;
z_center = 0.1;
r = 0.5;
x_d = x_center + r * cos(0);
z_d = z_center + r * sin(0);

R = [-1 0 0; 0 -1 0; 0 0 1];
t = [x_center -1 z_center];
rt = rt2tr(R,t);
q0 = ur5.ikunc(rt);
plot(ur5,q0)
hold on
trplot(rt)

for ct = 1:nu
    nlobj.MV(ct).Min = -10.0;
    nlobj.MV(ct).Max = 10.0;
end

nlobj.Model.StateFcn = "manipDT";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;

%%
%valida nlmpc
%q0 = qr;
%u0 = [1 1 1 1 1 1];
%validateFcns(nlobj,q0,u0,[],{Ts});

%%
%loop de controle
mv = zeros(nu,1);
mv_min = -10.0;
mv_max = 10.0;
%yref = ikine(ur5, xf);

T_sample = 0.1;

%yref = [pi/2 -pi/6 -pi/4 pi/2 pi/2 0];
%xf = ur5.fkine(yref)
%q0 = [0 0 0 0 0 0];
%xf = transl(x_d, -1, z_d);
%yref = ur5.ikine6s(xf);

x = q0';
yref = q0;
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

%hbar = waitbar(0,'Simulation Progress');
xHistory = x;
mvHistory = mv;
i = 0;
goalHistory = yref;
while(i <= 3*pi)
    q_old = yref;
    x_d = x_center + r * cos(i);
    z_d = z_center + r * sin(i);

    R = [-1 0 0; 0 -1 0; 0 0 1];
    trans = [x_d -1 z_d];
    rt = rt2tr(R,trans);
    yref = ur5.ikunc(rt,q_old);
    
    %feedforward
    xd_dot = (yref - q_old)/T_sample
    
    [mv,nloptions,info] = nlmpcmove(nlobj,x,mv,yref,[],nloptions);
    mvd = mv + xd_dot';
    %mvd = mv;
    mvd(mvd > mv_max) = mv_max;
    mvd(mvd < mv_min) = mv_min;
    x = manipDT(x,mvd,Ts);
    y = x;
    %waitbar(ct*Ts/20,hbar);
    xHistory = [xHistory x];
    mvHistory = [mvHistory mvd];
    goalHistory = [goalHistory;yref];
    i = i+T_sample
end

%%
%plota a saída
xl = []
yl = []
zl = []
x_goal = []
y_goal = []
z_goal = []
roll = []
pitch = []
yaw = []
erroD = []
goalHistory = goalHistory';
for k = 1:length(xHistory)
   pose = ur5.fkine(xHistory(:,k));
   poseGoal = ur5.fkine(goalHistory(:,k));
   transGoal = poseGoal.t;
   x_goal = [x_goal transGoal(1)];
   y_goal = [y_goal transGoal(2)];
   z_goal = [z_goal transGoal(3)];
   oriGoal = poseGoal.o;
   trans = pose.t;
   ori = pose.o;
   xl = [xl trans(1)];
   yl = [yl trans(2)];
   zl = [zl trans(3)];
   roll = [roll ori(1)];
   pitch = [pitch ori(2)];
   yaw = [yaw ori(3)];
   err = sqrt((transGoal(1) - trans(1))^2 + (transGoal(2) - trans(2))^2 + (transGoal(3) - trans(3))^2);
   erroD = [erroD err];
end

%%
%plot da trajetoria
%trplot(xf)
%hold on
t = 0:0.1:20;
Q = xHistory';
plot(ur5,Q);
hold on
plot3(xl,yl,zl,'Color',[1 0 0],'LineWidth',2);
%%
%plota estados
subplot(2,3,1);
plot(t(1:96),xl(1:96),"LineWidth",1.5);
yline(xl(96),'-.');
title("x");

subplot(2,3,2);
plot(t(1:96),yl(1:96),"LineWidth",1.5);
yline(yl(96),'-.');
title("y");

subplot(2,3,3);
plot(t(1:96),zl(1:96),"LineWidth",1.5);
yline(zl(96),'-.');
title("z");

subplot(2,3,4);
plot(t(1:96),roll(1:96),"LineWidth",1.5);
yline(roll(96),'-.');
title("roll");

subplot(2,3,5);
plot(t(1:96),pitch(1:96),"LineWidth",1.5);
yline(pitch(96),'-.');
title("pitch");

subplot(2,3,6);
plot(t(1:96),yaw(1:96),"LineWidth",1.5);
yline(yaw(96),'-.');
title("yaw");

%%
subplot(2,2,1);
plot(t(1:96),erroD(1:96),"LineWidth",1.5);
yline(roll(96),'-.');
title("erro");

subplot(2,2,2);
plot(t(1:96),roll(1:96),"LineWidth",1.5);
yline(roll(96),'-.');
title("roll");

subplot(2,2,3);
plot(t(1:96),pitch(1:96),"LineWidth",1.5);
yline(pitch(96),'-.');
title("pitch");

subplot(2,2,4);
plot(t(1:96),yaw(1:96),"LineWidth",1.5);
yline(yaw(96),'-.');
title("yaw");

%%
%plota sinal de controle
u1 = mvHistory(1,:);
u2 = mvHistory(2,:);
u3 = mvHistory(3,:);
u4 = mvHistory(4,:);
u5 = mvHistory(5,:);
u6 = mvHistory(6,:);

subplot(2,3,1);
plot(t(1:50),u1(1:50),"LineWidth",1.5);
title("u1");

subplot(2,3,2);
plot(t(1:50),u2(1:50),"LineWidth",1.5);
title("u2");

subplot(2,3,3);
plot(t(1:50),u3(1:50),"LineWidth",1.5);
title("u3");

subplot(2,3,4);
plot(t(1:50),u4(1:50),"LineWidth",1.5);
title("u4");

subplot(2,3,5);
plot(t(1:50),u5(1:50),"LineWidth",1.5);
title("u5");

subplot(2,3,6);
plot(t(1:50),u6(1:50),"LineWidth",1.5);
title("u6");


%%
%penalizando mais a variacao do sinal de controle
nlobj.Weights.OutputVariables = [0.1 0.1 0.1 0.1 0.1 0.1];
nlobj.Weights.ManipulatedVariablesRate = [1 1 1 1 1 1];

%%
%loop de controle
mv = zeros(nu,1);
%yref = ikine(ur5, xf);
yref = [pi/2 -pi/6 -pi/4 pi/2 pi/2 0];
xf = ur5.fkine(yref)
q0 = [0 0 0 0 0 0];

x = q0';

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

%hbar = waitbar(0,'Simulation Progress');
xHistory = x;
mvHistory = mv;
i = 0;
for ct = 1:(5/Ts)
    [mv,nloptions,info] = nlmpcmove(nlobj,x,mv,yref,[],nloptions);
    x = manipDT(x,mv,Ts);
    y = x;
    %waitbar(ct*Ts/20,hbar);
    xHistory = [xHistory x];
    mvHistory = [mvHistory mv];
    i = i+1;
end
