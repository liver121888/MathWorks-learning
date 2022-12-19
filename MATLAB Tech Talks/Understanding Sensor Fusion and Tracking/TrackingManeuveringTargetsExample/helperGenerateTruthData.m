function [Xgt, tt, figs] = helperGenerateTruthData
%% Generate ground truth
vx = 200; % m/s
omega = 10; % deg/s
acc = 3; % m/s/s
dt = 0.1;
tt = (0:dt:floor(1000*dt));
figs = [];

Xgt = NaN(9,numel(tt));
Xgt(:,1) = 0;

% Constant velocity
seg1 = floor(numel(tt)/3);
Xgt(2,1) = vx;
slct = eye(9);
slct(3:3:end,:) = [];
for m = 2:seg1
    X0 = slct*Xgt(:,m-1);
    X1 = constvel(X0, dt);
    X1 = slct'*X1;
    Xgt(:,m) = X1;
end

% Constant turn
seg2 = floor(2*numel(tt)/3);
slct = eye(9);
slct(3:3:end,:) = [];
for m = seg1+1:seg2
    X0 = slct*Xgt(:,m-1);
    X0 = [X0(1:4);omega];
    X1 = constturn(X0, dt);
    X1 = X1(1:4);
    X1 = [X1(1:2);0;X1(3:4);0;zeros(3,1)];
    Xgt(:,m) = X1;
end

% Constant acceleration
first = true;
for m = seg2+1:numel(tt)
    X0 = Xgt(:,m-1);
    if first
        vel = X0(2:3:end);
        ua = vel/norm(vel);
        va = acc*ua;
        X0(3:3:end) = va;
        first = false;
    end
    X1 = constacc(X0, dt);
    Xgt(:,m) = X1;
end

% Drop acceleration dimension
slct = eye(9);
slct(3:3:end,:) = [];
Xgt = slct*Xgt;

figs = [figs figure];
plot(Xgt(1,1:seg1),Xgt(3,1:seg1),'.-');
hold on;
plot(Xgt(1,seg1+1:seg2),Xgt(3,seg1+1:seg2),'.-');
plot(Xgt(1,seg2+1:end),Xgt(3,seg2+1:end),'.-');
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('True Position')
axis equal;
legend('Constant Velocity', 'Constant Turn', 'Constant Acceleration')
end