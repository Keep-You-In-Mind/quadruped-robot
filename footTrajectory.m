

% 王立鹏,王军政,汪首坤,等. 基于足端轨迹规划算法的液压四足机器人步态控制策略
function  [x_swing,x_stance,y]=footTrajectory(S,Tm,T,H,sample_time)
% S = 160 ;%单位mm
% Tm = 0.5;%单位s
% H = 230;%单位mm

% T = 2*Tm;
% y = A*Tm/4/pi*( t - Tm/4/pi*sin(4*pi*t/Tm)) +C2;
i = 1;
%sample_time %采样周期

for t = 0:sample_time:Tm
    x_swing(i) = S*( t/Tm - 1/2/pi*sin(2*pi*t/Tm) )- S/2;
    if t>=0 && t<Tm/2
        y_swing(i) = 2*H*( t/Tm - 1/4/pi*sin(4*pi*t/Tm) );
    else
        if t>=Tm/2 && Tm>=t
        y_swing(i) = 2*H*( 1 - t/Tm + 1/4/pi*sin(4*pi*t/Tm) );
        end
        
    end
    i = i+1;
end

%电动四足机器人单腿设计与轨迹规划研究 岳天奇
t = Tm+sample_time:sample_time:T;
x_stance = S*( (2*Tm-t)/Tm + 1/2/pi*sin(2*pi*t/Tm)) - S/2;
y_stance = -H;

x = [x_swing x_stance];
y = [y_swing ones( 1,length(x_stance) )*y_stance*0];%我需要把y移到0处
t = 0:sample_time:T;
% plot(x,y,'r');

%动画
% figure
% for i=1:length(x)
%     plot(x(i),y(i),'r*');
%     axis([min(x) max(x)+1 min(y) max(y)+1])
%     hold on
%     pause(0.01);
% end
% hold off
