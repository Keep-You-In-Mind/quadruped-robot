

    %机器人
    %设X轴正方向为机器人前进方向

    % 初始参数
    L_total = 0.5;%总长度1m
    L1 = 0.1*L_total ;%第一腿节长度
    L2 = 0.4*L_total;%第二腿节
    L3 = 0.5*L_total;%第三腿节
    BodyWidth= 0.25;
    BodyLength= 0.6;%
    BodyThick = 0.05;
    BodyCenter_To_Arm=BodyLength*0.4;
    legX = 0.05;
    legY = 0.05;
    legZ = 0.05;
    R_foot = 0.025;
    
    % 触地足端生成
    X=R_foot*cosd(0:0.2:90);
    Y= R_foot*sind(0:0.2:90);
    ball = [X' Y'];

    Mark =1 ;%求解逆运动学

    % 获得足端轨迹
    % 足端初始位置是:
    %     X_FOOTS = 0;
    %     Y_FOOTS = L1;
    %     Z_FOOTS = -L2-L3;

    S = 50;%S H 为毫米
    H = 50;
    Tm = 0.1;
    T = 2*Tm;
    sampleTime = 0.001;
    [x_swing,x_stance,Z_trajectory]=getTrajectory(S,Tm,T,H,sampleTime);
    figure
    plot([x_swing x_stance],Z_trajectory);

    %X_FOOT等单位是m
    X_FOOT = ([x_swing x_stance])/1000-S/2/2/1000;
    Y_FOOT = L1;
    Z_FOOT = -L2-L3+Z_trajectory/1000+0.15;

    %getFootSpace(L1,L2,L3);



    open_system('quadrupedRobot.slx');








function [x,y,z]=forwardKinematics(a,L1,L2,theta0,theta1,theta2)
    
    H = L1*cosd(theta1)+L2*cosd(theta1+theta2);
    x = L1*sind(theta1)+L2*sind(theta1+theta2);
    z = -H*cosd(theta0)+a*sind(theta0);
    y = a*cosd(theta0)+H*sind(theta0);
end



function getFootSpace(L1,L2,L3)
    %footWorkSpace

    % theta0 = [-60:60];
    % theta1 = [-60:60];
    % theta2 = [-60:60];
    for  index = 1:0.0001:1.5
        theta0 = rand()*60;
        if rand()>=0.5
            theta0=-theta0;
        end

        theta1 = rand()*60;
        if rand()>=0.5
            theta1=-theta1;
        end

        theta2 = rand()*60;
        if rand()>=0.5
            theta2=-theta2;
        end

        [x,y,z]=forwardKinematics(L1,L2,L3,theta0,theta1,theta2);  
        plot3(x,y,z,'go');
        hold on

    end

    hold off
end


function  [x_swing,x_stance,y_trajectory]=getTrajectory(S,Tm,T,H,sampleTime)
    %% 王立鹏 基于足端轨迹规划算法
    [x_swing,x_stance,y_trajectory]=footTrajectory(S,Tm,T,H,sampleTime);%
end
