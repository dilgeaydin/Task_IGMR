clear all
close all
clc
syms x1 y1 z1 x2 y2 real %1 for the position to pick up 2 for the position to drop
syms q1 q2 q3 real %joint variables
vel=0.1; %velocity of the conveyor belt and time
samptime=0.05;
%% forward kinematics
T1=DH(0,0.312,0,0);
T2=DH(q1,0.05,0.225,0);
T3=DH(q2,(-0.116-q3),0.225,0);
Tend=simplify(T1*T2*T3);
%% inverse
% eqn = Tend(1:3,4)==[x1;y1;z1];
pos1=[0.22;-0.1;0.01]; %first spotted position to be picked up
eqn1 = Tend(1:3,4)==pos1; 
time=0:samptime:1;
for i=1:length(time) %normally this is a while loop which breaks when the end effector calibrates its orientation and picks up the parts
eqn = Tend(1:3,4) == pos1;
pos1(1)=pos1(1)-vel*0.05;%meanwhile the belt is moving, the end effector is moving horizontally
temp1=vpa(solve(eqn).q1);
temp2=vpa(solve(eqn).q2);
q11(i)=vpa(rad2deg(temp1(1)),3); %first joint trajectory
q22(i)=vpa(rad2deg(temp2(1)),3); %second joint trajectory
end

q33=vpa(solve(eqn).q3); %stroke
stroke=q33(1)*ones(1,length(time));

for i=1:length(time)
pos(1:3,i)=vpa(subs(Tend(1:3,4),[q1 q2 q3],[deg2rad([q11(i),q22(i)]),stroke(i)]),3);%check if it's the right position
end

eqn2 = Tend(1:3,4)==[0.32;0.1;0.01]; %position to be dropped
temp1=vpa(solve(eqn2).q1);
temp2=vpa(solve(eqn2).q2);
temp3=vpa(solve(eqn2).q3);
q11(i+1)=vpa(rad2deg(temp1(1)),3); %first joint trajectory
q22(i+1)=vpa(rad2deg(temp2(1)),3); %second joint trajectory
stroke(i+1)=vpa((temp3(1)),3);
time(i+1)=time(end)+samptime;
pos(1:3,i+1)=vpa(subs(Tend(1:3,4),[q1 q2 q3],[deg2rad([q11(i+1),q22(i+1)]),stroke(i+1)]),3);%check if it's the right position

figure
plot(time,q11,time,q22,'LineWidth',2)
grid on
title('Joint variables')
xlabel('Time(sec)')
ylabel('Degrees (°)')
legend('First joint','Second joint','location','best')

figure
plot(time,pos*100,'LineWidth',2)
grid on
title('end effector position')
xlabel('Time(sec)')
ylabel('Position (cm)')
legend('x','y','z','location','best')