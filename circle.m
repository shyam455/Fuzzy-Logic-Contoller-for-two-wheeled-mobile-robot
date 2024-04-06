% Circle Trajectory of Two Wheeled Mobile Robot

clc; clear all; close all;

delta_t = 0.1; % time for one step
D = 10; % distance between robot's wheels

x = 500;
y = 0;
theta = 90;

traj(1,1) = x;
traj(2,1) = y;

mfd = zeros(1,5);
mfa = zeros(1,7);

r = zeros(5,7);

% five output linguistic variables for linear velocity
lv(1) = 0; % ZS(1)
lv(2) = 5;  % SS(2)
lv(3) = 10; % MS(3)
lv(4) = 15; % FS(4)
lv(5) = 20; % VFS(5)

% seven output linguistc variables for anguler velocity
Wlv(1) = 0;      % ZAV(1)
Wlv(2) = 60;     % SPAV(2)
Wlv(3) = -60;    % SNAV(3)
Wlv(4) = 120;    % MPAV(4)
Wlv(5) = -120;   % MNAV(5)
Wlv(6) = 180;    % LPAV(6)
Wlv(7) = -180;   % LNAV(7)

% RULE BASE

% rule(e_d,e_theta) = VW
% 1->ZA         1->ZD 
% 2->SPA        2->SD
% 3->SNA        3->MD
% 4->MPA        4->LD
% 5->MNA        5->VLD
% 6->LPA
% 7->LNA     

rule(1,1)=11;
rule(1,2)=12;
rule(1,3)=13;
rule(1,4)=14;
rule(1,5)=15;
rule(1,6)=16;
rule(1,7)=17;
rule(2,1)=21;
rule(2,2)=22;
rule(2,3)=23;
rule(2,4)=24;
rule(2,5)=25;
rule(2,6)=26;
rule(2,7)=27;
rule(3,1)=31;
rule(3,2)=32;
rule(3,3)=33;
rule(3,4)=34;
rule(3,5)=35;
rule(3,6)=36;
rule(3,7)=37;
rule(4,1)=41;
rule(4,2)=42;
rule(4,3)=43;
rule(4,4)=44;
rule(4,5)=45;
rule(4,6)=46;
rule(4,7)=47;
rule(5,1)=51;
rule(5,2)=52;
rule(5,3)=53;
rule(5,4)=54;
rule(5,5)=55;
rule(5,6)=56;
rule(5,7)=57;

count = 450;
b = pi/count;

liss(1,1) = 0;
liss(2,1) = 0;

error(1,1) = 0;

rms_sum = 0;

for i=1:1:count
    
    % desired position(Lissajous trajectory)
    xd = 500*cos(2*i*b);
    yd = 500*sin(2*i*b);

    liss(1,i) = xd;
    liss(2,i) = yd;
    
    % error in distance 
    e_d = sqrt((xd-x)^2+(yd-y)^2);

    thetad = atan2d(yd-y,xd-x); % desired orientation of robot
    if thetad<0
        thetad = thetad + 360;
    end
    
    e_theta = thetad - theta; % error in orientaion of robot
    if e_theta>180
        e_theta = e_theta-360;
    end
    if e_theta<-180
        e_theta = e_theta + 360;
    end

    % ZD
    if e_d>=0 && e_d<5
        mfd(1,1) = (5-e_d)/5;
    else
        mfd(1,1) = 0;
    end
    
    % SD
    if e_d>=0 && e_d<=5
        mfd(1,2) = e_d/5;
    elseif e_d>5 && e_d<=10
        mfd(1,2)=(10-e_d)/5;
    else
        mfd(1,2)=0;
    end

    % MD
    if e_d>=5 && e_d<=10
        mfd(1,3) = (e_d-5)/5;
    elseif e_d>10 && e_d<=15
        mfd(1,3)=(15-e_d)/5;
    else
        mfd(1,3)=0;
    end

    % LD
    if e_d>=10 && e_d<=15
        mfd(1,4) = (e_d-10)/5;
    elseif e_d>15 && e_d<=20
        mfd(1,4)=(20-e_d)/5;
    else
        mfd(1,4)=0;
    end

    % VLD
    if e_d>=15 && e_d<=20
        mfd(1,5) = (e_d-15)/5;
    elseif e_d>20
        mfd(1,5)=1;
    else
        mfd(1,5)=0;
    end
    
    % ZA
    if e_theta>=-60 && e_theta<=0
        mfa(1,1) = (e_theta+60)/60;
    elseif e_theta>0 && e_theta<=60
        mfa(1,1)=(60-e_theta)/60;
    else
        mfa(1,1)=0;
    end
    
    % SPA
    if e_theta>=0 && e_theta<=60
        mfa(1,2) = (e_theta)/60;
    elseif e_theta>60 && e_theta<=120
        mfa(1,2)=(120-e_theta)/60;
    else
        mfa(1,2)=0;
    end

    % SNA
    if e_theta>=-120 && e_theta<=-60
        mfa(1,3) = (e_theta+120)/60;
    elseif e_theta>-60 && e_theta<=0
        mfa(1,3)=(-e_theta)/60;
    else
        mfa(1,3)=0;
    end

    % MPA
    if e_theta>=60 && e_theta<=120
        mfa(1,4) = (e_theta-60)/60;
    elseif e_theta>120 && e_theta<=180
        mfa(1,4)=(180-e_theta)/60;
    else
        mfa(1,4)=0;
    end

    % MNA
    if e_theta>=-180 && e_theta<=-120
        mfa(1,5) = (e_theta+180)/60;
    elseif e_theta>-120 && e_theta<=-60
        mfa(1,5)=(e_theta+60)/(-60);
    else
        mfa(1,5)=0;
    end

    % VPA
    if e_theta>=120 && e_theta<=180
        mfa(1,6) = (e_theta-120)/60;
    else
        mfa(1,6)=0;
    end

    % VNA
    if e_theta>=-180 && e_theta<=-120
        mfa(1,7) = (e_theta+120)/(-60);
    else
        mfa(1,7)=0;
    end
    
    % We are using center of gravity method for defuzzyfication so
    % calculating numerator and denominator for crisp output
    sum_num_V = 0;
    sum_num_W = 0;
    sum_den_V = 0;
    sum_den_W = 0;
    
    for j=1:5
        for k=1:7
            r(j,k) = min(mfd(1,j),mfa(1,k));
                        
            p =fix(rule(j,k)/10);
            q = (rule(j,k)-10*p);

            sum_num_V = sum_num_V + r(j,k)*lv(p);
            sum_num_W = sum_num_W + r(j,k)*Wlv(q);
            
            sum_den_V = sum_den_V + r(j,k);
            sum_den_W = sum_den_W + r(j,k);
        end
    end


    % Calculating crisp output values
    V = sum_num_V/sum_den_V;
    W = sum_num_W/sum_den_W;
    
    % Updating x and y coordinates using kinematics
    x = x + V*cosd(theta)*delta_t*9.7;
    y = y + V*sind(theta)*delta_t*9.7;
    
    % Updating orientation of robot
    delta_theta = W*delta_t*9.7;
    theta = theta + delta_theta;
    if theta<0
        theta = theta + 360;
    end
    
    % Storing actual trajectory of mobile robot
    traj(1,i) = x;
    traj(2,i) = y;

    error(1,i) = sqrt((xd-x)^2+(yd-y)^2);
    rms_sum = rms_sum + error(1,i)*error(1,i);

end
rms_error = sqrt(rms_sum/count);
rms_error

figure, plot(traj(1,:),traj(2,:),'ro'), hold on, plot(liss(1,:),liss(2,:),'g');
legend('Actual Trajectory', 'Desired Trajectory');
set(legend,'FontSize',15);
xlabel('X-axis(cm)'), ylabel('Y-axis(cm)'); %, grid;
set(gca,'FontSize',15);
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',15); 
h_ylabel = get(gca,'YLabel');
set(h_ylabel,'FontSize',15);grid,

figure(2);
time = 1:1:i;
plot(time,error(1,:),'k');
legend('Error');
set(legend,'FontSize',15);
xlabel('Time'),ylabel('Error(cm)');
set(gca,'FontSize',15);
h_xlabel = get(gca,'XLabel');
set(h_xlabel,'FontSize',15);
h_ylabel=get(gca,'YLabel');
set(h_ylabel,'FontSize',15);grid,