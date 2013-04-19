%% SERIAL STUFF
%ME567 PROJECT
%%CONFIGURATION
COMPORT = 'COM5' %Differs on each PC.
%% Setup Serial Communication to the OWI-565 Robotic ARM
delete(instrfind) %Deletes all instrument objects 
%as we know we only have one
s = serial(COMPORT);
s.BytesAvailableFcnMode = 'terminator'; %Serial Stream of sends 5 Joints
% Encoder counts followed by CR/LF
s.Terminator = 'CR/LF'
s.BytesAvailableFcn = @mycallback %Function Called wen CR/LF is seen
%Setup Data Storage Locations
s.UserData.isNew = 0
s.UserData.newData = 0;
 %Opens the Serial Communcation Port
 if strcmp(s.Status,'closed'), fopen(s); end
% get(s) %Uncomment this line and it gives all the COM port stats. Useful
% for debugging. 
% fread(s,1,'char') %Function to read the Serial Port

% Inverse Kinematic

% DH parameters
%        |a |alpha |d   |theta
% Link 1 |0 |  90  |H   |theta1
% Link 2 |L1|  0   |0   |theta2
% Link 3 |L2|  0   |0   |theta3
% Link 4 |L3|  0   |0   |theta4

%% The General Translation Matrix
syms theta alpha a d

A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];

a90 = sym(pi/2);

syms H L1 L2 L3 t1 t2 t3 t4

A10 = subs(A,[theta alpha a d],[t1 a90 0 H]);
A21 = subs(A,[theta alpha a d],[t2 0 L1 0]);
A32 = subs(A,[theta alpha a d],[t3 0 L2 0]);
A43 = subs(A,[theta alpha a d],[t4 0 L3 0]);

A20 = simplify (A10*A21);
A30 = simplify (A20*A32);
A40 = simplify (A30*A43);

%% Link definition (theta d a alpha)
L(1)=Link([0,4.5,0,pi/2]);
L(2)=Link([0,0,9,0]);
L(3)=Link([0,0,11.1,0]);
L(4)=Link([0,0,11.5,0]);

owi535=SerialLink(L,'name','OWI-535');

% initial theta with robotic arm “straight up”
Init_theta = [pi/2 pi/2 0 0];

T_init = owi535.fkine(Init_theta);
Pos_init = T_init(1:3,4);

% Initial position
figure
owi535.plot(Init_theta);

%% Path Generation
% random motion
% A1 = [15;10;3];
% A2 = [-15;0;3];
% A3 = [0;15;15];
% disp1 = [A1,A2,A3]

%% Draw a line
% linestep = 30;
% latitude = -2.5;

% Desired Starting Position
% % A0 = Pos_init;
% A1 = [13.6;23.6;-2.5];
% A2 = [-13.6;23.6;-2.5];
% A3 = Pos_init;
% 
% % trackX = [linspace(A0(1),A1(1),linestep), linspace(A1(1),A2(1),linestep),...
% %             linspace(A2(1),A3(1),linestep)];
% % trackY = [linspace(A0(2),A1(2),linestep),linspace(A1(2),A2(2),linestep),...
% %             linspace(A2(2),A3(2),linestep)];
% % trackZ = [linspace(A1(3),A2(3),linestep),linspace(A1(3),A2(3),linestep),...
% %             linspace(A2(3),A3(3),linestep)];
%         
% % trackX = [linspace(A1(1),A2(1),linestep),...
% %             linspace(A2(1),A3(1),linestep)];
% % trackY = [linspace(A1(2),A2(2),linestep),...
% %             linspace(A2(2),A3(2),linestep)];
% % trackZ = [linspace(A1(3),A2(3),linestep),...
% %             linspace(A2(3),A3(3),linestep)];
%         
% trackX = [linspace(A1(1),A2(1),linestep),A3(1)];
% trackY = [linspace(A1(2),A2(2),linestep),A3(2)];
% trackZ = [linspace(A1(3),A2(3),linestep),A3(3)];
% 
% disp1 = [trackX;trackY;trackZ];


%% Draw an M
% % desired position 
linestep = 10;
latitude = -2.5;

% M1
A0 = [13.6;23;latitude];
A1 = [13.6;23;latitude];
A2 = [8;15;latitude];
A3 = [0;23;latitude];
A4 = [-8;15;latitude];
A5 = [-13.6;23;latitude];

% M2

% A1 = [10;-20;latitude];
% A2 = [8;-7;latitude];
% A3 = [0;-14;latitude];
% A4 = [-8;-7;latitude];
% A5 = [-10;-20;latitude];

A6 = Pos_init;

trackX=[linspace(A0(1),A1(1),linestep), linspace(A1(1),A2(1),linestep),...
    linspace(A2(1),A3(1),linestep),linspace(A3(1),A4(1),linestep),...
    linspace(A4(1),A5(1),linestep),linspace(A5(1),A6(1),linestep)];
    
trackY=[linspace(A0(2),A1(2),linestep), linspace(A1(2),A2(2),linestep),...
    linspace(A2(2),A3(2),linestep),linspace(A3(2),A4(2),linestep),...
    linspace(A4(2),A5(2),linestep),linspace(A5(2),A6(2),linestep)];;

trackZ=[linspace(A0(3),A1(3),linestep), linspace(A1(3),A2(3),linestep),...
    linspace(A2(3),A3(3),linestep),linspace(A3(3),A4(3),linestep),...
    linspace(A4(3),A5(3),linestep),linspace(A5(3),A6(3),linestep)];

disp1 = [trackX;trackY;trackZ];

%% Draw a circle
% linestep = 10;
% % set the start position
% radius = 15;
% latitude = 3;
% init_angle = -pi/4;
% final_angle = 5*pi/4;
% disp_theta = [init_angle: (final_angle - init_angle)/linestep : final_angle];
% 
% trackX = radius*cos(disp_theta);
% trackY = radius*sin(disp_theta);
% trackZ = latitude*ones(1,size(disp_theta,2))
% 
% disp1 = [trackX;trackY;trackZ];


%% 
% set desire position
Pos_desire = disp1;

% set current position
Current_theta = Init_theta;
Current_Pos = Pos_init;

% % position difference
epsilon = 0.1;
step = 0.1;

% propotional gain
K = 2;
% K = 10;

% desired speed
% Spd_desire= 2*(A2-A1)/1000;
% Spd_desire=[0.001;0.001;0.005];
Spd_desire=[0.01;0.01;0.05];

% parameters to adjusting the singularity
w0 = 0.01;
k0 = 0.001;

% joint velocity saturation limit
deg = pi/180;

% theta limit
limit_theta_1_p = 235 * deg;
limit_theta_1_n = - 45 * deg;

% limit_theta_2_p = 90 * deg;
% limit_theta_2_n = -90 * deg;

limit_theta_2_p = 180 * deg;
limit_theta_2_n = 0 * deg;

limit_theta_3_p = 150 * deg;
limit_theta_3_n = -150 * deg;

limit_theta_4_p = 60 * deg;
limit_theta_4_n = -60 * deg;

% for vedio M only
% limit_theta_1_p = 360 * deg;
% limit_theta_1_n = - 360 * deg;
% 
% limit_theta_2_p = 360 * deg;
% limit_theta_2_n = - 360 * deg;
% 
% limit_theta_3_p = 360 * deg;
% limit_theta_3_n = -360 * deg;
% 
% limit_theta_4_p = 360 * deg;
% limit_theta_4_n = -360 * deg;
% % end here

% theta_dot_limit
limit_theta_dot_p = 0.1*[(limit_theta_1_p - limit_theta_1_n) /step;...
    (limit_theta_2_p - limit_theta_2_n)/step;...
    (limit_theta_3_p - limit_theta_3_n)/step;...
    (limit_theta_4_p - limit_theta_4_n)/step];

limit_theta_dot_n = - limit_theta_dot_p;

% limit_theta_dot_p = 0.1*[270 *deg/step;...
%     180*deg/step;...
%     300*deg/step;...
%     120*deg/step];
% 
% limit_theta_dot_n = - limit_theta_dot_p;

theta_dot_display = [];
current_theta_display = [];
current_pos_display = [];
current_pos_vedio = [];
Theta_Command = [];%zeros(4,linestep+1);
% h = figure(100);
its = 0;
WHEREAMI = 1
for i = 1:1:size(Pos_desire,2) 
%     h = figure(i);
    while (sqrt(sum((Pos_desire(:,i) - Current_Pos).^2)) > epsilon)
    
    A10_real = double(subs (A10,[t1 a90 0 H],[Current_theta(1) pi/2 0 4.5]));
    A21_real = double(subs(A21,[t2 0 L1 0],[Current_theta(2) 0 9 0]));
    A32_real = double(subs(A32,[t3 0 L2 0],[Current_theta(3) 0 11.1 0]));
    A43_real = double(subs(A43,[t4 0 L3 0],[Current_theta(4) 0 6.5 0]));
    
    A20_real = A10_real*A21_real;
    A30_real = A20_real*A32_real;
    A40_real = A30_real*A43_real;
%     
    z00 = [0;0;1];
    z10 = A10_real(1:3,3);
    z20 = A20_real(1:3,3);
    z30 = A30_real(1:3,3);
    
    o00 = [0;0;0];
    o10 = A10_real(1:3,4);
    o20 = A20_real(1:3,4);
    o30 = A30_real(1:3,4);
    o40 = A40_real(1:3,4);
   
    Pn = o40;
    
    Jw (:,1) = z00;
    Jw (:,2) = z10;
    Jw (:,3) = z20;
    Jw (:,4) = z30;
    
    Jv (:,1) = cross (z00,(Pn - o00));
    Jv (:,2) = cross (z10,(Pn - o10));
    Jv (:,3) = cross (z20,(Pn - o20));
    Jv (:,4) = cross (z30,(Pn - o30));
    
    % Jacobian with only velocity
    J = [Jv];
    
    % check whether there is singularity
    w = sqrt(det(J*J'));
    if (w < w0)
        k1 = k0*(1-w/w0)^2;
    else
        k1 = 0;
    end

    % pesudo jacobian calculation
    J_pesudo = double(J'*inv(J*J'+k1*eye(size(J*J'))));

    % get theta dot
    theta_dot = J_pesudo * (Spd_desire + K*(Pos_desire(:,i)-Current_Pos));
    
    % saturation limit on theta_dot
    theta_dot = min(theta_dot,limit_theta_dot_p);
    theta_dot = max(theta_dot,limit_theta_dot_n);
    
    %record theta_dot_display
    theta_dot_display = [theta_dot_display,theta_dot];
    
    % get current theta 
    Current_theta = double(Current_theta + theta_dot' * step);
    
    Current_theta(1) = min(Current_theta(1),limit_theta_1_p);
    Current_theta(1) = max(Current_theta(1),limit_theta_1_n);

    Current_theta(2) = min(Current_theta(2),limit_theta_2_p);
    Current_theta(2) = max(Current_theta(2),limit_theta_2_n);
    
    Current_theta(3) = min(Current_theta(3),limit_theta_3_p);
    Current_theta(3) = max(Current_theta(3),limit_theta_3_n);
    
    Current_theta(4) = min(Current_theta(4),limit_theta_4_p);
    Current_theta(4) = max(Current_theta(4),limit_theta_4_n);

    
    % record current theta
    current_theta_display = [current_theta_display,Current_theta'];
    
    % get current end effector position
    % current tranmission matrix
    T_current = owi535.fkine(Current_theta);
    
    % assign this to the current position
    Current_Pos = T_current(1:3,4);
    current_pos_display = [current_pos_display,Current_Pos];
    %   plot(Current_Pos)
    %owi535.plot(Current_theta);
    %hold on;
    
    if (abs (Current_Pos(3) - latitude) < epsilon )
        %current_pos_vedio = [current_pos_vedio,Current_Pos];
    end
    
    if (~isempty(current_pos_vedio))
        %plot3(current_pos_vedio(1,:),current_pos_vedio(2,:),...
           % current_pos_vedio(3,:),'b','LineWidth',3);
    end
    
    end
    
        Theta_Command = [Theta_Command,Current_theta'];
    WHEREAMI = WHEREAMI +1
    
    
    
    
    
    % for vedio record only
    % F(i) = getframe(h);
    % close(h);
    % pause (1)
end

Theta_Command = Theta_Command*180/pi
for i = 1:1:size(Theta_Command,2)
    %pause
    %Send New Command
    for j = 4:-1:1
        setJointPos(s,j-1,Theta_Command(j,i))
        pause(.25)
    end
    %owi535.plot(Theta_Command(:,i)'*pi/180)
    while(max(abs(s.UserData.newData(1:4)-(Theta_Command(1:4,i)')))>5)
        its = its+1
        if(its>20) 
            break
        end
        s.UserData.newData(1:4)-(Theta_Command(1:4,i)')
        pause(.25)
    end
    its = 0;
    
end









% 
% its = 0;
% hold off
% 
% % plot of theta_dot
% figure
% for i = 1:1:size(theta_dot_display,1)
%     subplot(2,2,i)
%     plot(theta_dot_display(i,:))
%     xlabel('step')
%     ylabel(['theta dot ',num2str(i)])
%     grid on
% end
% 
% % plot of theta
% figure
% for i = 1:1:size(current_theta_display,1)
%     subplot(2,2,i)
%     plot(current_theta_display(i,:))
%     xlabel('step')
%     ylabel(['theta',num2str(i)])
%     grid on
% end
% 
% % plot of position
% Position  = ['X','Y','Z']
% figure 
% for i = 1:1:size(current_pos_display,1)
%     subplot(2,2,i)
%     plot(current_pos_display(i,:))
%     xlabel('step')
%     ylabel([Position(i)])
%     grid on
% end    
%    
% figure
% plot3(current_pos_display(1,:),current_pos_display(2,:),current_pos_display(3,:))
% grid on
% owi535.teach

%% Generate Vedio
% writerObj = VideoWriter('simulationM.avi');
% open(writerObj);
% writeVideo(writerObj,F);
% myVideo.FrameRate = 1;
% close(writerObj);
