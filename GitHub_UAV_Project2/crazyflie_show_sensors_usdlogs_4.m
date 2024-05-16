% crazyflie load usd card log csv file for sensor analysis
clear all

% available files:
csvfilename_motors_off = 'L2Data1.csv';
csvfilename_hover = 'L2Data2.csv';
csvfilename_motion_x = 'L2Data3.csv';
csvfilename_motion_y = 'L2Data4.csv';
csvfilename_motion_z = 'L2Data5.csv';
csvfilename_motion_inf = 'L2Data6.csv';

% read file
csvfilename = csvfilename_motion_y;
array = dlmread(csvfilename,',',1,0);
T = table2array(readtable(csvfilename)); % Matlab only

% get data from table
time = array(:,1)'*1e-3;
pos = array(:,2:4)'; % [m]
vel = array(:,5:7)'; % [m/s]
lbd = array(:,8:10)'; % [deg]
gyro = array(:,11:13)'; % [deg/s]
acc = array(:,14:16)'*9.81; % [Gs]
baro_asl = array(:,17)'; % [m]
% lighthouse = array(:,18:20))'; % [m]

% convert date to print format
t = time - time(1);

syms  ax ay az phi_m theta_m psi_m b_acc n_acc real
% phi_m = 0 ;
theta_m = 0 ;
psi_m = 0 ;

rotx = [	1	,	0			,	 0         ;
			0	,	cos(phi_m)	,	-sin(phi_m)  ;
			0	,	sin(phi_m)	,	 cos(phi_m)  ];

roty = [	 cos(theta_m)	,	0	,	sin(theta_m)  ;
			0				,	1	,	0       ;
			-sin(theta_m)	,	0	,	cos(theta_m)	];

rotz = [	cos(psi_m)	,	-sin(psi_m)	,	0   ;
			sin(psi_m)	,	 cos(psi_m)	,	0   ;
			0			,	 0		,	1	];

R = rotz*roty*rotx;

g = 9.81 ;
e3 = [0;0;-1] ;
p_dd = [0;0;0] ;

acc_m =  vpa ( R' * (p_dd - g * e3) + b_acc + n_acc ) 

phi_m = asin((acc (2,:)./g))*180/pi ;

figure(14)
plot(t,lbd(1,:),t,phi_m)
xlabel('$$t$$ [s]');
ylabel('$$\phi[deg]$$');
grid on
legend({'DataSet4','Accelerometer'},'FontSize',8)


% plot data
% initPlots;
% crazyflie_show_sensor_data(t,acc,gyro,baro_asl,pos,vel,lbd);
