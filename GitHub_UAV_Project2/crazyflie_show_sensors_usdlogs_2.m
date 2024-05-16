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
csvfilename = csvfilename_hover;
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

figure(10)
subplot(3,1,1)
plot (t(:,715:6742),acc(1,715:6742))
xlabel('$$t$$ [s]');
ylabel('$$a_x[m/s^2]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

subplot(3,1,2)
plot (t(:,715:6742),acc(2,715:6742))
xlabel('$$t$$ [s]');
ylabel('$$a_y[m/s^2]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

subplot(3,1,3)
plot (t(:,715:6742),acc(3,715:6742))
xlabel('$$t$$ [s]');
ylabel('$$a_z[m/s^2]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

figure(11)
subplot(3,1,1)
plot (t(:,715:6742),gyro(1,715:6742))
xlabel('$$t$$ [s]');
ylabel('$$\omega_x[deg/s]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

subplot(3,1,2)
plot (t(:,715:6742),gyro(2,715:6742))
xlabel('$$t$$ [s]');
ylabel('$$\omega_y[deg/s]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

subplot(3,1,3)
plot (t(:,715:6742),gyro(3,715:6742))
xlabel('$$t$$ [s]');
ylabel('$$\omega_z[deg/s]$$');
set(groot,'defaultLineLineWidth',0.5)
grid on

X_a = mean (acc(:,715:6742)') ;
cov_a = cov (acc(:,715:6742)') ;
disp(['Média Acelerómetro: ', num2str(X_a)]) ;
disp('Covariância Acelerómetro: ') ;
disp (diag(cov_a)) ;


X_g = mean (gyro(:,715:6742)') ;
cov_g = cov (gyro(:,715:6742)') ;
disp(['Média Gyro: ', num2str(X_g)]);
disp('Covariância Gyro: ');
disp(diag(cov_g));

% plot data
%initPlots;
% crazyflie_show_sensor_data(t,acc,gyro,baro_asl,pos,vel,lbd);
