initPlots
clear all
clc

% Arquivos disponíveis:
csvfilename_motors_off = 'L2Data1.csv';
csvfilename_hover = 'L2Data2.csv';
csvfilename_motion_x = 'L2Data3.csv';
csvfilename_motion_y = 'L2Data4.csv';
csvfilename_motion_z = 'L2Data5.csv';
csvfilename_motion_inf = 'L2Data6.csv';

% Ler arquivo
csvfilename = csvfilename_motion_x;  % Arquivo correto para o DataSet3
array = dlmread(csvfilename,',',1,0);
T = table2array(readtable(csvfilename, 'VariableNamingRule', 'preserve')); % Apenas Matlab

% Obter dados da tabela
time = array(:,1)'*1e-3;
lbd = array(:,8:10)'; % [deg]


%sacar valores
gyro = array(:,11:13)';

% Converter data para formato de impressão
t = time - time(1);

% Simular sistema e estimativa
Nsim = length(t);
%% 


% for k = 1:Nsim
% A (:,:,k) = [ 0 0 1 0 0 0 
%       0 0 0 1 0 0
%       0 0 0 0 0 0
%       0 0 0 0 0 0
%       0 0 0 0 0 0
%       0 0 0 0 0 0];
% end

B = zeros(6,0);

g = 9.81;


% C = zeros(4,6);
% for k=1:Nsim
% 
%     C (:, :, k) = [ 0, 0, 1, 0, 1, 0;
%                     0, 0, 0, 1, 0, 1;
%                     0, -cos(lbd(2,k)*pi/180)*g, 0, 0, 0, 0  ;        
%                    cos(lbd(1,k)*pi/180), 0, 0, 0, 0, 0;
%                  ];
% 
% end

D = zeros(2,0);

Ts = time(2) - time(1);        % Amostragem de tempo
% F(,k) = expm(A*Ts);                % Matriz de estado discreta
% F = Ts*A ;

G = [1;0;0;0;0;0];
% H = C;
% ny = size(H,1);
nx = 6;                % Número de estados, colunas da matriz H = C = 6

% Ganhos de Kalman
Q = 0.013*diag([1,1,2,2,1,1]);   %covariancia do ruido do processo
R = cov([gyro(1:2,:);lbd(1:2,:)]'); %covariancia 

%% 

% Valores iniciais
P0 = 2*eye(nx);
x0 = zeros(nx,1);


y = [gyro(1:2,:);lbd(1:2,:)];
x = zeros(nx,Nsim);
xe = zeros(nx,Nsim);    % x estimado
Pe = zeros(nx,nx,Nsim);

x(:,1) = x0;
xe(:,1) = x0;
Pe(:,:,1) = P0;
%% 

for k = 1:Nsim
   
    A  = [ 0 0 1 0 0 0 
                  0 0 0 1 0 0
                  0 0 0 0 0 0
                  0 0 0 0 0 0
                  0 0 0 0 0 0
                  0 0 0 0 0 0];

    F = Ts*A ;

    C  = [ 0, 0, 1, 0, 1, 0;
                    0, 0, 0, 1, 0, 1;
                    0, -cos(lbd(2,k)*pi/180)*g, 0, 0, 0, 0  ;        
                    cos(lbd(1,k)*pi/180)*g, 0, 0, 0, 0, 0 ];
    H = C;

    % Predizer próxima estimativa:
    Gu = 0;
    [xem,Pem] = extendKalman_predict_function(xe(:,k),Pe(:,:,k),F,Q,Gu);


    % Obter medições (simuladas)
    % v_noise = sqrtm(R)*randn(ny,1);
    % y(:,k) = H*x(:,k) + v_noise;

    % Atualizar estimativa com informação da medição
    [xe(:,k),Pe(:,:,k),K] = extendkalman_update(xem,Pem,y(:,k),H,R);

    % Simular dinâmica do sistema e ruído de processo
    xp = F*x(:,k) + Gu; % Ruído de processo não adicionado

    if k < Nsim % Atualizar próximo estado até o último instante
        x(:,k+1) = xp;
    end
end

l = load ('dados_kalmanfilter_y.mat')

xfl = double(l.dados_kalmanfilter_y)

% Mostrar resultados roll (ângulo de rotação)
figure(20041);
plot(t,xe(1,:)*180/pi,t,xfl(1,:)*180/pi,t,lbd(1,:)); 
grid on;
xlabel('$$t$$[s]');
ylabel('$$\phi[deg]$$');
legend('EKF','KF','DataSet3');  % Legenda correta

% Mostrar resultados pitch (ângulo de inclinação)
figure(20043);
plot(t,xe(2,:)*180/pi,t,xfl(2,:)*180/pi,t,lbd(2,:)); 
grid on;
xlabel('$$t$$[s]');
ylabel('$$\theta[deg]$$');
legend('EKF','KF','DataSet3');  % Legenda correta

% figure(20042);
% sigma_phi = sqrt(permute(Pe(1,1,:),[3,1,2]));
% plot(t,sigma_phi*180/pi);
% grid on;
% xlabel('t [s]');
% ylabel('$$\sigma_{\phi}$$ [deg]');
% title('roll angle STD')