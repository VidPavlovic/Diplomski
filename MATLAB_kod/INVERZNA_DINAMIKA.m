%% Uvod
% -MATLAB 2023
% -pokrenut kinematske parametre, pa dinamicke, provritit simualciju za
%  odabrani slucaj, pa pokrenut plot
% -pri sljedecoj simulaciji sve opet

%Vid Pavlovic

%% parametri kinematika
clc
clear all

wb = 0.141;
wp = 0.02625;
L = 0.230;
l = 0.544;

sb = (6 / sqrt(3)) * wb;
sp = (6 / sqrt(3)) * wp;
ub = 2*wb;
up = 2*wp;

a = wb - up;
b = sp/2 - (sqrt(3)/2) * wb;
c = wp - 0.5 * wb;

kinematika = [a,b,c,L,l];


%% radno podrucje bez ogranicenja
kut_min = -60 * (pi/180);
kut_max = 89.9* (pi/180);
korak = 5* (pi/180);

x_points = [];
y_points = [];
z_points = [];

for j1 = kut_min:korak:kut_max
    for j2 = kut_min:korak:kut_max
        for j3 = kut_min:korak:kut_max
            [x_temp, y_temp, z_temp] = DIREKTNA(j1, j2, j3, kinematika);
            if x_temp ~= 0 && y_temp ~= 0 && z_temp ~= 0
                x_points = [x_points, x_temp];
                y_points = [y_points, y_temp];
                z_points = [z_points, z_temp];
            end
        end
    end
end

cmap = jet;
z_normalized = (z_points - min(z_points)) / (max(z_points) - min(z_points));
figure;
scatter3(x_points, y_points, z_points,6,cmap(ceil(z_normalized * (size(cmap,1) - 1)) + 1, :), 'filled');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;

%% s ogranicenjem
x_points_new = [];
y_points_new = [];
z_points_new = [];

beta_max = 20;

for kord = 1:length(z_points)
    [theta1, theta2, theta3, beta1t, beta2t, beta3t] = INVERZNA(x_points(1,kord), y_points(1,kord), z_points(1,kord), kinematika);
    if (beta1t < beta_max && beta1t >-beta_max) && (beta2t < beta_max && beta2t >-beta_max) && (beta3t < beta_max && beta3t >-beta_max)
        x_points_new(end+1) = x_points(1,kord);
        y_points_new(end+1) = y_points(1,kord);
        z_points_new(end+1) = z_points(1,kord);
    end
end

scatter3(x_points_new, y_points_new, z_points_new,5,y_points_new,'filled');
xlim([-0.3 0.3]);
ylim([-0.3 0.3]);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;

%% dinamika
I = 1.76;%PRIJENOSNI OMJER REMEN
mn = 0.350; %masa prihvatnice
m_payload = 0.400; %teret
mfb = 2 * 0.102;  %dezina donjeg clana (DVA CLANA)
r = 2/3;
mb = 0.308; %nadlaktica
mc = 0.062; %ZGLOB navojna
g = -9.81; %m/s2
Ibc = (L^2)*((1/3)*mb + mc + r*mfb); %inercija
mnt = mn + m_payload + 3*(1-r)*mfb; %kg
mbr = mb + mc + r*mfb; %kg
Rgb = L*((0.5*mb + mc + r*mfb)/(mbr));
dinamika = [g;Ibc;mnt;mbr;Rgb;I];

%% SLUCAJ 1 TRAJEKTORIJA
Tt= 3;
t = 0:0.001:Tt;
radius = 0.17;

ddq = zeros(length(t),3);
dq = zeros(length(t),3);
q = zeros(length(t),3);
xyz_pos = zeros(length(t),3);
xyz_pos_direkt = zeros(length(t),3);
xyz_v = zeros(length(t),3);
xyz_a = zeros(length(t),3);
beta = zeros(length(t),3);
J = zeros(length(t),3,3);
dJ = zeros(length(t),3,3);

A = zeros(length(t),3);
C= zeros(length(t),3);
G = zeros(length(t),3);
T = zeros(length(t),3);
T_mot = zeros(length(t),3);
T_gn = zeros(length(t),3);
T_gb = zeros(length(t),3);
T_b = zeros(length(t),3);
T_n = zeros(length(t),3);
m_max = zeros(length(t),1);

for i = 1:length(t)
    [xyz_pos(i,:),xyz_v(i,:),xyz_a(i,:)]= KINEM_TEST(t(i),radius,1);
    [q(i,1),q(i,2),q(i,3),beta(i,1),beta(i,2),beta(i,3)] = INVERZNA(xyz_pos(i,1),xyz_pos(i,2),xyz_pos(i,3),kinematika);
    [dq(i,:), ddq(i,:)] = JACOBIAN_INV_F(q(i,:),xyz_pos(i,:),kinematika, xyz_v(i,:), xyz_a(i,:));
    [J(i, :, :),dJ(i, :, :)] = RETURN_JACOBIAN(q(i,:),xyz_pos(i,:),kinematika,dq(i,:));
    [xyz_pos_direkt(i,1),xyz_pos_direkt(i,2),xyz_pos_direkt(i,3),beta(i,1),beta(i,2),beta(i,3)] = DIREKTNA(q(i,1),q(i,2),q(i,3),kinematika);
    [A(i,:),C(i,:),G(i,:),T(i,:),T_mot(i,:),T_gn(i,:),T_gb(i,:),T_b(i,:),T_n(i,:)] = Robot(q(i,:),dq(i,:),ddq(i,:),dinamika, squeeze(J(i,:,:)),squeeze(dJ(i,:,:)));
    [m_max(i)] = vakuum(xyz_a(i,:));
end

set(0, 'DefaultLineLineWidth', 2);
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')

%VANJSKE KOORDINATE
figure
subplot(3,1,1)
plot(t,xyz_pos(:,1))
hold on
plot(t,xyz_pos(:,2))
hold on
plot(t,xyz_pos(:,3))
hold on
plot(t, xyz_pos_direkt(:,1),'--')
hold on
plot(t, xyz_pos_direkt(:,2),'--')
hold on
plot(t, xyz_pos_direkt(:,3),'--')
hold on
xlabel("Vrijeme t[s]")
ylabel("Pozicija[m]")
legend("X","Y","Z","X direktna", "Y direktna", "Z direktna")

subplot(3,1,2)
plot(t,xyz_v(:,1))
hold on
plot(t,xyz_v(:,2))
hold on
plot(t,xyz_v(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Brzina [m/s]")
legend("X","Y","Z")

subplot(3,1,3)
plot(t,xyz_a(:,1))
hold on
plot(t,xyz_a(:,2))
hold on
plot(t,xyz_a(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Akceleracija [m/s^2]")
legend("X","Y","Z")



%% TROKUTNO

Tt= 1.5;
t = 0:0.001:Tt;

ddq = zeros(length(t),3);
dq = zeros(length(t),3);
q = zeros(length(t),3);
xyz_pos = zeros(length(t),3);
xyz_v = zeros(length(t),3);
xyz_a = zeros(length(t),3);
beta = zeros(length(t),3);

A = zeros(length(t),3);
C= zeros(length(t),3);
G = zeros(length(t),3);
T = zeros(length(t),3);
T_mot = zeros(length(t),3);
T_gn = zeros(length(t),3);
T_gb = zeros(length(t),3);
T_b = zeros(length(t),3);
T_n = zeros(length(t),3);


J = zeros(length(t),3,3);
dJ = zeros(length(t),3,3);
m_max = zeros(length(t),1);

% 0.14 za kraj radnog prostora otp

[q10,q20,q30,~,~,~] = INVERZNA(0.140,0.140,-0.6,kinematika);
[q1f,q2f,q3f,~,~,~] = INVERZNA(-0.140,-0.140,-0.4,kinematika);
tau = 1.02;

for i = 1:length(t)
    [q(i,1),dq(i,1),ddq(i,1)]= TRIANGLE(q10,q1f,t(i),tau);
    [q(i,2),dq(i,2),ddq(i,2)]= TRIANGLE(q20,q2f,t(i),tau);   %
    [q(i,3),dq(i,3),ddq(i,3)]= TRIANGLE(q30,q3f,t(i),tau);   %
    [xyz_pos(i,1),xyz_pos(i,2),xyz_pos(i,3),beta(i,1),beta(i,2),beta(i,3)] = DIREKTNA(q(i,1),q(i,2),q(i,3),kinematika);
    [xyz_v(i,:), xyz_a(i,:)] = JACOBIAN_F(q(i,:),xyz_pos(i,:),kinematika, dq(i,:), ddq(i,:));
    [J(i, :, :),dJ(i, :, :)] = RETURN_JACOBIAN(q(i,:),xyz_pos(i,:),kinematika,dq(i,:));
    [A(i,:),C(i,:),G(i,:),T(i,:),T_mot(i,:),T_gn(i,:),T_gb(i,:),T_b(i,:),T_n(i,:)] = Robot(q(i,:),dq(i,:),ddq(i,:),dinamika, squeeze(J(i,:,:)),squeeze(dJ(i,:,:)));
    [m_max(i)] = vakuum(xyz_a(i,:));
end

%% TRAPEZNO

Tt = 1.5;
t = 0:0.001:Tt;

ddq = zeros(length(t),3);
dq = zeros(length(t),3);
q = zeros(length(t),3);
xyz_pos = zeros(length(t),3);
xyz_v = zeros(length(t),3);
xyz_a = zeros(length(t),3);

A = zeros(length(t),3);
C= zeros(length(t),3);
G = zeros(length(t),3);
T = zeros(length(t),3);
T_mot = zeros(length(t),3);
T_gn = zeros(length(t),3);
T_gb = zeros(length(t),3);
T_b = zeros(length(t),3);
T_n = zeros(length(t),3);
J = zeros(length(t),3,3);
dJ = zeros(length(t),3,3);
m_max = zeros(length(t),1);

[q10,q20,q30,~,~,~] = INVERZNA(0.140,0.140,-0.6,kinematika);
[q1f,q2f,q3f,~,~,~] = INVERZNA(-0.140,-0.140,-0.4,kinematika);
dq0 = 0;

% q1f = 0;
% q2f = q20;
% q3f = q30;

i = I * 13.73;
vmax_pps = 2500; %pps mot
degree_per_step = deg2rad(0.9);
dq_max = (vmax_pps*degree_per_step)/i;
amax_pps = 30000;
ddq_max = (amax_pps*degree_per_step)/i;


for i = 1:length(t)
    [q(i,1),dq(i,1),ddq(i,1)]= TRAP(t(i),q10,q1f,dq0,dq_max,ddq_max);
    [q(i,2),dq(i,2),ddq(i,2)]= TRAP(t(i),q20,q2f,dq0,dq_max,ddq_max);   
    [q(i,3),dq(i,3),ddq(i,3)]= TRAP(t(i),q30,q3f,dq0,dq_max,ddq_max);
    [xyz_pos(i,1),xyz_pos(i,2),xyz_pos(i,3),beta(i,1),beta(i,2),beta(i,3)] = DIREKTNA(q(i,1),q(i,2),q(i,3),kinematika);
    [xyz_v(i,:), xyz_a(i,:)] = JACOBIAN_F(q(i,:),xyz_pos(i,:),kinematika, dq(i,:), ddq(i,:));
    [J(i, :, :),dJ(i, :, :)] = RETURN_JACOBIAN(q(i,:),xyz_pos(i,:),kinematika,dq(i,:));
    [A(i,:),C(i,:),G(i,:),T(i,:),T_mot(i,:),T_gn(i,:),T_gb(i,:),T_b(i,:),T_n(i,:)] = Robot(q(i,:),dq(i,:),ddq(i,:),dinamika, squeeze(J(i,:,:)),squeeze(dJ(i,:,:)));
    [m_max(i)] = vakuum(xyz_a(i,:));
end

%% plotanje STARIJE VERZIJE

set(0, 'DefaultLineLineWidth', 2);
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')

%UNUTARNJE KOORDINATE
figure
subplot(3,1,1)
plot(t,q(:,1))
hold on
plot(t,q(:,2))
hold on
plot(t,q(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Pozicija q [rad]")
legend("q_1","q_2","q_3")

subplot(3,1,2)
plot(t,dq(:,1))
hold on
plot(t,dq(:,2))
hold on
plot(t,dq(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Brzina dq [rad/s]")
legend("dq_1","dq_2","dq_3")

subplot(3,1,3)
plot(t,ddq(:,1))
hold on
plot(t,ddq(:,2))
hold on
plot(t,ddq(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Akceleracija ddq [rad/s^2]")
legend("ddq_1","ddq_2","ddq_3")

%VANJSKE KOORDINATE
figure
subplot(3,1,1)
plot(t,xyz_pos(:,1))
hold on
plot(t,xyz_pos(:,2))
hold on
plot(t,xyz_pos(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Pozicija[m]")
legend("X","Y","Z")

subplot(3,1,2)
plot(t,xyz_v(:,1))
hold on
plot(t,xyz_v(:,2))
hold on
plot(t,xyz_v(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Brzina [m/s]")
legend("X","Y","Z")

subplot(3,1,3)
plot(t,xyz_a(:,1))
hold on
plot(t,xyz_a(:,2))
hold on
plot(t,xyz_a(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Akceleracija [m/s^2]")
legend("X","Y","Z")

%KOMPONENTE MOMENTA PO ÈLANOVIMA
figure
subplot(3,1,1)
plot(t, T(:,1))
hold on
plot(t, A(:,1))
hold on
plot(t, C(:,1))
hold on
plot(t, G(:,1))
hold on
xlabel("Vrijeme t[s]")
ylabel("Moment T_1 [Nm]")
legend('T','A','C','G')

subplot(3,1,2)
plot(t, T(:,2))
hold on
plot(t, A(:,2))
hold on
plot(t, C(:,2))
hold on
plot(t, G(:,2))
hold on
xlabel("Vrijeme t[s]")
ylabel("Moment T_2 [Nm]")
legend('T','A','C','G')

subplot(3,1,3)
plot(t, T(:,3))
hold on
plot(t, A(:,3))
hold on
plot(t, C(:,3))
hold on
plot(t, G(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Moment T_3 [Nm]")
legend('T','A','C','G')

%MOMENTI NA MOTORU
figure
subplot(2,1,1)
plot(t,T_mot(:,1))
hold on
plot(t,T_mot(:,2))
hold on
plot(t,T_mot(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Momenti T_m_o_t [Nm]")
legend("T_1", "T_2", "T_3")
subplot(2,1,2)
plot(t,dq(:,1)*(60/(2*pi))*I)
hold on
plot(t,dq(:,2)*(60/(2*pi))*I)
hold on
plot(t,dq(:,3)*(60/(2*pi))*I)
hold on
xlabel("Vrijeme t[s]")
ylabel("Brzina vrtnje motora [RPM]")
legend("dq_1","dq_2","dq_3")



%MAKSIMALNA MASA PRI PUNOM VAKUMU
figure
plot(t, m_max(:,1))
hold on
xlabel("Vrijeme t[s]");
ylabel("Maksimalna dopuštena masa tereta [kg]");

%KUT BETA
figure
plot(t, beta(:,1))
hold on
plot(t, beta(:,2))
hold on
plot(t, beta(:,3))
hold on
xlabel("Vrijeme t[s]");
ylabel("Kut zglobova beta [stup]");
legend("Beta1","Beta2","Beta3");

%POJEDINI MOMENTI
figure
subplot(3,1,1)
plot(t, T_gn(:,1))
hold on
plot(t, T_gb(:,1))
hold on
plot(t, T_b(:,1))
hold on
plot(t, T_n(:,1))
hold on
plot(t, T(:,1))
hold on
xlabel("Vrijeme t[s]")
ylabel("Moment T_1 [Nm]")
legend('T_g_n','T_g_b','T_b','T_n','T')

subplot(3,1,2)
plot(t, T_gn(:,2))
hold on
plot(t, T_gb(:,2))
hold on
plot(t, T_b(:,2))
hold on
plot(t, T_n(:,2))
hold on
plot(t, T(:,2))
hold on
xlabel("Vrijeme t[s]")
ylabel("Moment T_2 [Nm]")
legend('T_g_n','T_g_b','T_b','T_n','T')


subplot(3,1,3)
plot(t, T_gn(:,3))
hold on
plot(t, T_gb(:,3))
hold on
plot(t, T_b(:,3))
hold on
plot(t, T_n(:,3))
hold on
plot(t, T(:,3))
hold on
xlabel("Vrijeme t[s]")
ylabel("Moment T_3 [Nm]")
legend('T_g_n','T_g_b','T_b','T_n','T')

%%
function [dq,ddq] = JACOBIAN_INV_F(q,xyz,kinematika2,v,accel)
x = xyz(1);
y = xyz(2);
z = xyz(3);

theta1 = q(1);
theta2 = q(2);
theta3 = q(3);

a = kinematika2(1);
b = kinematika2(2);
c = kinematika2(3);
L = kinematika2(4);
l = kinematika2(5);

A = [x,y + a + L*cos(theta1),z + L*sin(theta1);
    2*(x + b) - sqrt(3)*L*cos(theta2), 2*(y + c) - L*cos(theta2), 2*(z + L*sin(theta2));
    2*(x - b) + sqrt(3)*L*cos(theta3), 2*(y + c) - L*cos(theta3), 2*(z + L*sin(theta3))];

b11 = L*((y + a)*sin(theta1) - z*cos(theta1));
b22 = -L*((sqrt(3)*(x + b) + y + c)*sin(theta2) + 2*z*cos(theta2));
b33 = L*((sqrt(3)*(x - b) - y - c)*sin(theta3) - 2*z*cos(theta3));

B = [b11, 0, 0;
    0, b22, 0;
    0, 0, b33];

J = inv(A)*B;

dq = inv(J)*[v(1);v(2);v(3)];

dx = v(1);
dy = v(2);
dz = v(3);
dtheta1 = dq(1);
dtheta2 = dq(2);
dtheta3 = dq(3);

dA = [dx, dy - L*sin(theta1)*dtheta1, dz + L*cos(theta1)*dtheta1;
      2*dx + L*sqrt(3)*sin(theta2)*dtheta2, 2*dy + L*sin(theta2)*dtheta2, 2*dz + 2*L*cos(theta2)*dtheta2;
      2*dx - sqrt(3)*L*sin(theta3)*dtheta3, 2*dy + L*sin(theta3)*dtheta3, 2*dz + 2*L*cos(theta3)*dtheta3];

%db11 =  L*((a + y)*cos(theta1)*dtheta1 + y*sin(theta1)*dtheta1 + sin(theta1)*dtheta1 - cos(theta1)*dtheta1); % pokusaj word
db11 = L*(dy*sin(theta1) + y*cos(theta1)*dtheta1) + L*a*cos(theta1)*dtheta1 - L*(dz*cos(theta1) - z*sin(theta1)*dtheta1); %novi prvi dio


db22 = -L*((sqrt(3)*dx + dy)*sin(theta2) + (c + sqrt(3)*(b + x) + y)*cos(theta2)*dtheta2 - 2*z*sin(theta2)*dtheta2 + 2*cos(theta2)*dz);
db33 = L*((sqrt(3)*dx - dy)*sin(theta3) + (-c + sqrt(3)*(-b + x) - y)*cos(theta3)*dtheta3 + 2*z*sin(theta3)*dtheta3 - 2*cos(theta3)*dz);
  
dB = [db11,0,0;
      0,db22,0;
      0,0,db33];

ddq = inv(B)*(A*[accel(1);accel(2);accel(3)] + dA*[v(1);v(2);v(3)] - dB*dq);
end

%%
function [dx_out, ddx] = JACOBIAN_F(q,xyz,kinematika,dq,ddq)
x = xyz(1);
y = xyz(2);
z = xyz(3);

theta1 = q(1);
theta2 = q(2);
theta3 = q(3);

a = kinematika(1);
b = kinematika(2);
c = kinematika(3);
L = kinematika(4);

A = [x,y + a + L*cos(theta1),z + L*sin(theta1);
    2*(x + b) - sqrt(3)*L*cos(theta2), 2*(y + c) - L*cos(theta2), 2*(z + L*sin(theta2));
    2*(x - b) + sqrt(3)*L*cos(theta3), 2*(y + c) - L*cos(theta3), 2*(z + L*sin(theta3))];

b11 = L*((y + a)*sin(theta1) - z*cos(theta1));
b22 = -L*((sqrt(3)*(x + b) + y + c)*sin(theta2) + 2*z*cos(theta2));
b33 = L*((sqrt(3)*(x - b) - y - c)*sin(theta3) - 2*z*cos(theta3));

B = [b11, 0, 0;
    0, b22, 0;
    0, 0, b33];

J = inv(A)*B;

v = J*[dq(1);dq(2);dq(3)];
dx_out = v;
dx = v(1);
dy = v(2);
dz = v(3);

dtheta1 = dq(1);
dtheta2 = dq(2);
dtheta3 = dq(3);

dA = [dx, dy - L*sin(theta1)*dtheta1, dz + L*cos(theta1)*dtheta1;
      2*dx + L*sqrt(3)*sin(theta2)*dtheta2, 2*dy + L*sin(theta2)*dtheta2, 2*dz + 2*L*cos(theta2)*dtheta2;
      2*dx - sqrt(3)*L*sin(theta3)*dtheta3, 2*dy + L*sin(theta3)*dtheta3, 2*dz + 2*L*cos(theta3)*dtheta3];

db11 = L*(dy*sin(theta1) + y*cos(theta1)*dtheta1) + L*a*cos(theta1)*dtheta1 - L*(dz*cos(theta1) - z*sin(theta1)*dtheta1); %novi prvi dio
db22 = -L*((sqrt(3)*dx + dy)*sin(theta2) + (c + sqrt(3)*(b + x) + y)*cos(theta2)*dtheta2 - 2*z*sin(theta2)*dtheta2 + 2*cos(theta2)*dz);
db33 = L*((sqrt(3)*dx - dy)*sin(theta3) + (-c + sqrt(3)*(-b + x) - y)*cos(theta3)*dtheta3 + 2*z*sin(theta3)*dtheta3 - 2*cos(theta3)*dz);
  
dB = [db11,0,0;
      0,db22,0;
      0,0,db33];

ddx = inv(A)*(dB*[dq(1);dq(2);dq(3)] + B*[ddq(1);ddq(2);ddq(3)] - dA*[v(1);v(2);v(3)]);
end

%%

function [theta1,theta2,theta3,beta1d,beta2d,beta3d] = INVERZNA(x,y,z, kinematika)

    a = kinematika(1);
    b = kinematika(2);
    c = kinematika(3);
    L = kinematika(4);
    l = kinematika(5);
    
    limit_lower = pi/2;
    limit_upper = -2*pi/3;
    
    
    E1 = 2 * L * (y + a);
    F1 = 2 * z * L;
    G1 = x^2 + y^2 + z^2 + a^2 + L^2 + 2 * y * a - l^2;
    
    E2 = -L * (sqrt(3) * (x + b) + y + c);
    F2 = 2 * z * L;
    G2 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2 * (x * b + y * c) - l^2;
    
    E3 = L * (sqrt(3) * (x - b) - y - c);
    F3 = 2 * z * L;
    G3 = x^2 + y^2 + z^2 + b^2 + c^2 + L^2 + 2 * (-x * b + y * c) - l^2;
    
    dis1 = E1^2 + F1^2 - G1^2;
    dis2 = E2^2 + F2^2 - G2^2;
    dis3 = E3^2 + F3^2 - G3^2;
    
    if dis1 < 0 || dis2 < 0 || dis3 < 0
        error('No real solution, discriminant is negative.');
    end
    
    theta_11 = 2 * atan((-F1 + sqrt(dis1)) / (G1 - E1));
    theta_12 = 2 * atan((-F1 - sqrt(dis1)) / (G1 - E1));
    theta_21 = 2 * atan((-F2 + sqrt(dis2)) / (G2 - E2));
    theta_22 = 2 * atan((-F2 - sqrt(dis2)) / (G2 - E2));
    theta_31 = 2 * atan((-F3 + sqrt(dis3)) / (G3 - E3));
    theta_32 = 2 * atan((-F3 - sqrt(dis3)) / (G3 - E3));
    
    if theta_11 > limit_lower && theta_11 < limit_upper
        theta1 = theta_11;
        theta2 = theta_21;
        theta3 = theta_31;
    else
        theta1 = theta_12;
        theta2 = theta_22;
        theta3 = theta_32;
    end
    
    x01 = x;
    x02 = x*cos(2*pi/3) + y*sin(2*pi/3);
    x03 = x*cos(4*pi/3) + y*sin(4*pi/3);
    
    beta1d = rad2deg(asin(x01/l));
    beta2d = rad2deg(asin(x02/l));
    beta3d = rad2deg(asin(x03/l));
    
    
end
%%

function [x0,y0,z0,beta1d,beta2d,beta3d] = DIREKTNA(theta1,theta2,theta3, kinematika)

    t = kinematika(1);
    Ld = kinematika(5);
    Lg = kinematika(4);
    
    y1 = -(t + Lg*cos(theta1));
    z1 = -Lg*sin(theta1);

    y2 = (t + Lg*cos(theta2)) * sind(30);
    x2 = y2 * tand(60);
    z2 = -Lg*sin(theta2);

    y3 = (t + Lg*cos(theta3)) * sind(30);
    x3 = -y3 * tand(60);
    z3 = -Lg*sin(theta3);
    
    dnm = (y2 - y1)*x3 - (y3 - y1)*x2;
    w1 = y1^2 + z1^2;
    w2 = x2^2 + y2^2 + z2^2;
    w3 = x3^2 + y3^2 + z3^2;
    a1 = (z2 - z1)*(y3 - y1) - (z3 - z1)*(y2 - y1);
    b1 = -((w2 - w1)*(y3 - y1) - (w3 - w1)*(y2 - y1)) / 2.0;
    a2 = -(z2 - z1)*x3 + (z3 - z1)*x2;
    b2 = ((w2 - w1)*x3 - (w3 - w1)*x2) / 2.0;
    a = a1^2 + a2^2 + dnm^2;
    b = 2 * (a1*b1 + a2*(b2 - y1*dnm) - z1*dnm^2);
    c = (b2 - y1*dnm)^2 + b1^2 + dnm^2 * (z1^2 - Ld^2);
    d = b^2 - 4*a*c;
    
    if d < 0
        x0 = NaN;
        y0 = NaN;
        z0 = NaN;
        return;
    end

    z0 = -0.5 * (b + sqrt(d)) / a;
    x0 = (a1*z0 + b1) / dnm;
    y0 = (a2*z0 + b2) / dnm;
    
    x01 = x0;
    x02 = x0*cos(2*pi/3) + y0*sin(2*pi/3);
    x03 = x0*cos(4*pi/3) + y0*sin(4*pi/3);
    
    beta1d = rad2deg(asin(x01/Ld));
    beta2d = rad2deg(asin(x02/Ld));
    beta3d = rad2deg(asin(x03/Ld));
end
%%

function [position,velocity,acceleration] = TRIANGLE(q_start, q_final, t, tau)
    Delta_q = q_final - q_start;
    tau_half = tau / 2;
    a = (4 * Delta_q) / (tau^2); 
    acceleration = 0;
    velocity = 0;
    position = q_start;

    if t < 0
        velocity = 0;
        position = q_start;
        
    elseif t <= tau_half
        acceleration = a;
        velocity = acceleration * t;
        position = q_start + 0.5 * acceleration * t^2;
        
    elseif t <= tau
        t_dec = t - tau_half;
        acceleration = -a;
        v_max = a * tau_half;
        velocity = v_max - a * t_dec;
        position = q_start + (0.5 * a * tau_half^2) + v_max * t_dec - 0.5 * a * t_dec^2;  
        
    else
        velocity = 0;
        position = q_final;
    end
end
%%
function [q, v, a] = TRAP(t, q0, qf, v0, vmax, amax)
    direction = sign(qf - q0);
    delta_q = abs(qf - q0);
    t_acc = (vmax - v0) / amax;
    q_acc = v0 * t_acc + 0.5 * amax * t_acc^2;
    t_dec = vmax / amax;
    q_dec = vmax * t_dec - 0.5 * amax * t_dec^2;
    if q_acc + q_dec < delta_q
        t_const = (delta_q - (q_acc + q_dec)) / vmax;
    else
        t_const = 0;
        t_acc = sqrt(2 * delta_q / amax);
        t_dec = t_acc;
    end
    t_total = t_acc + t_const + t_dec;
    q = q0;
    v = v0;
    a = 0;
    if t < t_acc
        q = q0 + direction * (v0 * t + 0.5 * amax * t^2);
        v = v0 + direction * amax * t;
        a = direction * amax;
    elseif t < t_acc + t_const
        q = q0 + direction * (q_acc + vmax * (t - t_acc));
        v = direction * vmax;
        a = 0;
    elseif t <= t_total
        t_rel = t - t_acc - t_const;
        q = q0 + direction * (q_acc + vmax * t_const + vmax * t_rel - 0.5 * amax * t_rel^2);
        v = direction * (vmax - amax * t_rel);
        a = -direction * amax;
    else
        q = qf;
        v = 0;
        a = 0;
    end
end
%%
function [xyz_pos,xyz_v , xyz_a] = KRUG(time, radius, total_time)

    omega = 2 * pi / total_time;
    theta = omega * time;
    x = radius * cos(theta);
    y = radius * sin(theta);
    z = -0.6;
    xyz_pos = [x, y, z];

    vx = -radius * omega * sin(theta);
    vy = radius * omega * cos(theta);
    vz = 0;
    xyz_v = [vx, vy, vz];
    
    ax = -radius * omega^2 * cos(theta);
    ay = -radius * omega^2 * sin(theta);
    az = 0;
    xyz_a = [ax, ay, az];
end


%%
function [A,C,G,T, T_mot, T_gn, T_gb, Tb_out, Tn_out] = Robot(q,dq,ddq, din_par,J,dJ)

g = din_par(1);
I = din_par(2);
mnt = din_par(3);
mbr = din_par(4);
Rgb = din_par(5);

i = din_par(6);

T_gn = transpose(J)*mnt*[0;0;-g];

%T_gb = Rgb*mbr*g*[cos(q(1));cos(q(2));cos(q(3))];
T_gb = Rgb*mbr*g*cos(q(:));


Ib = [I,0,0;0,I,0;0,0,I];
A = (Ib + mnt*transpose(J)*J)*ddq(:);
C = (transpose(J)*mnt*dJ)*dq(:);
G = (T_gn + T_gb);
T = A + C - G;
T_mot = A/i +  C/i - G/i; 


Tb_out = Ib*ddq(:);
Tn_out = transpose(J)*mnt*(dJ*dq(:) + J*ddq(:));
T_test = Tb_out + Tn_out - T_gn - T_gb;

end
%%
function [J,dJ] = RETURN_JACOBIAN(q,xyz,kinematika,dq)

x = xyz(1);
y = xyz(2);
z = xyz(3);

theta1 = q(1);
theta2 = q(2);
theta3 = q(3);

a = kinematika(1);
b = kinematika(2);
c = kinematika(3);
L = kinematika(4);

A = [x,y + a + L*cos(theta1),z + L*sin(theta1);
    2*(x + b) - sqrt(3)*L*cos(theta2), 2*(y + c) - L*cos(theta2), 2*(z + L*sin(theta2));
    2*(x - b) + sqrt(3)*L*cos(theta3), 2*(y + c) - L*cos(theta3), 2*(z + L*sin(theta3))];

b11 = L*((y + a)*sin(theta1) - z*cos(theta1));
b22 = -L*((sqrt(3)*(x + b) + y + c)*sin(theta2) + 2*z*cos(theta2));
b33 = L*((sqrt(3)*(x - b) - y - c)*sin(theta3) - 2*z*cos(theta3));

B = [b11, 0, 0;
    0, b22, 0;
    0, 0, b33];

J = inv(A)*B;

%disp(size(J));

v = J*[dq(1);dq(2);dq(3)];
dx = v(1);
dy = v(2);
dz = v(3);

dtheta1 = dq(1);
dtheta2 = dq(2);
dtheta3 = dq(3);

dA = [dx, dy - L*sin(theta1)*dtheta1, dz + L*cos(theta1)*dtheta1;
      2*dx + L*sqrt(3)*sin(theta2)*dtheta2, 2*dy + L*sin(theta2)*dtheta2, 2*dz + 2*L*cos(theta2)*dtheta2;
      2*dx - sqrt(3)*L*sin(theta3)*dtheta3, 2*dy + L*sin(theta3)*dtheta3, 2*dz + 2*L*cos(theta3)*dtheta3];


db11 = L*(dy*sin(theta1) + y*cos(theta1)*dtheta1) + L*a*cos(theta1)*dtheta1 - L*(dz*cos(theta1) - z*sin(theta1)*dtheta1); %novi prvi dio
db22 = -L*((sqrt(3)*dx + dy)*sin(theta2) + (c + sqrt(3)*(b + x) + y)*cos(theta2)*dtheta2 - 2*z*sin(theta2)*dtheta2 + 2*cos(theta2)*dz);
db33 = L*((sqrt(3)*dx - dy)*sin(theta3) + (-c + sqrt(3)*(-b + x) - y)*cos(theta3)*dtheta3 + 2*z*sin(theta3)*dtheta3 - 2*cos(theta3)*dz);
  
dB = [db11,0,0;
      0,db22,0;
      0,0,db33];
  
dJ = inv(A)*(-dA*J  + dB);
end
%%
function m_max = vakuum(a)
Fp = 50;
mi = 0.4;
S = 2;
g = 9.81;
m_max = Fp/(S*(g + a(3) + (1/mi)*(abs(a(1)) + abs(a(2)))));
end

function [xyz_pos, xyz_v, xyz_a] = KINEM_TEST(time, radius, total_time)
    omega = 1.5 * pi / total_time;
    theta = omega * time;
    
    x = radius * cos(theta);
    y = radius * sin(theta);
    z = -0.65 + 0.05 * cos(theta);
    xyz_pos = [x, y, z];
    

    vx = -radius * omega * sin(theta);
    vy = radius * omega * cos(theta);
    vz = -0.05 * omega * sin(theta);
    xyz_v = [vx, vy, vz];
    
    ax = -radius * omega^2 * cos(theta);
    ay = -radius * omega^2 * sin(theta);
    az = -0.05 * omega^2 * cos(theta);
    xyz_a = [ax, ay, az];
end