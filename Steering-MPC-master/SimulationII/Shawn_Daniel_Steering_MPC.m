%% Steering MPC Referencing Falcone Paper
%  Author : Shawn Daniel, shawncd@umich.edu
% 10/22/2018

clc
close all
clear all

tic
%% Constant Vehicle Parameters 
m=2050;                 %Vehicle mass (kg) [provided]
J=3344;                 %Vehicle yaw inertia (kg/m^2) [provided]
a=1.045;                %CG to front axle (m)
b=1.453;                %CG to rear axle (m)
dFy_f=70000;            %Front Wheel Cornering Stiffness (N/rad)
dFy_r=55000;            %Rear Wheel Cornering Stiffness (N/rad)
Fn_f=b/(a+b)*2050*9.81; %Front axle normal load (N)
Fn_r=a/(a+b)*2050*9.81; %Rear axle normal load (N)

% Assuming actual vehicle has constant longitudinal velocity.
xdot=15;                %Longitudinal Speed (m/s)




%% Pacejka Tire Model
% friction Coefficient
mu=[0.1,0.3,0.5,0.7,0.9]; 
mu_t=mu(2); %Choose fixed friction for the simulation
%Front Wheel Pacejka Constants
p_Df=mu_t*Fn_f/2; 
peakslip=mu_t*0.17*(70/55); 
asym=0.90*p_Df;
p_Cf=1+(1-2/pi*asin(asym/p_Df));
if p_Cf<=1
    p_Cf=1-(1-2/pi*asin(asym/p_Df));
end
p_Bf=dFy_f/(p_Cf*p_Df);
p_Ef=(p_Bf*peakslip-tan(pi/2/p_Cf))/(p_Bf*peakslip-atan(p_Bf*peakslip));
%Rear Wheel Constants
p_Dr=mu_t*Fn_r/2;
peakslip=mu_t*0.17;
asym=0.90*p_Dr;
p_Cr=1+(1-2/pi*asin(asym/p_Dr));
if p_Cr<=1
    p_Cr=1-(1-2/pi*asin(asym/p_Dr));
end
p_Br=dFy_r/(p_Cr*p_Dr);
p_Er=(p_Br*peakslip-tan(pi/2/p_Cr))/(p_Br*peakslip-atan(p_Br*peakslip));

%% Controller 

Ts=0.05;         %Sample time
simtime=20;      %Simulation time
t=0:Ts:simtime;  %Global time
Hp=6;           %Prediction Horizon
Hc=3;           %Control Horizon
max_steer=10*pi/180; % bon
max_steerchange=0.85*pi/180;
max_slip=2.2*pi/180; % bon
Q=[10,0,0;0,200,0;0,0,10];
Q1=Q(1,1);
Q2=Q(2,2);
Q3=Q(3,3);
R=50000;
rho=1000;

xi_ac=zeros(4,length(t));   %Actual Vehicle States [ydot, psi_dot, psi, Y]
alph_ac=zeros(2,length(t)); %Actual Slip Angles,
u_ac=zeros(1,length(t));    %Actual Input
Xac=zeros(1,length(t));     %Actual global longitudinal position
Xdot_temp=xdot;

eta_ref=zeros(3,Hp,length(t)); %Inertial frame Reference Trajectories
eta_shift=eta_ref;             %Transformed Trajecories (vehicle local frame)
eta_true=zeros(3,length(t));   %Simulated Vehicle Trajectores

dx1=25;                        %Constants that describe the "base" reference trajectory
dx2=21.95;
dy1=4.05;
dy2=5.7;

%Initialize LTV State Matrices
Ak=zeros(4,4);             %
Bk=zeros(4,2);            
Ck=[-1/xdot,-a/xdot,0,0];  
Dk=1;                      
Ak(3:4,:)=[0,Ts,1,0;Ts,0,Ts*xdot,1]; 

%Realtime Obstacle Avoidance Section:
Nobs=6;
X_obstacle=zeros(Nobs,1);
X_obstacle(1:2)=10;
X_obstacle(3:4)=15;
X_obstacle(5:6)=12.5;
X_obstacle(7)=5.3;
Y_obstacle=zeros(Nobs,1);
Y_obstacle(1)=-1.65;
Y_obstacle(2)=-1.6;
Y_obstacle(3)=-1.65;
Y_obstacle(4)=-1.6;
Y_obstacle(5)=-1.65;
Y_obstacle(6)=-1.6;
Y_obstacle(7)=0.70655;
S=15;

%% Simulation commence
for i=2:length(t)-1
    % Inertial/global reference trajectory
    z1=2.4/25*(Xac(i)-27.19)-1.2;
    z2=2.4/21.95*(Xac(i)-56.46)-1.2;
    eta_true(1,i)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2)); %Y
    eta_true(2,i)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2)); %psi 
    %Generate estimation of reference inertial longitudinal position (assume constant X_dot)
    X_cur=Xac(i);  
    % Reference trajectory to track defined below, within a finite horizon,
    % the same length as out MPC  control horizon.
    for j=1:Hp
        z1=2.4/25*(X_cur+Xdot_temp*t(j)-27.19)-1.2;
        z2=2.4/21.95*(X_cur+Xdot_temp*t(j)-56.46)-1.2;
        eta_ref(1,j,i)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2)); %Y_ref
        eta_ref(2,j,i)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2)); %psi_ref
        eta_ref(3,j,i)=-(.2304*dy1*Xdot_temp*sinh(0.96e-1*X_cur+0.96e-1*Xdot_temp*t(j)-3.81024)...
            /(cosh(0.96e-1*X_cur+0.96e-1*Xdot_temp*t(j)-3.81024)^3*dx1)...
            -.2624145785*dy2*Xdot_temp*sinh(.1093394077*X_cur+.1093394077*Xdot_temp*t(j)-7.373302959)/...
            (cosh(.1093394077*X_cur+.1093394077*Xdot_temp*t(j)-7.373302959)^3*dx2))/...
            ((-1.2*dy1/(cosh(0.96e-1*X_cur+0.96e-1*Xdot_temp*t(j)-3.810240000)^2*dx1)...
            +1.2*dy2/(cosh(.1093394077*X_cur+.1093394077*Xdot_temp*t(j)-7.373302959)^2*dx2))^2+1); %r_ref
    end
    %Transform reference trajectory to car body frame
    
    % Below is our error frame calculation of the system sp
    for j=1:Hp
        eta_shift(2,j,i)=(eta_ref(2,j,i)-xi_ac(3,i)); % psi_ref
        eta_shift(1,j,i)=-Xdot_temp*t(j)*sin(xi_ac(3,i))+...%Y_ref
            (eta_ref(1,j,i)-xi_ac(4,i))*cos(xi_ac(3,i));
        eta_shift(3,j,i)=eta_ref(3,j,i); % psi_dot_ref
    end
     % Calculate the actual slip angles of the front and rear tires
    alph_ac(1,i)=atan(Ck*xi_ac(:,i)+Dk*u_ac(i-1));
    alph_ac(2,i)=atan([-1/xdot,b/xdot]*xi_ac(1:2,i));
    
%    Compute nonlinear cornering force (only) of the tires, front and back.
    [Fc_f,dFy_f]=Pacejka_F_dF(alph_ac(1,i),p_Bf,p_Cf,p_Df,p_Ef);  % Note the outputs are funcitons only of the "slip_front_tire_assembly"
    [Fc_r,dFy_r]=Pacejka_F_dF(alph_ac(2,i),p_Br,p_Cr,p_Dr,p_Er);  % " ' ' " "slip_rear_tire_assembly"
    F0_f = Fc_f - dFy_f*alph_ac(1,i); % Net cornering force as a function of the tire slip, steering wheel turn and the steering wheel turning rate.
    F0_r = Fc_r - dFy_r*alph_ac(2,i);
    % create LTV model of the nonlinear system
    % NOTE: We ignore longitudinal force, assuming velocity is constant.
    
    % Below is the nonolinear equaiton.
     % Note: Equations below are [2x2]
    bike_sys=[-2*(dFy_f+dFy_r)/(m*xdot),-2*(dFy_f*a-dFy_r*b)/(m*xdot)-xdot;...   % A
        -2*(dFy_f*a-dFy_r*b)/(J*xdot),-2*(dFy_f*a^2+dFy_r*b^2)/(J*xdot)]
    bike_in=2*[dFy_f/m,(F0_f+F0_r)/m;(dFy_f*a)/J,(F0_f*a-F0_r*b)/J]   % B
    bike_out=[1,0;0,1]  % C 
    bike_ff=0  % D
    
    
    %Convert from continuous time to discrete time
    sys=ss(bike_sys,bike_in,bike_out,bike_ff)
    sysd=c2d(sys,Ts);  
    [bike_sysd,bike_ind,bike_outd,bike_ffd]=ssdata(sysd);
    Ak(1:2,1:2)=bike_sysd
    Bk(1:2,1:2)=bike_ind
    
    % Constraints for the MPC control tracker
    maxAcc = 1.2;
    lb=ones(Hp,1)*-max_steer;
    ub=ones(Hp,1)*max_steer;  
    lb = [lb];
    ub = [ub];
    Amat=[];
    bmat=[];
    Aeq=[];
    beq=[];
    options = optimset('Algorithm','active-set'); % 34.692404->active-set, interior-point<-62.394862, 29.549879 ->sqp: Nice!!!
    [U,fval,exitflag]= fmincon(@(x)MY_costfunction(x,[xi_ac(1,i);xi_ac(2,i);0;0],Hp,Hc,Ts,eta_shift(1,:,i),eta_shift(2,:,i),...
        eta_shift(3,:,i),Q,R,Ak,Bk,xdot,t(i),Nobs,X_obstacle,Y_obstacle,S,rho),zeros(Hp+1,1),Amat,bmat,Aeq,beq,...
        lb,ub,@(x)constraintNL(x,Ak,Bk,Ck,Dk,[xi_ac(1,i);xi_ac(2,i);0;0],Hp,max_slip,max_steerchange),options);

    %Set Steer input for next time step based on optimal prediction
    u_ac(i)=U(2);
    
    % Update system states 
    xi_ac(1:3,i+1)=Ak(1:3,1:3)*xi_ac(1:3,i)+Bk(1:3,:)*[u_ac(i);1];
    xi_ac(4,i+1)=xi_ac(4,i)+Ts*(xdot*sin(xi_ac(3,i))+xi_ac(1,i)*cos(xi_ac(3,i)));
    Xdot_temp=xdot*cos(xi_ac(3,i))-xi_ac(1,i)*sin(xi_ac(3,i));
    Xac(i+1)=Xac(i)+Ts*(Xdot_temp); 
end

% Final Trajectory Step
z1=2.4/25*(Xac(end)-27.19)-1.2;
z2=2.4/21.95*(Xac(end)-56.46)-1.2;
eta_true(1,end)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
eta_true(2,end)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));

%Numerical differentiation of yaw angle
for i=2:length(t)-1
    eta_true(3,i)=(eta_true(2,i+1)-eta_true(2,i-1))/(2*Ts);
end

%% 8.4 Error results

%Calculate residuals
res=[xi_ac(4,:)-eta_true(1,:);xi_ac(3,:)-eta_true(2,:);xi_ac(2,:)-eta_true(3,:)];



%% 9. Plotted Results at current settings (15 m/s) 
figure
subplot(4,1,1)
hold on
plot(t,xi_ac(4,:),'LineWidth',2)
plot(t,eta_true(1,:),':','color','r','LineWidth',2)
legend('simulation','reference')
ylabel('Y [m]')
plot(X_obstacle,Y_obstacle,'ro')
hold off
subplot(4,1,2)
hold on
plot(t,xi_ac(3,:)*180/pi,'LineWidth',2)
plot(t,eta_true(2,:)*180/pi,':','color','r','LineWidth',2)
ylabel('\psi [deg]')
hold off
subplot(4,1,3)
hold on
plot(t,xi_ac(2,:)*180/pi,'LineWidth',2)
plot(t,eta_true(3,:)*180/pi,':','color','r','LineWidth',2)
ylabel('d \psi/dt [deg/s]')
hold off
subplot(4,1,4)
hold on
plot(t,u_ac*180/pi,'LineWidth',2)
ylabel('u [deg]')
xlabel('time [s]')
hold off

plot(t,xi_ac(1,:))
toc
%%
% Fin




