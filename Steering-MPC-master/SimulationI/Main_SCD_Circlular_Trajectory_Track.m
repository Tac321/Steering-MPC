% Holonomic Car MPC Steering control
%  Author : Shawn Daniel, shawncd@umich.edu
% 10/22/2018

% NOTE: At locations of "O PLAY", changing  the parameter values shows the
% robustness of MPC tracking or the reference trajectory

clc;
close all;
clear all;

%% Reference trajectory generation

tic

Nx=3;%Number of states
l=1; 
N=100*3; % Timeline discrete length. O PLAY
Np=30;%Prediction time horizon
Nc=20;%Control time horizon
T=0.05;%The sampling period
XXX=zeros(N,Nx*Np);

L=2.85;%wheelbase of car.
CEN=[0,10];       % center, of circular reference trajectory
Radius=10;       % radius, of circular reference trajectory
X0 = [0 0 pi/2]; % initial state,  of actual car system, O PLAY
vd1 = 1.3;  % Note this is constant body frame velocity of reference 
            % trajectory car

%% System initial states
% Play with this so see how MPC chases it's target trajectory and turns
% the real system to acheive reference trajectory convergence ASAP,
% considering the precense of system constraints
State_Initial=zeros(Nx,1);%state=[y_dot,x_dot,phi,Y,X],Here is the given initial value
State_Initial(1,1)=0;%x 
State_Initial(2,1)=0;%y
State_Initial(3,1)=pi/2;%phi
Q=diag([10 200 10])*3;
R=diag([100 100]);
 % Inertial frame defined ref traj.
[RefTraj_x,RefTraj_y,RefTraj_theta,RefTraj_delta]=Func_CircularReferenceTrajGenerate(X0(1),X0(2),CEN(1),CEN(2),Radius,N,vd1,T,L);
Xref(:,1)=RefTraj_x;  
Yref(:,1)=RefTraj_y;
PHIref(:,1)=RefTraj_theta;


%% Control Loop
for j=1:1:N
    % Max angular velocity acheivable by the system
    maxTurn = 45*pi/180; 
    % Max body frame longitudinal velocity
    maxAcc = 1.2;
    % MPC Constraints
    lb=[0;-maxTurn;0;-maxTurn]*1;
    ub=[maxAcc;maxTurn;maxAcc;maxTurn]*1;
    A=[];
    b=[];
    Aeq=[];
    beq=[];
    options = optimset('Algorithm','active-set');
    if(j+Np <= N)
        Xr= Xref(j:j+Np-1,1)     ;
        Yr= Yref(j:j+Np-1,1)     ;
        Psir = PHIref(j:j+Np-1,1);
    else
        Xr = [Xref(j:end,1)     ];
        Yr = [Yref(j:end,1)     ];
        Psir = [PHIref(j:end,1) ];
        if(size(Xr,1)<Np)
            for i= 1:(Np-size(Xr,1))
                Xr = [Xr ;Xref(end,1)];
                Yr = [Yr ;Yref(end,1)];
                Psir = [Psir ;PHIref(end,1)];
            end
        end
       
    end
    [A,fval,exitflag]= fmincon(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Xr,Yr,Psir,Q,R),    [1;0.1;1;0.1;],A,b,Aeq,beq,lb,ub,[],options); %Constrained solution, but slow,
    [~,X_predict,Y_predict]=MY_costfunction(A,State_Initial,Np,Nc,T,Xr,Yr,Psir,Q,R);
    % The two controllers are Acceleration/Breaking and steering respectivl
    playSlowasCarAllowedToGo=0*(vd1+.1); %Try setting higher than vd1 O PLAY
    v_actual=playSlowasCarAllowedToGo+A(1);  
    deltaf_actual=A(2);
    % State Update
    X00(1)=State_Initial(1,1);
    X00(2)=State_Initial(2,1);
    X00(3)=State_Initial(3,1);
    XOUT=dsolve('Dx-v_actual*cos(z)=0','Dy-v_actual*sin(z)=0','Dz-v_actual*tan(deltaf_actual)=0','x(0)=X00(1)','y(0)=X00(2)','z(0)=X00(3)');
    t=T;
    State_Initial(1,1)=eval(XOUT.x);
    State_Initial(2,1)=eval(XOUT.y);
    State_Initial(3,1)=eval(XOUT.z);
    
    figure(1)
    plot(State_Initial(1,1),State_Initial(2,1),'b*');
    hold on;
    plot(Xref,Yref,'r--');
    hold on;
    plot(Xr,Yr,'y-');
    hold on
    plot(X_predict,Y_predict,'g-')
end 

 toc 
 
 