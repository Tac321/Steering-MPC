% NOTE : If u get the error, optimizer has too many inputs, close matlab and
% openit again.

%% MPCC Simulation Script
clear
close all
clc 
addpath('splines');
addpath('~/Documents/GitHub/hpipm/interfaces/matlab/hpipm_matlab')

%% Load Parameters
CarModel = 'ORCA';
% CarModel = 'FullSize';

MPC_vars = getMPC_vars(CarModel);
ModelParams=getModelParams(MPC_vars.ModelNo);
% choose optimization interface options: 'Yalmip','CVX','hpipm','quadprog'
MPC_vars.interface = 'quadprog';

nx = ModelParams.nx;
nu = ModelParams.nu;
N = MPC_vars.N;
Ts = MPC_vars.Ts;
%% import an plot track
% use normal ORCA Track
load Tracks/track2.mat
% use RCP track
% load Tracks/trackMobil.mat
% track2 = trackMobil;
% shrink track by half of the car widht plus safety margin
% TODO implement orientation depending shrinking in the MPC constraints
safteyScaling = 1.5*1.4*2;  % *1.4  % Choose zero such that the borders are the actual physical boundaries of the track
[track,track2] = borderAdjustment(track2,ModelParams,safteyScaling);  % Morphed by the above constant, scalor.

trackWidth = norm(track.inner(:,1)-track.outer(:,1));
% plot shrinked and not shrinked track 
figure(1);
plot(track.outer(1,:),track.outer(2,:),'r')
hold on
plot(track.inner(1,:),track.inner(2,:),'r')
plot(track2.outer(1,:),track2.outer(2,:),'k')
plot(track2.inner(1,:),track2.inner(2,:),'k')
%% Simulation lenght and plotting
simN = 500;
%0=no plots, 1=plot predictions
plotOn = 1;
%0=real time iteration, 1=fixed number of QP iterations, 2=fixed number of damped QP iterations
QP_iter = 0;
% number of cars 
% there are two examples one with no other cars and one with 4 other cars
% (inspired by the set up shown in the paper)
% n_cars = 1; % no other car
n_cars = 5; % 4 other cars
%% Fit spline to track
% TODO spline function only works with regular spaced points.
% Fix add function which given any center line and bound generates equlally
% space tracks.
[traj, borders] =splinify(track);
tl = traj.ppy.breaks(end);  % Normalized length of the track.

% store all data in one struct
TrackMPC = struct('traj',traj,'borders',borders,'track_center',track.center,'tl',tl);
%% Define starting position
startIdx = 1; %point (in terms of track centerline array) allong the track 
% where the car starts, on the center line, aligned with the track, driving
% straight with vx0
%since the used bicycle model is not well defined for slow velocities use vx0 > 0.5
if CarModel == "ORCA"
    vx0 = 1;
elseif CarModel == "FullSize"
    vx0 = 15;
end


% Restudy THIS!
% find theta that corresponds to the 10th point on the centerline
[theta, ~] = findTheta([track.center(1,startIdx),track.center(2,startIdx)],track.center,traj.ppx.breaks,trackWidth,startIdx);
thetagrr=theta  % This is zero!

% Initiallize at the track center location.
x0 = [track.center(1,startIdx),track.center(2,startIdx),... % point on centerline
      atan2(ppval(traj.dppy,theta),ppval(traj.dppx,theta)),... % aligned with centerline
      vx0 ,0,0,theta]'; %driving straight with vx0, and correct theta progress
    
% the find theta function performs a local search to find the projection of
% the position onto the centerline, therefore, we use the start index as an
% starting point for this local search
last_closestIdx = startIdx;
%% First initial guess:
% Push the box car forward to populate the initial guess of (xmpc) to   
% prevent it from beign all zeros. NOT REALLY NECCESARY but it makes you
% feel good.

x = repmat(x0,1,N+1); % all points identical to current measurment  [nx x Np+1]

% first inital guess, all points on centerline aligned with centerline
% spaced as the car would drive with vx0.
for i = 2:N+1
    theta_next = x(ModelParams.stateindex_theta,i-1)+Ts*vx0;  
    phi_next = atan2(ppval(traj.dppy,theta_next),ppval(traj.dppx,theta_next));
    % phi_next can jump by two pi, make sure there are no jumps in the
    % initial guess
    if (x(ModelParams.stateindex_phi,i-1)-phi_next) < -pi
        phi_next = phi_next-2*pi;
    end
    if (x(ModelParams.stateindex_phi,i-1)-phi_next) > pi
        phi_next = phi_next+2*pi;
    end
    x(:,i) = [ppval(traj.ppx,theta_next),ppval(traj.ppy,theta_next),... % point on centerline
              phi_next,... % aligned with centerline
              vx0 ,0,0,theta_next]'; %driving straight with vx0, and correct theta progress
end

u = zeros(3,N); % zero inputs
uprev = zeros(3,1); % last input is zero
%% Ohter cars
Y = ObstacelsState(track,traj,trackWidth,n_cars);

if size(Y,2) ~= n_cars-1
    error('n_cars and the number of obstacles in "Y" does not match')
end
%% Initialize logging arrays
X_log = zeros(nx*(N+1),simN);
U_log = zeros(3*N,simN);
B_log = zeros(4*N,simN);
qpTime_log = zeros(1,simN);
% Feedback terms
%% initializtion
% solve problem 5 times without applying input
% inspiered by sequential quadratic programming (SQP)
for i = 1:5
    % formulate MPCC problem and solve it
    Iter_damping = 0.5; % 0 no damping.
    
    [x_up, u_up, b, exitflag,info] = optimizer(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev );
%     x = Iter_damping*x + (1-Iter_damping)*x_up;
    u = Iter_damping*u + (1-Iter_damping)*u_up; % populate the umpc so it's initial guess isn't alll zeros.

    if plotOn == 1
        % plot predictions
        PlotPrediction(x,u,b,Y,track,track2,traj,MPC_vars,ModelParams)
    end
end
%% Simulation
for i = 1: simN*3.5/3.5
%     ii=i
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% MPCC-Call %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % augment state and inputs by shifting previus optimal solution
    [x,u] = augState(x,u,x0,MPC_vars,ModelParams,tl);

    [x, u, b, exitflag,info] = optimizer(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev); 
        % From this point on "x,a nd umpc are just good initial guesses for the newton method to operate on!
        
    qpTime_log(i) = info.QPtime;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% simulate system %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                     % not Xmpc but x0 here, for real.
    x0 = SimTimeStep(x(:,1),u(:,1),Ts,ModelParams)' % x_kp1 = f(xk), x(t) the current position of our car.
     x0 = unWrapX0(x0); % Force/ constraint the convention definint the
%      attitude of the vehicle through out entire simulaiton of 
%      flight/action/drive/whateva

    [ theta, last_closestIdx] = findTheta(x0,track.center,traj.ppx.breaks,trackWidth,last_closestIdx);
    x0(ModelParams.stateindex_theta) = theta;
    uprev = u(:,1); % This is 'u0' for the next iteration, u's previous state.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% plotting and logging %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if plotOn == 1
        PlotPrediction(x,u,b,Y,track,track2,traj,MPC_vars,ModelParams)
    end
    
    % log predictions and time
    X_log(:,i) = reshape(x,(N+1)*7,1);
    U_log(:,i) = reshape(u,(N)*3,1);
    B_log(:,i) = reshape(b,N*4,1);
    
    
end


PlotLog( X_log,U_log,Y,track,track2,simN,Ts)

%% Generating Stats
a = 1;
for i=1:simN-1
    if X_log(ModelParams.stateindex_theta,i+1) - X_log(ModelParams.stateindex_theta,i) < -0.9*tl
        LapStart(a) = i;
        a = a+1;
    end
end

if length(LapStart) > 1
    LapTime = diff(LapStart)*Ts;
else
    LapTime = NaN;
end

disp('------------------------------------')
disp(['Lap Time(s): ',num2str(LapTime)])
disp('------------------------------------')
disp(['Mean Computation Time: ',num2str(mean(qpTime_log))])
disp(['Max Computation Time: ',num2str(max(qpTime_log))])
disp(['Min Computation Time: ',num2str(min(qpTime_log))])
disp('------------------------------------')











