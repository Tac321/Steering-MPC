% Simplified border stuff below , without obstacle avoidance, so I could
% under stand what is going on

function [new_b_array, last_border_array]= getNewBorders(traj,borders,track_center,X_all,n_cars,caridx,MPC_vars,ModelParams)
% this function will replace the carinsight function
%
% Inputs:
%   traj        - pathinfo (gotten from splinify)
%   borders     - trackinfo (gotten from splinify)
%   track_center- track.center (where track is what went through splinify)
%   X_all       - state vector of all cars
%   n_cars      - total number of cars
%   carindex    - index of controlled car
%   MPC_vars    - MPC parameters
%   ModelParams - Model parameters
%
% Outputs:
%   new_b_array - [N]x[4]x[n_cars] matrix containing the points to define
%                 corridors for each car
%  last_border_array - [ncars]x[3] matrix containing the points to define
%                 corridors for each car
%
% Pseudo code:
%
% for each car (caridx)
%   for all other cars (othercaridx)
%       if othercaridx is "close enough" to caridx
%           populate grid   % For the take over maneuver
%       end
%   end
% end
% 
% Note: X is length (N+1)*nx, while grid has N rows. The i'th row of grid
% corresponds to the (i+1)'th section of X. That is, x0 does NOT have a
% corresponding line in grid.

% design parameters
DesignParameters.N = MPC_vars.N;
DesignParameters.Ts = MPC_vars.Ts;
DesignParameters.grid_width = 12; % track width is split into grid_width+1 equal sections,   Take over maneuver resolution

%   with margin
DesignParameters.carwidth =  ModelParams.W*2; %[m]
DesignParameters.carlength = ModelParams.L*2; %[m] 

lookahead = X_all(MPC_vars.N*ModelParams.nx+ModelParams.stateindex_theta,1) - X_all(ModelParams.stateindex_theta,1);
DesignParameters.thetadifferencethreshold=lookahead*1.25; %[m] (used by CarIsClose() )
DesignParameters.Cost_dead_end = 5000;
DesignParameters.Cost_maxanglechange = 2000; % cost when maxanglechange is exceeded. Choose a cost that is greater than Cost_dead_end. used in getCost_AngleChange()%
DesignParameters.Cost_Weights = [0.5 3 0.5]; % [deviation length anglechange]
DesignParameters.time_step_cost_discount_factor = 0.95;

% misc. constants
Constants.UNOCCUPIED=0; % fill value of grid_isOccupiedBy if unoccupied (used by getOptimalPathMatrix() and Find_Nearest_Borders() )
Constants.DEAD_END_PATH = -2; % fill value of Optm_Path_Mtx if no valid paths (used by getOptimalPathMatrix() and Find_Nearest_Borders() )
Constants.LAST_BORDER_XMAX = 10.0; % [m] largest value for x, used as default last_border

nx = ModelParams.nx;
N = MPC_vars.N;

% %%%%%%%%%%%%% end of control variables %%%%%%%%%%%%%%%%%%%%

grid_width = DesignParameters.grid_width;

% misc. defines
tracklength = traj.ppx.breaks(end);

% declare variable
% new_b_array=zeros(N,4); %[N]x[4]x[n_cars] matrix 
last_border_array=zeros(3,n_cars); %[3]x[n_cars] matrix 


% grid coordinates for current car,   % For obstacle avoidance only.
X = X_all(:,caridx); % full state vector of current car
[~, ~, new_b_default] = getGridCoordinates(traj,borders,track_center,X,DesignParameters,ModelParams);

% new_b_array{caridx} is a [N]by[4] matrix. Each row contains two
% (x,y) coordinates marking the corridor

    % then use default borders
    new_b_array = new_b_default;
    last_border_array = 777; % x<=10    

end



function [grid_x, grid_y, new_b_default] = getGridCoordinates(traj,borders,track_center,X,DesignParameters,ModelParams)
% assumes x,y are the 1 and 2 states

nx=ModelParams.nx;
N=DesignParameters.N;
grid_width=DesignParameters.grid_width; % 12

tracklength = traj.ppx.breaks(end);

% find physical thetas for each future position in X
theta_phys=zeros(N,1);
for i=1:N   % Forward into the horizon.
xy_posn=X(i*nx+1:i*nx+2);% assume x,y are the 1 and 2 states,  De incoming MPC state. Ignoreing the initial state x0
theta_virt = X(nx*i+nx); % The thetaV state, not the initial one.

% approximate theta_phys as theta_virt: should hold if lag ERROR weighting is high enough
theta_virt=mod(theta_virt,tracklength); % Preventrs "wrap around" incresing length
theta_phys(i)=theta_virt;   % tHIS ASSUMES THAT OUR THETA PHYSICAL ACTUAL TRAJECTORY WILL BE EQUAL TO THE VIRTUAL ONE. Our projection along the reference track.

end

TrackLeftx=ppval(borders.pplx,theta_phys); % x coord of the left track/inner track border constraint
TrackLefty=ppval(borders.pply,theta_phys); % y coord "  ' ' "
TrackRightx=ppval(borders.pprx,theta_phys);
TrackRighty=ppval(borders.ppry,theta_phys);
Dx = TrackRightx - TrackLeftx;
Dy = TrackRighty - TrackLefty;

% border if no car is near
new_b_default = [TrackLeftx TrackLefty TrackRightx TrackRighty]; % Yes! I like!
% SIZOb = size(new_b_default);  % [np x 4] : matrix de mpc

grid_x = zeros(N,grid_width); % Only for the obstacle avoidance problem.
grid_y = zeros(N,grid_width);


end
 




