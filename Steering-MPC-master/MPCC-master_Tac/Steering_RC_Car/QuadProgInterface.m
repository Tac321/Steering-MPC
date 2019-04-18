% Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific languag e governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ X,U,dU,info] = QuadProgInterface(stage,MPC_vars,ModelParams)
nx = ModelParams.nx;
nu = ModelParams.nu;
ng = 2;
nz = nx+2*nu;
nxu = nx+nu;
N = MPC_vars.N;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H = zeros(nz*N+nxu,nz*N+nxu);
f = zeros(nz*N+nxu,1);
for i = 1:N+1
    if i<N+1   %   from 0, N-1
        H_i = blkdiag(stage(i).Qk,stage(i).Rk); % [13x13]
        H((i-1)*nz+[1:nz],(i-1)*nz+[1:nz]) = H_i; % This Overall matrix will be block diagonal.  [10x1]
        f((i-1)*nz+[1:nxu+3]) = [stage(i).fk;0;0;0]; % last 3 values will be 0's for the delta Control terms.
    else% for i=N+1 crop of the delta U term.
        H((i-1)*nz+[1:nxu],(i-1)*nz+[1:nxu]) = stage(i).Qk;
        f((i-1)*nz+[1:nxu]) = stage(i).fk;  %bon
    end
end

H = 0.5*(H+H');

Aeq = zeros(nxu*(N+1),nz*N+nxu); % I get it., N+1 to compensate for the initial state and controller of the real system.  % Like this, system has dynamic lag, omponent, it doesn't just telepor tthe steering wheel where we want.
beq = zeros(nxu*(N+1),1);

x0scale = blkdiag(MPC_vars.Tx,MPC_vars.Tu)*[stage(1).x0;stage(1).u0];
Aeq(1:nxu,1:nxu) = eye(nxu);  % First [10x10] block of Aeq is identity.
beq(1:nxu) = x0scale;   


for i = 1:N 
    if i<N  
        Aeq((i)*nxu+[1:nxu],(i-1)*nz+[1:2*nz]) = [-stage(i).Ak,-stage(i).Bk,eye(nxu),zeros(nxu,nu)];
        beq((i)*nxu+[1:nxu]) = stage(i).gk;
    else    
        Aeq((i)*nxu+[1:nxu],(i-1)*nz+[1:(nz+nxu)]) = [-stage(i).Ak,-stage(i).Bk,eye(nxu)];
        beq((i)*nxu+[1:nxu]) = stage(i).gk;
    end 
end 
 
% A = [];%ones(1,nz*N+nxu);
% b = [];%zeros(ng*N,1);
% ng == 2 
A = zeros(ng*N,nz*N+nxu);  % [80 x 530]
b = zeros(ng*N,1);         % [80 x 1]

% Below just ensures, simplistically that the x_mpc lies between the
% boundaries of the virtual shrunk track, very clever!
for i = 1:N
    A((i-1)*ng+1,i*nz+[1:nxu]) = stage(i+1).Ck;% for i=1, row 1, columns 13+10 = 23, stage(i+1) skipping the initial state value within the x_mpc
    b((i-1)*ng+1) = stage(i+1).ug;
    
    A((i-1)*ng+2,i*nz+[1:nxu]) = -stage(i+1).Ck;
    b((i-1)*ng+2) = -stage(i+1).lg;
end

LB = zeros(nz*N+nxu,1); % The bounds for "z"
UB = zeros(nz*N+nxu,1);

for i=1:N+1
    if i<N+1
        LB((i-1)*nz+[1:nz]) = stage(i).lb;
        UB((i-1)*nz+[1:nz]) = stage(i).ub;
    else
        LB((i-1)*nz+[1:nxu]) = stage(i).lb(1:nxu);
        UB((i-1)*nz+[1:nxu]) = stage(i).ub(1:nxu);
    end
end

options = optimoptions(@quadprog,'MaxIterations',100,'Display','off');
tic

% Note : We formed "A, Aeq, b, beq" with the initial guess "x_mpc"
% Warmstarted at the commence of the MPC formulation. THE ONLY REQUIREMENT
% % MAJOR IS THAT U_MPC IS NOT ALL 0's!

% [z,~,exitflag] = quadprog(H,f,A,b,Aeq,beq,LB,UB,[],options);

% Use SCD/Istvan method below:
ub = UB;  % Same at ub = []
lb = LB; % "  ''  "
Anet = [Aeq;A];
ru= [beq; b];   % "  ''  "
rl= [beq;-b];  % CAREFUL!
[m,n] = size(Anet);
Aadmm = [Anet; speye(n)];
uu = [ru; ub];
ll = [rl; lb];
[z, ~,~,J,exitflag] = IPM_scdII(H,f,Aadmm,ll,uu);

QPtime = toc;

% Make the logging, extraction variables , to   plot horizon estimate
% trajectory forward in time.
X = zeros(nx,N+1);
U = zeros(nu,N);
dU = zeros(nu,N);

% Extract the state|x_kp1, U and the dU from the augmented system,
% if exitflag == 1
    for i = 1:N+1
        X(1:nx,i) = MPC_vars.invTx*z((i-1)*nz+[1:nx]);  % @ i = N+1, z(

        if i>1
            U(1:nu,i-1) = MPC_vars.invTu*z((i-1)*nz+nx+[1:nu]);  % Ignoring the very first entry on U to get the corresponding dUk with it's Uk for all subsequent iterations?
        end

        if i<=N
            dU(1:nu,i) = z((i-1)*nz+nxu+[1:nu]);  % cropped off for i = N+1
        end     
    end
% end

info.QPtime = QPtime;
% % % if exitflag == 1
% % %     info.exitflag = 0; % com'on man this freaking unneccisarly confusing!
% % % else
% % %     info.exitflag = 1;
% % % end

end
