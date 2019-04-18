function [cost,X_predict,Y_predict] = MY_costfunction(x, State_Initial, Np, Nc,T,...
    etaref1,etaref2,etaref3,Q,R,Ak,Bk,xdot,tiempo,Nobs,X_obstacle,Y_obstacle,S,rho)
cost = 0; 
l = 1;
% Y, psi, psi_doot
delta_f = zeros(Np,1); %Control amount delta_f
J_obst=zeros(Np,1);
Ts = 0.05;
for i =1:1:Np+1
    gr(i)=x(i);
end

% gr(Np+1) be epsilon

for i =1:1:Np
    if i == 1        
        xi_h(:,1)=State_Initial;
        for j=1:1:Nobs
            J_obst(i,1)=J_obst(i,1)+1/( sqrt((tiempo-X_obstacle(j,1))^2+ (xi_h(4,i)-Y_obstacle(j,1))^2) +0.000001);  % So the future trajectory will be buffered to avoid the obstacles
        end
    else
        if(i>2 && i<=Np-2 && Np >4)
            delu(i-2)= gr(i) - gr(i-1);
        end
        delta_f(i,1) = gr(i);
        xi_h(:,i)= Ak*xi_h(:,i-1)+Bk*[delta_f(i,1);1];  % This makes the MPC linear, making this Nonlinear state update requires symbolic math engine, increasing time and effectiveness of algorithm!
        tiempo= tiempo + Ts;
        for p=1:1:Nobs
            J_obst(i,1)=J_obst(i,1)+1/( sqrt((tiempo-X_obstacle(p,1))^2+ (xi_h(4,i)-Y_obstacle(p,1))^2 )+0.000001);  % So the future trajectory will be buffered to avoid the obstacles
        end
    end
end

eta_h= [0,0,0,1;0,0,1,0;0,1,0,0]*xi_h;

if(Np > 4)
    cost = Q(1,1)*sum_square(eta_h(1,:)-etaref1) ...
        +Q(2,2)*sum_square(eta_h(2,:)-etaref2) ...
        +Q(3,3)*sum_square(eta_h(3,:)-etaref3) ...
        +R*sum_square(delu(1:Np-4)) ...
        +S*sum(J_obst(:)) ...
        +rho*gr(Np+1);
else
    cost = Q(1,1)*sum_square(eta_h(1,:)-etaref1) ...
        +Q(2,2)*sum_square(eta_h(2,:)-etaref2) ...
        +Q(3,3)*sum_square(eta_h(3,:)-etaref3) ...
        +S*sum(J_obst(:)) ...
        +rho*gr(Np+1);
end
 

