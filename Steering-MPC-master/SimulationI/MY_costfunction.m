
function [cost,X_predict,Y_predict] = MY_costfunction(x, State_Initial, Np, Nc,T,Xref,Yref,PHIref,Q,R)
l = 1;
X=State_Initial(1,1);
Y=State_Initial(2,1);
PHI=State_Initial(3,1);
X_predict = zeros(Np,1);
Y_predict = zeros(Np,1);
PHI_predict = zeros(Np,1);
v = zeros(Np,1); %Control amount v, The longitudinal VELOCITY due to  accelerator and brakes.
delta_f = zeros(Np,1); %Control amount anglar velocity of the car

% Prediction Horizon update
for i =1:1:Np
    if i == 1
        v(i,1) = x(1);
        delta_f(i,1) = x(2);
        X_predict(i,1) = X +T*v(i,1)*cos(PHI);
        Y_predict(i,1) = Y +T*v(i,1)*sin(PHI);
        PHI_predict(i,1)=PHI+T*v(i,1)*tan(delta_f(i,1))/l;
    else
        v(i,1)=x(3); 
        delta_f(i,1)=x(4);
        X_predict(i,1)=X_predict(i-1)+T*v(i,1)*cos(PHI_predict(i-1)); % Note here the update of the eestimated state used Euler, or 1st degree derive Taylor approximation fo rhtis LTV system.
        Y_predict(i,1)=Y_predict(i-1)+T*v(i,1)*sin(PHI_predict(i-1));
        PHI_predict(i,1)=PHI_predict(i-1)+T*v(i,1)*tan(delta_f(i,1))/l;
    end
end

cost = Q(1,1)*sum_square(X_predict(:,1)-Xref(:,1)) ...
        +Q(2,2)*sum_square(Y_predict(:,1)-Yref(:,1)) ...
        +Q(3,3)*sum_square(PHI_predict(:,1)-PHIref(:,1)) ;%...
        +R(1,1)*sum_square(v(1:Nc,1))...
        +R(2,2)*sum_square(delta_f(1:Nc,1));
 




