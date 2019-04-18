function [ F,dF ] = Pacejka_F_dF(slip,B,C,D,E)
    % This function matches that of the Pacejka equation in the Volvo paper.
    F=D*sin(C*atan(B*slip-E*(B*slip-atan(B*slip))) );  % Equation (4) where X == slip, Y== side force
    numerator=-D*C*(-B+E*(B-B/(B^2*slip^2+1)))*cos(C*atan(-B*slip+E*(B*slip-atan(B*slip))));
    denominator=(-B*slip+E*(B*slip-atan(B*slip)))^2+1;
    dF=numerator/denominator;
end

