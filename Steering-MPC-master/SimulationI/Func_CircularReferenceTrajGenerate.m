function [RefTraj_x,RefTraj_y,RefTraj_theta,RefTraj_delta]=Func_CircularReferenceTrajGenerate(Pos_x,Pos_y,CEN_x,CEN_y,Radius,N,Velo,Ts,L)
    %RefTraj is the reference path to be generated

    RefTraj=zeros(N,4);
    Alpha_init=Func_Alpha_Pos(CEN_x,CEN_y,Pos_x,Pos_y);
    Omega=Velo/Radius;
    DFWA=atan(L/Radius);  
    for k=1:1:N
        Alpha(k)=Alpha_init+Omega*Ts*(k-1);  % Hardcode reference trajectory vehicle polar coord "theta"
        RefTraj(k,1)=Radius*cos(Alpha(k))+CEN_x;%x  % Hard code ref vehicle X(t)
        RefTraj(k,2)=Radius*sin(Alpha(k))+CEN_y;%y  % " " Y(t)
        RefTraj(k,3)=Func_Theta_Pos(Alpha(k));%psi_ref
        RefTraj(k,4)=DFWA;%Front wheel yaw angle, can be used
    end

    RefTraj_x= RefTraj(:,1);
    RefTraj_y= RefTraj(:,2);
    RefTraj_theta= RefTraj(:,3);
    RefTraj_delta= RefTraj(:,4);
end