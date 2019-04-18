  function [c,ceq] = constraintNL(x,Ak,Bk,Ck,Dk,State_Initial,Np,max_slip,max_steerchange)
 
for i =1:1:Np+1
    gr(i)=x(i); 
end

% Let gr(Np+1) be epsilon

for i =1:1:Np
    if i == 1      
        xi_h(:,i)=State_Initial;
        alph_h(1)=atan( Ck*xi_h(:,i) );  %
    else
        if(i>2 && i<=Np-2 && Np >4)
            delu(i-2)= gr(i) - gr(i-1);
        end
        delta_f(i,1) = gr(i);
        xi_h(:,i)= Ak*xi_h(:,i-1)+Bk*[delta_f(i,1);1]; % This makes the constraints linear, Nonlinear constraint update equation requires symbolic math engine, note: Maple is very powerful!
        alph_h(i)= atan( Ck*xi_h(:,i-1)+Dk*delta_f(i,1) ); 
    end
end

% if(Np>4)
%     c = [alph_h-max_slip;-alph_h-max_slip;delta_f'-max_steerchange;-delta_f'-max_steerchange]; % Careful!
% else
%     c = [alph_h-max_slip;-alph_h-max_slip;];
% end
c = [alph_h-(max_slip+gr(Np+1));-alph_h-(max_slip+gr(Np+1) ); ones(1,Np)*(-gr(Np+1))];
ceq = [];
    
    
    