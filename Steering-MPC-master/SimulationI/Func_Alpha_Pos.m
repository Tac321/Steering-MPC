
function K=Func_Alpha_Pos(Xb,Yb,Xn,Yn)  
AngleY=Yn-Yb; 
AngleX=Xn-Xb;
K=atan2(AngleY,AngleX);
if(K<0)
    K=K+2*pi;
elseif(K>2*pi)
    K=K-2*pi;
end
    


