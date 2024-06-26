function  kf = kalmanINS(kf, dt,i,FUZZ,ti)


kf.A =  expm(kf.F * dt);
kf.Qd = (kf.G * kf.Q * kf.G') .* dt;
kf.xi = kf.A * kf.xp;
kf.Pi = (kf.A * kf.Pp * kf.A') + kf.Qd;
kf.Pi =  0.5 .* (kf.Pi + kf.Pi');
kf.S = (kf.R + kf.H * kf.Pi  * kf.H');				
kf.v =  kf.z - kf.H * kf.xi; 						
kf.K = (kf.Pi * kf.H') * (kf.S)^(-1) ;				
kf.xp = kf.xi + kf.K * kf.v; 
kf.Pp = kf.Pi - kf.K * kf.S *  kf.K';                
kf.Pp =  0.5 .* (kf.Pp + kf.Pp'); 

kf = FA_EKF(kf,FUZZ,ti);
end


