function [lat,lon,h,roll,pitch,yaw] = ZeroVelocity(STAT,dti,i,vel,lat,lon,h,roll,pitch,yaw)
idz = floor( STAT.zupt_win / dti ); 
    
    
    if ( i > idz )
        
        vel_m = mean (vel(i-idz:i , :));
        
       if (abs(vel_m) <= STAT.zupt_th)
            
            roll(i) = mean (roll(i-idz:i , :));
            pitch(i)= mean (pitch(i-idz:i , :));
            yaw(i)  = mean (yaw(i-idz:i , :));
            
            lat(i) = mean (lat(i-idz:i , :));
            lon(i) = mean (lon(i-idz:i , :));
            h(i)   = mean (h(i-idz:i , :));
          
        end
    end
end