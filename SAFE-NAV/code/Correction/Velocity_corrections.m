function [vel] = Velocity_corrections(vel,KALMAN,i)
        
        vel (i,1) = vel (i,1) - KALMAN.xp(4);
        vel (i,2) = vel (i,2) - KALMAN.xp(5);
        vel (i,3) = vel (i,3) - KALMAN.xp(6);
        
end