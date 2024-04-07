function [roll,pitch,yaw] = Attitude_corrections(roll,pitch,yaw,KALMAN,i)
        roll(i)  = roll(i)  - KALMAN.xp(1);
        pitch(i) = pitch(i) - KALMAN.xp(2);
        yaw(i)   = yaw(i)   - KALMAN.xp(3);
        
end