function [lat,lon,h] = Position_corrections(lat,lon,h,KALMAN,i)
        lat(i) = lat(i) - (KALMAN.xp(7));
        lon(i) = lon(i) - (KALMAN.xp(8));
        h(i)   = h(i)   - KALMAN.xp(9);
        
end