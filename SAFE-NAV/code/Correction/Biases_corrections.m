function [gb_dyn,ab_dyn] = Biases_corrections(gb_dyn,ab_dyn,KALMAN,i)
        gb_dyn   = KALMAN.xp(10:12);
        ab_dyn   = KALMAN.xp(13:15);
end