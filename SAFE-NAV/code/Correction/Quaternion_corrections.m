function [q,DCMbn] = Quaternion_corrections(q_update,KALMAN)
antm = [0 q_update(3) -q_update(2); -q_update(3) 0 q_update(1); q_update(2) -q_update(1) 0];
        q = q_update + 0.5 .* [q_update(4)*eye(3) + antm; -1.*[q_update(1) q_update(2) q_update(3)]] * KALMAN.xp(1:3);
        q = q / norm(q);
[DCMbn] = quaternion2DCMbn (q);        
end