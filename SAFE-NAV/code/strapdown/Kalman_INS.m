function [EST_n,E_est_n] = Kalman_INS (E_est,E_meas,EST,MEA)

KG = E_est./(E_est+E_meas);
EST_n = EST + KG.*(MEA-EST);
E_est_n=[1-KG].*(E_est);