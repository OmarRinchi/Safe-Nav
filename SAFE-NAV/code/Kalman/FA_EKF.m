function  [kf,S,A_index] = FA_EKF(kf,ti,i,error_Q,Q_mat,STATE,ACTION)
kf.L=kf.L+1;
kf.V = [kf.V kf.v];
if (kf.L>=15)
kf.mat=0;
kf.mean=0;
    for y=1:15
    kf.mat = kf.mat + kf.V(:,kf.L-15+y)*kf.V(:,kf.L-15+y)';
    kf.mean = kf.mean + kf.V(:,kf.L-15+y);
    end
    kf.mat=kf.mat/15;
    kf.mean=sum(kf.mean/15);
    ROR=trace(kf.S) - trace(kf.mat);
    ROR_mean=kf.mean;
    kf.TT=[kf.TT ti];
    kf.ROR = [kf.ROR ROR];

    %% Q learning
    differences = abs(ROR - STATE);
    [minDifference, S] = min(differences);
    selectedRow = Q_mat(S,:);
    [cash, A_index] = max(selectedRow);
    A=ACTION(A_index);
    alpha=A;
    %%
    %alpha=evalfis([ROR],FUZZ);
    kf.Pp=kf.Pp*alpha^1.3;

else
    A_index = randi(length(ACTION));
    S = randi(length(STATE));
end

end