function [t_series,way_traj,psi_traj]=min_energy_traj_series(waypts,psi_pts,T)
    n=length(psi_pts)-1;
    t_series=[0];
    way_traj=[waypts(:,1)];
    psi_traj=[psi_pts(1)];
    
    for i=1:n
        rs=waypts(:,i);
        re=waypts(:,i+1);

        ts=T/n*(i-1);
        te=T/n*i;

        psi_s=psi_pts(i);
        psi_e=psi_pts(i+1);
        
        out=min_energy_traj(ts,te,rs,re,psi_s,psi_e);
        
        t=transpose(out.STATES(:,1));
        t(1)=[];
        t_series=[t_series,t];
        
        x=transpose(out.STATES(:,2));
        x(1)=[];
        
        y=transpose(out.STATES(:,4));
        y(1)=[];
        
        z=transpose(out.STATES(:,6));
        z(1)=[];
        
        r=[x;y;z];
        way_traj=[way_traj,r];
        
        psi=transpose(out.STATES(:,11));
        psi(1)=[];
        psi_traj=[psi_traj,psi];
    end
        
end 