function [final_xvel,final_yvel,final_x,final_y] = usg_eval(t,xvel,yvel,y0,g,Ve,tau,theta)
    body_rad = 6371000;
    final_xvel = Ve.*log(tau./(tau-t)).*cos(theta) + xvel;
    final_yvel = Ve.*log(tau./(tau-t)).*sin(theta) + ((xvel.*xvel)./(body_rad+y0)-g).*t + yvel;
    final_x = -Ve.*((tau-t).*log(tau./(tau-t))-t).*cos(theta) + xvel.*t;
    final_y = -Ve.*((tau-t).*log(tau./(tau-t))-t).*sin(theta) + 0.5.*((xvel.*xvel)./(body_rad+y0)-g).*t.*t + yvel.*t + y0;
end

