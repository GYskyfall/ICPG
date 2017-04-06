function [cost] = cost_fun(params,final_state,t,theta,thrust,weights)
    [xvel,yvel,~,y] = usg_eval(t,params(1),params(2),params(3),params(4),params(5),params(6)./thrust,theta);
    xvel_error = ((final_state(1)-xvel)./weights(1)).^2;
    yvel_error = ((final_state(2)-yvel)./weights(2)).^2;
    y_error = ((final_state(3)-y)./weights(3)).^2;
    cost = xvel_error + yvel_error + y_error;
end