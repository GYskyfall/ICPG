current_state = {2150,1100,65000,9,3380,468};                               % current_state = [xvel0,yvel0,y0,g,Ve,tau];
final_state = {7700,0,165000};                                               % final_state = [xvel,yvel,y];
weights = [0.05,0.05,50];                                                   % weights dividing [xvel,yvel,y]


% Minimize cost function to find [burnTime,angle,thrust]
cost = @(x) cost_fun(current_state,final_state,x(1),x(2),x(3),weights);
best_guidance = fmincon(cost,[0,0,0.5],...
    [],[],[],[],...
    [0,0,0.5],[500,deg2rad(45),1]);                                                % [lowerBound],[upperBound]


% Integrate flight trajectory
time = 1:best_guidance(1);
[final_xvel,final_yvel,final_x,final_y] = usg_eval(time,current_state(1),current_state(2),...
                                                   current_state(3),current_state(4),current_state(5),...
                                                   current_state(6)/best_guidance(3),best_guidance(2));
best_guidance = [best_guidance(1),rad2deg(best_guidance(2)),best_guidance(3)];
best_final_state = [final_xvel(end),final_yvel(end),final_y(end)];
plot(final_x./1000,final_y./1000)
