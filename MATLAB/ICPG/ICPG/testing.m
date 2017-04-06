% guidance_params = {350,22,0.95};
current_state = {2050,1200,65000,3380,468};
body_params = {3.986004418e14,6371000};

target_state = {7700,0,165000};
weights = {1,1,0.1};

% final_state = icpg_eval(guidance_params,current_state,body_params);
% icpg_cost(target_state,weights,guidance_params,current_state,body_params);


% Minimize cost function to find [burnTime,angle,thrust]
cost = @(x) icpg_cost(target_state,weights,{x(1),x(2),x(3)},current_state,body_params);
% best_guidance = fmincon(cost,[0,0,0.5],...
%     [],[],[],[],...
%     [0,0,0.5],[500,45,1]);                                                % [lowerBound],[upperBound]

best_guidance= [216.440730812824 7.34027871492903 1.76148919008061];
% Integrate flight trajectory
guidance_params = {(0:ceil(best_guidance(1))),best_guidance(2),best_guidance(3)};
time = guidance_params{1};
final_state = icpg_eval(guidance_params,current_state,body_params);
best_final_state = [final_state{1}(end),final_state{2}(end),final_state{4}(end)];
plot(final_state{3}./1000,final_state{4}./1000);