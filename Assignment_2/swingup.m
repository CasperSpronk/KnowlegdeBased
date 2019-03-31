function [par, ta, xa] = swingup(par)

    par.simtime = 10;     % Trial length
    par.simstep = 0.05;   % Simulation time step
    par.maxtorque = 1.5;  % Maximum applicable torque
    
    
    if strcmp(par.run_type, 'learn')
        %%
        % Obtain SARSA parameters
        par = get_parameters(par);
        
		% TODO: Initialize the outer loop

        % Initialize bookkeeping (for plotting only)
        ra = zeros(par.trials, 1);
        tta = zeros(par.trials, 1);
        te = 0;

        % Outer loop: trials
        for ii = 1:par.trials

            % TODO: Initialize the inner loop

            % Inner loop: simulation steps
            for tt = 1:ceil(par.simtime/par.simstep)
                
                % TODO: obtain torque
                
                % Apply torque and obtain new state
                % x  : state (input at time t and output at time t+par.simstep)
                % u  : torque
                % te : new time
                [te, x] = body_straight([te te+par.simstep],x,u,par);

                % TODO: learn
                % use s for discretized state
                
                % Keep track of cumulative reward
                ra(ii) = ra(ii)+reward;

                % TODO: check termination condition
            end

            tta(ii) = tta(ii) + tt*par.simstep;

            % Update plot every ten trials
            if rem(ii, 10) == 0
                plot_Q(Q, par, ra, tta, ii);
                drawnow;
            end
        end
        
        % save learned Q value function
        par.Q = Q;
 
    elseif strcmp(par.run_type, 'test')
        %%
        % Obtain SARSA parameters
        par = get_parameters(par);
        
        % Read value function
        Q = par.Q;
        
        x = swingup_initial_state();
        
        ta = zeros(length(0:par.simstep:par.simtime), 1);
        xa = zeros(numel(ta), numel(x));
        te = 0;
        
        % Initialize a new trial
        s = discretize_state(x, par);
        a = execute_policy(Q, s, par);

        % Inner loop: simulation steps
        for tt = 1:ceil(par.simtime/par.simstep)
            % Take the chosen action
            TD = max(min(take_action(a, par), par.maxtorque), -par.maxtorque);

            % Simulate a time step
            [te,x] = body_straight([te te+par.simstep],x,TD,par);

            % Save trace
            ta(tt) = te;
            xa(tt, :) = x;

            s = discretize_state(x, par);
            a = execute_policy(Q, s, par);

            % Stop trial if state is terminal
            if is_terminal(s, par)
                break
            end
        end

        ta = ta(1:tt);
        xa = xa(1:tt, :);
        
    elseif strcmp(par.run_type, 'verify')
        %%
        % Get pointers to functions
        learner.get_parameters = @get_parameters;
        learner.init_Q = @init_Q;
        learner.discretize_state = @discretize_state;
        learner.execute_policy = @execute_policy;
        learner.observe_reward = @observe_reward;
        learner.is_terminal = @is_terminal;
        learner.update_Q = @update_Q;
        learner.take_action = @take_action;
        par.learner = learner;
    end
    
end

% ******************************************************************
% *** Edit below this line                                       ***
% ******************************************************************
function par = get_parameters(par)
    % TODO: set the values
    par.epsilon = 0.1;      % Random action rate
    par.gamma = 0.99;       % Discount rate
    par.alpha = 0.25;       % Learning rate
    par.pos_states = 30;    % Position discretization
    par.vel_states = 30;    % Velocity discretization
    par.actions = 5;        % Action discretization
    par.trials = 200;       % Learning trials
end

function Q = init_Q(par)
    % TODO: Initialize the Q table.
    Q = zeros(par.pos_states,par.vel_states,par.actions);
end

function s = discretize_state(x, par)
    % TODO: Discretize state. Note: s(1) should be
    % TODO: position, s(2) velocity.
    positionRange = (pi/15)*par.pos_states;
    %modulate position between edge posibilities
    position = mod(x(1),positionRange); %modulate position between -2pi and 2pi
    position = position / positionRange;% set position value between 0 and 1
    discreet_position = ceil(position * par.pos_states); % set position to values between 1 and par.pos_states
    % set discreet position to 1 if position was equal to exactly 0
    if discreet_position == 0
        discreet_position = 1;
    end
    %clip to edge velocity
    velocityEdge = (pi/3)*par.vel_states/2; % set velocity edge
    velocity = x(2);
    velocity(velocity > velocityEdge) = velocityEdge; % set velocities that exceed the max velocity to the max velocity
    velocity(velocity < -velocityEdge) = -velocityEdge; % set velocities that exceed the min velocity to the min velocity
    velocity = (velocity + velocityEdge)/(2*velocityEdge); % set velocity value between 0 and 1
    discreet_velocity = ceil(velocity * par.vel_states); % set position to values between 1 and par.vel_states
    if discreet_velocity == 0 % makes sure that a velocity value of exaclty -5*pi will be in range of par.vel_states 
        discreet_velocity = 1;
    end
    %return value
    s = [discreet_position, discreet_velocity];
end

function u = take_action(a, par)
    % TODO: Calculate the proper torque for action a. This cannot
    % TODO: exceed par.maxtorque.
    a = a - par.actions/2 - 0.5; % splits actions into positve and negative torque
    a = a / (par.actions / 2 - 0.5); % sets actions to range between -1 and 1 
    u = a * par.maxtorque; % multiplies by maxtorque to set the value of the torque
end

function r = observe_reward(a, sP, par)
    % TODO: Calculate the reward for taking action a,
    % TODO: resulting in state sP.
    if sP(1) == 15 && sP(2) == 15
        r = 10;
    else
        r = 0;
    end
end

function t = is_terminal(sP, par)
    % TODO: Return 1 if state sP is terminal, 0 otherwise.
    if sP(1) == 15 && sP(2) == 15
        t = 1;
    else
        t = 0;
    end
end


function a = execute_policy(Q, s, par)
    % TODO: Select an action for state s using the
    % TODO: epsilon-greedy algorithm.
end

function Q = update_Q(Q, s, a, r, sP, aP, par)
    % TODO: Implement the SARSA update rule.
end

