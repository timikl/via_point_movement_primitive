classdef Polynomial
    properties
        x0
        dx0
        ddx0
        x1
        dx1
        ddx1
        dt
        t
        trajectory
        time_split
        a
        velocity
        acceleration
        jerk
    end
    
    methods
        function obj = Polynomial(x0, dx0, ddx0, x1, dx1, ddx1, t, dt)
            obj.x0 = x0;
            obj.dx0 = dx0;
            obj.ddx0 = ddx0;
            obj.x1 = x1;
            obj.dx1 = dx1;
            obj.ddx1 = ddx1;
            obj.dt = dt;
            obj.t = t;
            obj.time_split = linspace(0, obj.t, round(obj.t / obj.dt));
            
            a0 = x0;
            a1 = dx0;
            a2 = ddx0 / 2;
            a3 = (20 .* obj.x1 - 20 .* obj.x0 - (8 .* obj.dx1 + 12 .* obj.dx0) .* t - (3 .* obj.ddx0 - obj.ddx1) .* t.^2) ./ (2 .* t.^3);
            a4 = (30 .* obj.x0 - 30 .* obj.x1 + (14 .* obj.dx1 + 16 .* obj.dx0) .* t + (3 .* obj.ddx0 - 2 .* obj.ddx1) .* t.^2) ./ (2 .* t.^4);
            a5 = (12 .* obj.x1 - 12 .* obj.x0 - (6 .* obj.dx1 + 6 .* obj.dx0) .* t - (obj.ddx0 - obj.ddx1) .* t.^2) ./ (2 .* t.^5);
            
            obj.a = [a5, a4, a3, a2, a1, a0];
        end
        
        function y = trajectory_at_time(obj, t)
            y = obj.a(1) .* t.^5 + obj.a(2) .* t.^4 + obj.a(3) .* t.^3 + obj.a(4) .* t.^2 + obj.a(5) .* t + obj.a(6);
        end
        
        function obj = whole_trajectory_calculate(obj)
            obj.trajectory = zeros(1, length(obj.time_split));
            for j = 1:length(obj.time_split)
                obj.trajectory(j) = obj.trajectory_at_time(obj.time_split(j));
            end
        end
        
        function obj = velocity_calculate(obj)
            obj.velocity = [obj.dx0];  % Initialize with square brackets
            for j = 2:length(obj.time_split)
                obj.velocity(j) = (obj.trajectory(j) - obj.trajectory(j - 1)) / obj.dt;
            end
        end
        
        function obj = acceleration_calculate(obj)
            obj.acceleration = [obj.ddx0];  % Initialize with square brackets
            for j = 2:length(obj.time_split)
                obj.acceleration(j) = (obj.velocity(j) - obj.velocity(j - 1)) / obj.dt;
            end
        end
        
        function obj = jerk_calculate(obj)
            obj.jerk = zeros(1, length(obj.time_split));  % Initialize with zeros
            for j = 2:length(obj.time_split)
                obj.jerk(j) = (obj.acceleration(j) - obj.acceleration(j - 1)) / obj.dt;
            end
        end
    end
end

