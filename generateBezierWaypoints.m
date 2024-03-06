% x0, y0, theta0 = starting pose of robot
% vt = constant local forward linear velocity of robot
% L = distance from robot starting point to person
% d = signaling distance
% c = clearance (lateral passing distance)
% k = curvature of bezier curve
% n = number of points in each motion segment
function waypoints = generateBezierWaypoints(x0, y0, theta0, vt, L, d, c, k, n)
    % Generate waypoints for the straight line 1
    straight_line1 = [linspace(x0, x0 + (L - d)*cos(theta0), n); linspace(y0, y0 + (L - d)*sin(theta0), n); theta0*ones(1, n)];

    % Generate waypoints for the curvature-defined Bezier curve
    start_point_bezier = [x0 + (L - d)*cos(theta0); y0 + (L - d)*sin(theta0); theta0];
    end_point_bezier = [x0 + L*cos(theta0) - c*sin(theta0); y0 + L*sin(theta0) + c*cos(theta0); theta0];

    control_point1 = [x0 + (L - d + k*d)*cos(theta0); y0 + (L - d + k*d)*sin(theta0); theta0];
    control_point2 = [x0 + L*cos(theta0) - c*sin(theta0) - k*d*cos(theta0); y0 + L*sin(theta0) + c*cos(theta0) - k*d*sin(theta0); theta0];

    t = linspace(0, 1, n); % Parameter values between 0 and 1
    b = @(t) (1-t).^3 .* start_point_bezier + ...
              3*(1-t).^2.*t .* control_point1 + ...
              3*(1-t).*t.^2 .* control_point2 + ...
              t.^3 .* end_point_bezier;

    bezier_curve = zeros(3, n);
   

    for i = 1:n
        bezier_curve(:,i) = b(t(i));                  
       if i ~= 1
        bezier_curve(3,i) = atan2(bezier_curve(2,i)-bezier_curve(2,i-1), ...
            bezier_curve(1,i)-bezier_curve(1,i-1)); 
       end
    end

    % Generate waypoints for the straight line 2 (move 1 meter past the
    % participant
    extra = 1;
    straight_line2 = [linspace(x0 + L*cos(theta0) - c*sin(theta0), x0 + (L + extra)*cos(theta0) - c*sin(theta0), n); linspace(y0 + L*sin(theta0) + c*cos(theta0), y0 +(L + extra)*sin(theta0) + c*cos(theta0), n); theta0*ones(1, n)];

    % Generate waypoints for the straight line 3 (circle back to the
    % starting point)
    offset = 1;
    straight_line3 = [linspace( x0 + (L + extra)*cos(theta0) - c*sin(theta0), x0 + (L + extra)*cos(theta0) - (c + offset) * sin(theta0), n); linspace(y0 +(L + extra)*sin(theta0) + c*cos(theta0), y0 +(L + extra)*sin(theta0) + (c + offset)*cos(theta0), n); theta0*ones(1, n)];

    % Generate waypoints for the straight line 4
    straight_line4 = [linspace(x0 + (L + extra)*cos(theta0) - (c + offset) * sin(theta0), x0 - (c+offset) * sin(theta0), n); linspace(y0 +(L + extra)*sin(theta0) + (c + offset)*cos(theta0), y0 + (c + offset) * cos(theta0), n); theta0*ones(1, n)];

    % Generate waypoints for the straight line 5
    straight_line5 = [linspace(x0 - (c+offset) * sin(theta0), x0, n); linspace(y0 + (c + offset) * cos(theta0), y0, n); theta0*ones(1, n)];

    % Concatenate waypoints for the straight line and the Bezier curve
    waypoints = [straight_line1(:,1:end-1), bezier_curve, straight_line2(:,1:end-1), straight_line3(:,1:end-1), straight_line4(:,1:end-1), straight_line5(:,1:end-1)];
        
   
    [~, totalL] = size(waypoints);
     distPoints = zeros(1, totalL);
    deltaT = zeros(1, totalL);
    omegas = zeros(1, totalL);

 for i = 1:totalL-1
        distPoints(i) = sqrt( (waypoints(1,i+1) - waypoints(1,i))^2 ...
            + (waypoints(2, i+1) - waypoints(2, i))^2);
        deltaT(i) = distPoints(i) / vt;
        omegas(i) = (waypoints(3, i+1) - waypoints(3, i))...
            / deltaT(i);
 end

    vtArr = zeros(1, totalL);
    vtArr(1, :) = vt;

    waypoints = [waypoints; omegas; vtArr];
  %waypoints(3, n/2) = 0;


    % Display the waypoints
    disp('Generated Waypoints:');
    disp(waypoints);

    % Plot the straight line and Bezier curve with waypoints
    figure;
    plot(x0, y0, 'go', 'MarkerSize', 10, 'LineWidth', 2);
    hold on;
    plot(end_point_bezier(1), end_point_bezier(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(control_point1(1), control_point1(2), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
    plot(control_point2(1), control_point2(2), 'mo', 'MarkerSize', 8, 'LineWidth', 2);
    plot(waypoints(1, :), waypoints(2, :), 'ro-', 'LineWidth', 2);
    title('Path with Straight Line and Curvature-Defined Bezier Curve');
    xlabel('X-axis');
    ylabel('Y-axis');
    grid on;
    axis equal;
    legend('Start Point', 'End Point', 'Control Point 1', 'Control Point 2', 'Waypoints');
    hold off;
end