% NOW-HOW
% Inverted Pendulum Part:0
% Cart and Pendulum

clc
clear
close all


% Running the whole program
inverted_pendulum_0


function inverted_pendulum_0()
    % Going through cart and pendulum ball animation
    
    % Getting the screensize
    screenSize = get(0, 'ScreenSize');
    
    % Create a figure to capture key presses
    fig = figure('Name', 'Press ''q'' to close this figure', ...
                 'KeyPressFcn', @keyPress);
    % Setting figure to the whole screen size
    set(fig, 'OuterPosition', screenSize);

    % Our figure will have 2 plots
    % 1 row, 2 columns
    % 3rd number corresponds to the index of the subplot we want to
    % interact
    subplot(1,2,1)
    % X and Y labels
    xlabel("X Position [m]", FontSize= 28);
    ylabel("Y Position [m]", FontSize= 28);
    % Title of the figure
    title("Cart Dynamics",FontSize=28)
    % Makes axis 1 unit equal, so circle doesn't lose its shape
    axis equal
    % axis setting limits
    ylim([-30 30]);
    xlim([-100 100]);

    % Formatting axes
    hAxes = gca;
    hAxes.XAxis.FontSize = 16;
    hAxes.YAxis.FontSize = 16;
    % Drawing line as rail for the cart
    yline(-13,'-k',LineWidth=4)
    % Putting grid on the figure (Square Boxes)
    grid on
    
    % Interacting with subplot 2
    subplot(1,2,2)
    % X Y labels
    xlabel("X Position [m]", FontSize= 28);
    ylabel("Y Position [m]", FontSize= 28);
    % Title of the figure
    title("Pendulum Ball Dynamics",FontSize=28)
    % Makes axis 1 unit equal, so circle doesn't lost its shape
    axis equal
    % axis setting limits
    ylim([-50 50]);
    xlim([-50 50]);
    % Setting axes
    hAxes = gca;
    hAxes.XAxis.FontSize = 16;
    hAxes.YAxis.FontSize = 16;
    % Putting grid on the figure (Square Boxes)
    grid on

    % Implementing the dynamics and animating them
    dynamics()

    % This loop will keep running until the figure is manually closed
    while ishandle(fig)
        pause(0.1);  % Short pause to keep the loop manageable
    end

    % Close the window when q is pressed
    function keyPress(src, event)
        % Key press callback function
        if strcmp(event.Key, 'q')  % Check if the 'q' key was pressed
            close all;  % Call the close figure function
            % error("Closed Window!")
        end
    end
end


function dynamics()
    M=10;       % Cart mass               [kg]
    m=5;        % Ball mass               [kg]
    L=20;       % Pendulum length         [m]
    B_M = 10;  % Cart Friction coeff     [kg]
    B_m = 10;  % Ball Friction coeff     [kg*m^2]
    g=9.81;     % Gravitational Accel     [m/sec^2]
    u = 5;      % Input Force             [N = kg*m/sec^2]
    T = 0;
    % T = -m*g*L;
    
    % Running the simulation for 200 seconds
    tend = 20;
    tinc = 0.001;
    tspan = 0:tinc:tend;
    
    % Initial Conditions for the cart and ball
    % Cart Position, Cart Velocity
    y0_cart = [-50,0];
    % Ball angle, ball angular velocity
    y0_ball = [pi/2,0];
    
    % Running the dynamics for the cart
    [t, y] = ode45(@(t,y) cart_dynamics(y,M,B_M,u),tspan,y0_cart);
    % Assigning cart time and y
    % y_cart(:,1) x position
    % y_cart(:,2) v velocity
    t_cart = t;
    y_cart = y;

    % Running the dynamics for the ball
    [t,y] = ode45(@(t,y) ball_dynamics(y,m,L,B_m,g,T),tspan,y0_ball);
   % Assigning ball time and y
    % y_ball(:,1) theta angle
    % y_ball(:,2) w angular velocity
    t_ball = t;
    y_ball = y;
    
    subplot(1,2,1)
    draw_cart(t,y_cart(:,1))

    
    subplot(1,2,2)
    draw_ball(t,y_ball(:,1),L)

end

function dy = cart_dynamics(y,M,B_M,u)
    % States
    x = y(1,1);
    v = y(2,1);

    % Defining the incremental changes
    dy(1,1) = v;
    dy(2,1) = (u- B_M*v)/M;
end

function dy = ball_dynamics(y,m,L,B_m,g,T)
    % States
    th = y(1,1);
    w = y(2,1);

    % Defining the incremental changes
    dy(1,1) = w;
    dy(2,1) = (T - B_m*w + m*g*L*sin(th))/m;
end


function draw_ball(t, theta,L)
    % Animating the Ball
    % Ball Center
    xc = L*sin(theta(1,1));
    yc = L*cos(theta(1,1));
    % Drawing Ball, and Line
    ball = circle(xc,yc,6,'b','k');
    line = drawLine(0,xc,0,yc);
    % Pause for 1 seconds
    pause(1)

    % Changing title
    hText=findall(gca, 'Type', 'text');
    newText = sprintf('t = %.2f s', 0);
    set(hText(1), 'String', newText);  % Update text and shift position to follow the time


    % Running 200 seconds in steps of tinc*50
    for i = 2:5:length(t)

        % Ball center
        xc = L*sin(theta(i,1));
        yc = L*cos(theta(i,1));
        % Updating Ball and Line 
        updateCirc(ball,xc,yc,6)
        updateLine(line,0,xc,0,yc)
        
        % Updating title text
        newText = sprintf('t = %.2f s', t(i));
        set(hText(1), 'String', newText);  % Update text and shift position to follow the time
        
        % Pausing for the amount of time passed in the simulation
        pause(t(i,1)-t(i-1,1))
    end
end


function draw_cart(t, x_cart)
    % Animating the Cart Dynamics
    % Drawing Cart and tires
    cart = rectangle(x_cart(1), 40, 12);
    tire1 = circle(x_cart(1)-10,-9,3,'k','k');
    tire2 = circle(x_cart(1)+10,-9,3,'k','k');

    % Pause for 1 seconds
    pause(1)

    % Changing title
    hText=findall(gca, 'Type', 'text');
    newText = sprintf('t = %.2f s', 0);
    set(hText(1), 'String', newText);  % Update text and shift position to follow the time

    % Running 200 seconds in steps of tinc*50
    for i = 2:50:length(t)
        % Updating position of cart and tires
        updateRect(cart,x_cart(i,1),40,12)
        updateCirc(tire1,x_cart(i,1)-10,-9,3)
        updateCirc(tire2,x_cart(i,1)+10,-9,3)
        
        % Updating title text
        newText = sprintf('t = %.2f s', t(i));
        set(hText(1), 'String', newText);  % Update text and shift position to follow the time
        
        % Pausing for the amount of time passed in the simulation
        pause(t(i,1)-t(i-1,1))
    end
end


%%
% Support functions to draw and update positions

function rect = rectangle(x, width, height)
    % creating a rectangular shape
    xpos = [x-width/2 x-width/2 x+width/2 x+width/2];
    ypos = [-height/2 height/2 height/2 -height/2];
    rect = patch(xpos, ypos, 'red');
end

function updateRect(rect,x, width, height)
    % updating x and y position
    xpos = [x-width/2 x-width/2 x+width/2 x+width/2];
    % ypos = [-height/2 height/2 height/2 -height/2];
    set(rect,'XData',xpos);
end

function circ = circle(xc,yc,r,fillcolor,edgecolor)
    % creating ball 
    theta = linspace(0, 2*pi, 100);
    x = r*cos(theta) + xc;
    y = r*sin(theta) + yc;
    circ = patch('XData',x,'YData',y,'EdgeColor', edgecolor, 'FaceColor', fillcolor);
end

function updateCirc(circ,xc,yc,r)
    % updating ball position
    theta = linspace(0, 2*pi, 100);
    x = r*cos(theta) + xc;
    y = r*sin(theta) + yc;
    set(circ,"XData",x,"YData",y);
end

function line = drawLine(x1,x2,y1,y2)
    % Draw line
    xpos = [x1 x2];
    ypos = [y1 y2];
    line = patch(xpos,ypos,'k');
end

function updateLine(line,x1,x2,y1,y2)
    % Updating Line position
    set(line,"XData",[x1 x2],"YData",[y1 y2])
end