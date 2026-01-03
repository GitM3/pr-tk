% NOW-HOW
% Linear Controller Inverted Pendulum

clc
clear
close all
%%
%
% Running the whole program
inverted_pendulum_full


function inverted_pendulum_full()
    % Going through the full inverted Pendulum Simulation and Animation
    
    global x
    screenSize = get(0, 'ScreenSize');
    
    % Create a figure to capture key presses
    fig = figure('Name', 'Press ''q'' to close this figure', ...
                 'KeyPressFcn', @keyPress);
    xlabel("X Position [m]", FontSize= 15);
    ylabel("Y Position [m]", FontSize= 15)
    set(fig, 'OuterPosition', screenSize);
    % Title of the figure
    title("Inverted Pendulum Stabilization: \theta=0, x=0",FontSize=20)
    % Makes axis 1 unit equal, so circle doesn't lost its shape
    axis equal

    % axis setting limits
    hAxes = findall(fig,'Type','axes');
    ylim([-50 50]);
    xlim([-500 500]);
    yline(-13,'-k',LineWidth=4)
    % Putting grid on the figure (Square Boxes)
    grid on
    
    % Pause for 1 second
    pause(1)

    % X, V, theta, w
    y0 = [-100, 0, 0.5, 0];
    % Stabilize the system for initial conditions X=-100 Theta = -0.1
    stabilize(y0)

    % Set the ButtonDownFcn for the axes
    % this will make it wait until the mouse is clicked
    set(hAxes, 'ButtonDownFcn', @XpositionClick);

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
    
    % Click mouse to set new X position and angle
    function XpositionClick(src, event)
        % Callback function that executes on a mouse click within the axes
        y0 = [0 0 0 0];
        
        % Get coordinates
        coords = get(src, 'CurrentPoint');  % Gets the x, y coordinates of the click
        x_press = coords(1, 1);
        y0(1) = x_press;
        
        % Getting the patches and setting new position
        patches = findall(gca,'type','patch');
        for i = 1:length(patches)
            patch = patches(i);
            patch.Vertices(:,1) = patch.Vertices(:,1) + x_press;
        end

        % Getting the current figure
        fig = gcf;
        hAxes = findall(gca,'Type','axes');
        hText = findall(gca,'Type','text');

        % Asking to choose an angle because it was already clicked and x
        % position set
        newText = 'Click to choose an angle';
        set(hText(1), 'String', newText);  % Update text and shift position to follow the time
        
        % waiting for the user click to set an angle
        set(hAxes, 'ButtonDownFcn',@(src, event)uiresume(fig)); 
        uiwait(fig)
        
        % This is after we chose an angle
        coords = get(src, 'CurrentPoint');  % Gets the x, y coordinates of the click
        x_press = coords(1, 1)-x_press;
        y = coords(1, 2);
        angle = atan2((x_press-x(end)),y);
        y0(3) = angle;
        
        % Delete all patches to show new animation
        delete(findall(gca, 'Type', 'patch'));
        stabilize(y0)
        
        % making the code wait for another mouseclick
        set(hAxes, 'ButtonDownFcn', @XpositionClick);
   end
end


function stabilize(y0)
    % Stabilize the system with the initial conditions
    % y0 = [x0, v0, theta0, w0]

    % Linearized System
    M=10;       % Cart mass               [kg]
    m=5;        % Ball mass               [kg]
    L=20;       % Pendulum length         [m]
    B_M = 0.1;  % Cart Friction coeff     [kg]
    B_m = 0.1;  % Ball Friction coeff     [kg*m^2]
    g=9.81;     % Gravitational Accel     [m/sec^2]

    alpha = 1;  % Jacobian matrix coefficient (Up = 1) (Down = -1)

    % Linearized State Space Matrices
    A = [ 0 1 0 0;
        0 -B_M/M -g*m/M alpha*B_m/L/M;
        0 0 0 1;
        0 alpha*B_M/L/M alpha*(M+m)*g/L/M -(M+m)*B_m/L^2*M*m];

    B = [0;
        1/(M);
        0;
        -alpha/(L*M)];
    
    % Checking stability and controllability
    eig(A)             % if real parts all negative then stable
    rank(ctrb(A,B))    % needs to equal to 4 to be controllable

    % Creating a weight matrix
    Q = diag([1 1 10 100]);
    % Regularization weight
    R = 10;
    % Controller K
    K = lqr(A,B,Q,R);
    
    % Running the simulation for 200 seconds
    tend = 150;
    tinc = 0.01;
    tspan = 0:tinc:tend;
    
    [t,y] = ode45(@(t,y) inverted_pendulum(y,M,m,L,B_M,B_m,g, K),tspan,y0);

    x = y(:,1);
    v = y(:,2);
    th = y(:,3);
    w = y(:,4);
    draw_inverted_pendulum(t,x,th,L)
end

function dy = inverted_pendulum(y,M,m,L,B_M,B_m,g,K)
    % This is bare system dynamics, how the system will affect
    % to initial conditions
    % and force u
    % LQR controller for stabilization
    reference = [0; 0; 0; 0];

    % Force u applied on the cart
    u = -K * (y - reference);

    % States
    x = y(1,1);
    v = y(2,1);
    th = y(3,1);
    w = y(4,1);

    % State Space Model
    dy(1,1) = v;
    dy(2,1) = (L*u + B_m*w*cos(th) - m*L*g*sin(th)*cos(th) + m*L^2*w^2*sin(th) - B_M*L*v)/( L*( M + m - m*(cos(th))^2 ) );
    dy(3,1) = w;
    dy(4,1) = (-m*L*cos(th)*u - m^2*L^2*w^2*sin(th)*cos(th) + B_M*v*m*L*cos(th) - (M+m)*B_m*w + (M+m)*m*g*L*sin(th))/(m*L^2*(M+m-m*(cos(th))^2));
end

function draw_inverted_pendulum(t, x_cart, theta, L)
    % Animating the Inverted Pendulum
    
    % Ball Center
    xc = x_cart(1,1)+L*sin(theta(1,1));
    yc = L*cos(theta(1,1));
    % Drawing Cart, Ball, and Line
    cart = rectangle(x_cart(1), 40, 12);
    ball = circle(xc,yc,6,'b','k');
    line = drawLine(x_cart(1),xc,0,yc);
    tire1 = circle(x_cart(1)-10,-9,3,'k','k');
    tire2 = circle(x_cart(1)+10,-9,3,'k','k');

    % Pause for 3 seconds
    pause(1)

    % Changing title
    hText=findall(gca, 'Type', 'text');
    newText = sprintf('t = %.2f s', 0);
    set(hText(1), 'String', newText);  % Update text and shift position to follow the time


    % Running 200 seconds in steps of tinc*50
    for i = 2:75:length(t)
        % Updating position of cart
        updateRect(cart,x_cart(i,1),40,12)

        % Ball center
        xc = x_cart(i,1)+L*sin(theta(i,1));
        yc = L*cos(theta(i,1));
        % Updating Ball and Line 
        updateCirc(ball,xc,yc,6)
        updateCirc(tire1,x_cart(i,1)-10,-9,3)
        updateCirc(tire2,x_cart(i,1)+10,-9,3)

        updateLine(line,x_cart(i,1),xc,0,yc)
        
        % Updating title text
        newText = sprintf('t = %.2f s', t(i));
        set(hText(1), 'String', newText);  % Update text and shift position to follow the time
        
        % Pausing for the amount of time passed in the simulation
        pause(t(i,1)-t(i-1,1))
    end
    pause(0.5)
    % setting new title
    set(hText(1), 'String', {'Click!', 'Select the new X-Position.'})
end

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
