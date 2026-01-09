% Clearing all variables and closing existing connections
clear all;

% Station coordinates
pos_A = [100, 0, 110];
pos_B = [0, 100, 0];
pos_C = [-100, 0, 30];

% Initializing EV3 object
myev3 = legoev3('usb');

% 0 is for picking the ball
% 1 is for placing the ball
homing(myev3);
stationB(0, myev3);
stationC(1, myev3);
stationC(0, myev3);
stationA(1, myev3);
stationA(0, myev3);
stationB(1, myev3);
stationB(0, myev3);
stationA(1, myev3);
stationA(0, myev3);
stationC(1, myev3);
stationC(0, myev3);
stationB(1, myev3);

% Homing Function
function [] = homing(myev3) 
    % Initializing variables
    motorC = motor(myev3, 'C');
    touch1 = touchSensor(myev3, 1);
    touch2 = touchSensor(myev3, 3);
    press1 = readTouch(touch1);
    enc2 = readRotation(motorC);

    % Calls for the function
    top(myev3);
    start(motorC);

    % Checks whether the touch sensor in the bottom is pressed
    if(press1 == 0)

        % Given a clockwise rotation with speed 25 until the touch sensor is triggered    
        while(press1 == 0)
            press1 = readTouch(touch1);
            motorC.Speed = 25;
        end

        % Flag variable is used for knowing the direction of rotation,
        % 1 for anticlockwise and -1 for clockwise rotation 
        flag = 1;
        w = 1;

        % Calls the function with some inputs
        downpos(flag, w, myev3)
        while true
            if (press1 == 1)
                motorC.Speed =  0;
                break;
            end
        end
    end
    pause(2);
end

% Operations to be performed in station A
function [] = stationA(k, myev3)
    sonic = sonicSensor(myev3, 2);
    if (k == 0)
        % For picking, First move to the desired downward position, open the
        % gripper, move to the desired height, close the gripper, move to the top
        % and go back to home B position 
        flag = -1;
        w = 0;
        downpos(flag, w, myev3);

        % Measuring the distance
        dist = readDistance(sonic);
        z = (0.2350 - dist) * 1000;
        openA(myev3);
        toppos(z, myev3);
        closeA(myev3);
        top(myev3);

        % Going back home
        flag = 1;
        downpos(flag, w, myev3);
    else
        % For placing the ball, First move to the desired downward position with
        % ball, move to the desired height, open the gripper, move to the top
        % , close the gripper, and go back to home

        flag = -1;
        w = 0;
        downpos(flag, w, myev3);

        % Measuring the distance
        dist = readDistance(sonic);
        z = (0.2350 - dist) * 1000;
        toppos(z, myev3);
        openA(myev3);
        top(myev3);
        closeA(myev3);

        % Going back home
        flag = 1;
        downpos(flag, w, myev3);
        pause(0.5);
    end
end

% Operations to be performed in station C
function [] = stationC(k, myev3)
    % Variable for ultrasonic sensor
    sonic = sonicSensor(myev3, 2);
    if (k == 0)
        % For picking, First move to the desired downward position, open the
        % gripper, move to the desired height, close the gripper, move to the top
        % and go back to home B position
        flag = 1;
        w = 0;
        downpos(flag, w, myev3);

        % Measuring the distance
        dist = readDistance(sonic);
        z = (0.2350 - dist) * 1000;
        openA(myev3);
        toppos(z, myev3);
        closeA(myev3);
        top(myev3);

        % Going back home
        flag = -1;
        downpos(flag, w, myev3);
    else
        % For placing the ball, First move to the desired downward position with
        % ball, move to the desired height, open the gripper, move to the top
        % , close the gripper, and go back to home
        flag = 1;
        w = 0;
        downpos(flag, w, myev3);
        
        % Measuring the distance
        dist = readDistance(sonic);
        z = (0.2350 - dist) * 1000;
        toppos(z, myev3);
        openA(myev3);
        top(myev3);
        closeA(myev3);

        % Going back home
        flag = -1;
        downpos(flag, w, myev3);
        pause(0.5);
    end
end


% Operations to be performed in station B
function [] = stationB(k, myev3)
    % Variable for ultrasonic sensor
    sonic = sonicSensor(myev3, 2);
    
    if (k == 0)
        openA(myev3);
        dist = readDistance(sonic);
        z = (0.2350 - dist) * 1000;
        toppos(z, myev3);
        closeA(myev3);
        top(myev3);
    else
        dist = readDistance(sonic);
        z = (0.2350 - dist) * 1000;
        toppos(z, myev3);
        openA(myev3);
        top(myev3);
        closeA(myev3);
        pause(0.5);
    end
end

% Controlling the down position
function [] = downpos(f, w, myev3)
    motorC = motor(myev3, 'C');
    enc2 = readRotation(motorC); 
    resetRotation(motorC);

    % w = 1 is used only to bring it to home
    if (w == 1)
        theta_1 = 115;
    elseif (w == 0)
        theta_1 = 90;
    end
    
    % Calculating the angle for the height using inverse kinematics geometric approach
    enc2 = readRotation(motorC);

    % Error to be minimized and here the gear ratio of 3
    error_1 = (theta_1) + (enc2 * f / 3);
    start(motorC);

    % Initial total error and old error value are assigned as 0
    ek0 = 0;
    t_error = 0;
    
    % Runs the motor B unless the error gets minimized to 1
    while (error_1 > 1)
        % Calling the controller function by providing error as input
        motorC.Speed = -controller(error_1, myev3, ek0, t_error) * f;        
        error_1 = (theta_1) + (enc2 * f / 3);
        enc2 = readRotation(motorC);
    end
    
    t_error = 0;
    motorC.Speed = 0;
    resetRotation(motorC);
end

% To adjust the height of the gripper
function [] = toppos(z, myev3)
    motorB = motor(myev3, 'B');
    sonic = sonicSensor(myev3, 2);
    enc1 = readRotation(motorB); 

    % Lengths of the links
    l1 = 50;
    l2 = 95;
    l3 = 185;
    l4 = 110;
    l0 = 70;

    resetRotation(motorB);
    enc1 = readRotation(motorB);

    % Calculating the angle for the height using inverse kinematics geometric approach
    theta_2 = -59.8838 + acosd((z - 25 + l4 - l0 - l1 - (l2 * sind(45))) / l3);

    % Error to be minimized and here the gear ratio of 5
    error_2 = double(theta_2) - (double(enc1) / 5);

    % Initial total error and old error value are assigned as 0
    ek0 = 0;
    t_error = 0;

    % Runs the motor B unless the error gets minimized to 2
    while (error_2 > 2)
        % Calling the controller function by providing error as input
        motorB.Speed = controller(error_2, myev3, ek0, t_error);        
        error_2 = double(theta_2) - (double(enc1) / 5);
        enc1 = readRotation(motorB);
    end
    
    t_error = 0;
    motorB.Speed = 0;
    resetRotation(motorB)
end

% Function to move the arm to the top until the touch sensor gets pressed
function [] = top(myev3)
    motorB = motor(myev3, 'B');
    touch2 = touchSensor(myev3, 3);
    press2 = readTouch(touch2);
    start(motorB);
    
    while (press2 == 0)
        press2 = readTouch(touch2);
        motorB.Speed = -25;
    end
    
    if (press2 == 1)
        motorB.Speed = 0;
    end
end

% To open the gripper irrespective of the direction of the rotation
function [] = openA(myev3)
    motorA = motor(myev3,'A');
    enc3 = readRotation(motorA);
    resetRotation(motorA);
    start(motorA);

    % the motor A rotates 170 degree in clockwise and -170 in anticlockwise
    % the 170 degree includes both opening and closing. so for only opening
    % the encoder value is maintained till 85
    while (abs(enc3) < 80)
        encvalue1 = zeros(1, 3);

        for i = 1:3
            motorA.Speed = 5;
            readRotation(motorA);
            enc3 = readRotation(motorA);
            encvalue1(i) = enc3;
        end

        % If the motor A doesn't rotate for positive speed the encoder reads
        % the same value so now the direction of rotation if changed by assign
        % the value of k as 0 and calls the function again - recursive function
        if(encvalue1(1) == encvalue1(2) && encvalue1(2) == encvalue1(3))
            while (abs(enc3)<80)
                motorA.Speed = -5;
                readRotation(motorA);
                enc3 = readRotation(motorA);
            end
            break;
        end
    end

    motorA.Speed = 0;
end

% To close the gripper by giving inputs to motor A
function [] = closeA(myev3)
    motorA = motor(myev3,'A');
    enc3 = readRotation(motorA);
    start(motorA);

    % Checks whether the gripper is open
    % Runs the motor A until the gripper closes and the encoder values
    % become 0
    while (abs(enc3) >= 0)
        encvalue1 = zeros(1, 3);

        for i = 1:3
            motorA.Speed = -5;
            readRotation(motorA);
            enc3 = readRotation(motorA);
            encvalue1(i) = enc3;
        end

        % If the gripper is closed or if the object (ball) constrains the
        % rotation the encoder value remains the same
        % if the values in the matrix are the same, the motor A is stopped
        % and comes out of the loo
        if(encvalue1(1) == encvalue1(2) && encvalue1(2) == encvalue1(3));
            motorA.Speed = 0;
            break;
        end
    end

    motorA.Speed=0;
    resetRotation(motorA);
end

% Function for PID controller
function d = controller(ek, myev3, ek0, t_error)
    motorA = motor(myev3,'A');

    %proportionality constant
    kp = 0.25;

    %intergal constant
    ki = 35;

    %derivative constant
    kd = 0.0001;

    i=0;

    % while loop is used to calculate the sampling time using tic toc
    while (i==0)
        tic;
        readRotation(motorA);
        toc;
        t= toc;
        i=1;
    end

    % Calculating total error
    t_error = t_error + ek;

    % calculating derivative de using finite difference
    de = (ek - ek0);

    %PID controller in digital form
    d = (kp*ek) + (ki*t*t_error) + ((kd/t)*de);

    % In order to maintain the speed If it goes below 20 
    if (d<20)
        d=20;
    end

    %shifting the current error value to old value for derivative term
    ek0 = ek;
end
