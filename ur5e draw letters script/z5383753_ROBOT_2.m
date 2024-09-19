% Author: Julia Joharis
% For MTRN4230 2023

%% ----- Robot 2 -----
%% PART A: Trace Digits
clear all;
startup_rvc;
load hershey;

% function to create characters' path
function traj = createPath(character, scale)
    % create the path
    path = [scale*character.stroke; zeros(1,numcols(character.stroke))]; 
    
    % Where ever there is an nan it indicates that we need to lift up.
    k = find(isnan(path(1,:)));
    
    % At these positions add in a z height
    path(:,k) = path(:,k-1); path(3,k) = 0.2*scale; % Determine the hight of the lift up motions. 0.2 * scale is the height. 0.2 is in m
    
    traj = [path'*1000]; % convert to the mm units so that we can use the rtde toolbox
end

% Prompt user for a desired part
partInput = input('Enter Part A | B | C: ','s');

if partInput == 'A'
    char = input('Enter characters to be written: ','s');
    disp('Writing in progress. . .');
    newTraj = [];
    spacingScale = 30;
    currentOffset = 0;
    liftHeight = 0.5 * 0.1 * 1000; % Lift height in mm
    
    for i = 1:length(char)
        trajectory = createPath(hershey{char(i)}, 0.04);
        
        if i > 1
            % Add lifting up segment before moving to the next character
            liftUp = [currentOffset, 0, liftHeight; currentOffset + spacingScale, 0, liftHeight];
            newTraj = [newTraj; liftUp];
        end
        
        % Apply offset for spacing between characters
        trajectory(:, 1) = trajectory(:, 1) + currentOffset;
        
        % Concatenate the new character trajectory to the total trajectory
        newTraj = [newTraj; trajectory];
        
        % Update the current offset by adding the constant spacing value
        currentOffset = currentOffset + spacingScale;
    end
    
    scatter3(newTraj(:,1), newTraj(:,2), newTraj(:,3));
    plot3(newTraj(:,1), newTraj(:,2), newTraj(:,3));

    disp('Program complete!');

elseif partInput == 'B'
    userInput = input('Enter x(mm),y(mm),z(degrees) separated by a space: ', 's');
    parts = strsplit(userInput); % Splits into parts
    
    % Extract the operands and operator
    xTrans = str2double(parts{1});
    yTrans = str2double(parts{2});
    zRot = str2double(parts{3});

    % Place coordinates into homogenous transformation matrices
    rotM_Z = [ cos(zRot), -sin(zRot), 0, 0;
               sin(zRot),  cos(zRot), 0, 0;
                       0,          0, 1, 0;
                       0,          0, 0, 1;
             ];
    transM_XY = [ 1, 0, 0, xTrans;
                  0, 1, 0, yTrans;
                  0, 0, 1,      0;
                  0, 0, 0,      1;
                ];

    % Calculate transformation matrix
    transMatrix = rotM_Z * transM_XY;

    char = input('Enter characters to be written: ','s');
    disp('Writing with transformation matrix in progress. . .');
    newTraj = [];
    spacingScale = 30;
    currentOffset = 0;
    liftHeight = 0.5 * 0.1 * 1000; % Lift height in mm
    
    for i = 1:length(char)
        trajectory = createPath(hershey{char(i)}, 0.04);
        
        if i > 1
            % Add lifting up segment before moving to the next character
            liftUp = [currentOffset, 0, liftHeight; currentOffset + spacingScale, 0, liftHeight];
            newTraj = [newTraj; liftUp];
        end
        
        % Apply offset for spacing between characters
        trajectory(:, 1) = trajectory(:, 1) + currentOffset;
        
        % Concatenate the new character trajectory to the total trajectory
        newTraj = [newTraj; trajectory];
        
        % Update the current offset by adding the constant spacing value
        currentOffset = currentOffset + spacingScale;
    end

    % Convert newTraj into a homogenous matrix
    [numRows, numCols] = size(newTraj);
    homogTraj = [newTraj'; ones(1, numRows)];

    % Multiply transformation into trajectory and organize into a 3-col matrix
    transTraj = transMatrix * homogTraj;
    transTraj = transTraj(1:3, :)';

    newTraj = transTraj; % replaces newTraj

    scatter3(newTraj(:,1), newTraj(:,2), newTraj(:,3));
    plot3(newTraj(:,1), newTraj(:,2), newTraj(:,3));

    disp('Program complete!');

elseif partInput == 'C'
    % Assuming valid inputs
    userInput = input('Enter the expression in the form\n"[integer]<SPACE>[operation]<SPACE>[integer]": ', 's');
    parts = strsplit(userInput); % Splits into parts
    
    % Extract the operands and operator
    num1 = str2double(parts{1});
    operation = parts{2};
    num2 = str2double(parts{3});

    % Perform the operation
    switch operation
        case '+'
            result = num1 + num2;
        case '-'
            result = num1 - num2;
        case '*'
            result = num1 * num2;
            parts{2} = 'x';
        case '/'
            if num2 == 0
                error('Division by zero is not allowed.');
            end
            result = num1 / num2;
        otherwise
            error('Invalid operation. Only +, -, *, / are allowed.');
    end
    
    answer = num2str(result); % Convert to string
    parts{end+1} = answer;
    % Variables to be written: parts{1}, parts{2}, parts{3}, parts{4}
    % Execute paths in long division
    disp('Writing in progress. . .');    
    newTraj = [];
    spacingScale = 30;
    liftHeight = 0.5 * 0.1 * 1000;
    alignToRight = 0;
    currentXOffset = 0;
    currentYOffset = 300;
   
    % parts{1}
    for i = 1:length(parts{1})
        currPart = parts{1}(i);
        trajectory = createPath(hershey{currPart}, 0.04);
        if i > 1    
            % Add lifting up segment before moving to the next character
            liftUp = [currentXOffset, currentYOffset, liftHeight; currentXOffset + spacingScale, currentYOffset + spacingScale, liftHeight];
            newTraj = [newTraj; liftUp];
        end
        
        % Apply x-offset for spacing between characters
        trajectory(:, 1) = trajectory(:, 1) + currentXOffset;

        % Shift the character up by a certain y-height
        trajectory(:, 2) = trajectory(:, 2) + currentYOffset;
        
        % Concatenate the new character trajectory to the total trajectory
        newTraj = [newTraj; trajectory];
        
        % Update the current x-offset by adding the constant spacing value
        alignToRight = currentXOffset; % Store second last x-position
        currentXOffset = currentXOffset + spacingScale;
    end

    % Update the current y-offset by finding the lowest y-value and minus with spacing
    currentYOffset = min(newTraj(:, 2)) - (spacingScale*2);

    liftUp = [currentXOffset, currentYOffset, liftHeight; currentXOffset + spacingScale, currentYOffset + spacingScale, liftHeight];
    newTraj = [newTraj; liftUp];

    % Parts{2} and {3}
    tempTraj = []; % for shifting purposes
    
    for i = 1:length(parts{3})
        currPart = parts{3}(i);
        trajectory = createPath(hershey{currPart}, 0.04);
        if i > 1    
            % Add lifting up segment before moving to the next character
            liftUp = [currentXOffset, currentYOffset, liftHeight; currentXOffset + spacingScale, currentYOffset + spacingScale, liftHeight];
            tempTraj = [tempTraj; liftUp];
        end
        
        % Apply x-offset for spacing between characters
        %if (length(parts{1}) > 1 && length(parts{3}) > 1)
            trajectory(:, 1) = trajectory(:, 1) + (currentXOffset);
        %end

        % Shift the character up by a certain y-height
        trajectory(:, 2) = trajectory(:, 2) + currentYOffset;
        
        % Shifts characters to the left
        %trajectory(:, 1) = trajectory(:, 1) - currentXOffset;
        
        % Concatenate the new character trajectory to the total trajectory
        tempTraj = [tempTraj; trajectory];
        
        % Update the current x-offset by adding the constant spacing value
        alignToRight = currentXOffset; % Store second last x-position
        currentXOffset = currentXOffset + spacingScale;
    end
    
    liftUp = [currentXOffset, currentYOffset, liftHeight; currentXOffset + spacingScale, currentYOffset + spacingScale, liftHeight];
    tempTraj = [tempTraj; liftUp];
    
    for i = 1:length(parts{2})
        currPart = parts{2}(i);
        trajectory = createPath(hershey{currPart}, 0.04);
        if i > 1    
            % Add lifting up segment before moving to the next character
            liftUp = [currentXOffset, currentYOffset, liftHeight; currentXOffset + spacingScale, currentYOffset + spacingScale, liftHeight];
            tempTraj = [tempTraj; liftUp];
        end
        
        % Apply x-offset for spacing between characters
        trajectory(:, 1) = trajectory(:, 1) + currentXOffset;

        % Shift the character up by a certain y-height
        trajectory(:, 2) = trajectory(:, 2) + currentYOffset;
        
        % Concatenate the new character trajectory to the total trajectory
        tempTraj = [tempTraj; trajectory];
        
        % Update the current x-offset by adding the constant spacing value
        alignToRight = currentXOffset; % Store second last x-position
        currentXOffset = currentXOffset + spacingScale;
    end

    liftUp = [currentXOffset, currentYOffset, liftHeight; currentXOffset + spacingScale, currentYOffset + spacingScale, liftHeight];
    tempTraj = [tempTraj; liftUp];
    
    % Shift to the left for alignment and push back into newTraj
    % adjust for lifting up
    if length(parts{3}) == 5
        tempTraj(:,1) = tempTraj(:,1) - 300/2;
    elseif length(parts{3}) == 4
        tempTraj(:,1) = tempTraj(:,1) - 245/2;
    elseif length(parts{3}) == 3
        tempTraj(:,1) = tempTraj(:,1) - 190/2;
    elseif length(parts{3}) == 2
        tempTraj(:,1) = tempTraj(:,1) - 125/2;
    elseif length(parts{3}) == 1
        tempTraj(:,1) = tempTraj(:,1) - 60/2;
    end
    newTraj = [newTraj; tempTraj];

    % Parts{4}
    currentYOffset = min(newTraj(:, 2)) - (spacingScale*2);
    currentXOffset = 0;
    for i = 1:length(parts{4})
        currPart = parts{4}(i);
        trajectory = createPath(hershey{currPart}, 0.04);
        if i > 1    
            % Add lifting up segment before moving to the next character
            liftUp = [currentXOffset, currentYOffset, liftHeight; currentXOffset + spacingScale, currentYOffset + spacingScale, liftHeight];
            newTraj = [newTraj; liftUp];
        end
        
        % Apply x-offset for spacing between characters
        trajectory(:, 1) = trajectory(:, 1) + currentXOffset;

        % Shift the character up by a certain y-height
        trajectory(:, 2) = trajectory(:, 2) + currentYOffset;
        
        % Concatenate the new character trajectory to the total trajectory
        newTraj = [newTraj; trajectory];
        
        % Update the current x-offset by adding the constant spacing value
        currentXOffset = currentXOffset + spacingScale;
    end

    % Plot the trajectory to visualize the long division style
    scatter3(newTraj(:,1), newTraj(:,2), newTraj(:,3));
    plot3(newTraj(:,1), newTraj(:,2), newTraj(:,3));

    disp('Writing complete!');
end

%% NOW USE THE RTDE TOOLBOX TO EXECUTE THIS PATH!


% % TCP Host and Port settings
% host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.230.128'; % THIS IP ADDRESS MUST BE USED FOR THE VMWARE
 host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
% 

% Calling the constructor of rtde to setup tcp connction
rtde = rtde(host,port);

% Setting home
home = [-588.53, -133.30, 371.91, 2.2214, -2.2214, 0.00];


poses = rtde.movej(home);
% Creating a path array
path = [];

% setting move parameters
v = 0.5;
a = 1.2;
blend = 0.001;

% TCP starting point -- to be changed during assessment
tcpStart = [-565.77, -469.74, 28];

% Populate the path array
for i = 1:length(newTraj)
    disp(i);
    disp(newTraj(i,1:3) + [-588.53, -133.30 100]);
    point = [[(newTraj(i,1:3) + tcpStart),(home(4:6))],a,v,0,blend];
    if isempty(path)
        path = point;
    else
        path = cat(1,path,point);
    end
end

% Execute the movement!
poses = rtde.movej(path);

rtde.drawPath(poses);

rtde.close;