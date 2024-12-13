%% Motion Capture system initialization 
% PLEASE INPUT VALUES HERE
RigidBodyIndex = 14 ;  % Make sure this matches OptiTrack Streaming Number
myIp = '192.168.0.221';          % Your IP Address
myClientObject = natnet;
myClientObject.HostIP = '192.168.0.103';            % OptiTrack's IP Address
myClientObject.ClientIP = myIp;    
myClientObject.connect;
if (myClientObject.IsConnected == 0 )
	fprintf( 'Client failed to connect\n' )
	fprintf( '\Make sure the host is connected to the network\n' )
	fprintf( '\tand that the host and client IP addresses are correct\n\n' ) 
	return
end
DataFrame = myClientObject.getFrame;
% Search for the rigid body by name
Index = 1; 
while DataFrame.RigidBodies(Index).ID ~= RigidBodyIndex
    Index = Index+1;
end
RigidBodyNameMemory = Index;
% INIT OVER
% Initializing ROS
rosinit('http://192.168.0.201:11311');  % Must use rosmaster ip
% Create the publisher
topicName = '/mocap_info';
messageType = 'std_msgs/Float32MultiArray';
pub = rospublisher(topicName, messageType);
% Create the message
msg = rosmessage(pub);
% Getting initial values for CARLA teleportation
[last_pos_x, last_pos_y, last_pos_z, last_heading_degrees] = getRealPose(myClientObject, RigidBodyNameMemory); 
for steps = 1:10000
    [current_real_x, current_real_y, current_real_z, current_heading_degrees] = getRealPose(myClientObject, RigidBodyNameMemory); 
    
    % Calculate the raw difference
    heading_diff = current_heading_degrees - last_heading_degrees;
    
    % Normalize the difference to be in the range -180 to 180 degrees
    if heading_diff > 180
        heading_diff = heading_diff- 360;
    elseif heading_diff < -180
        heading_diff = heading_diff + 360;
    end
     % Assign the values to the message
    msg.Data = [current_real_z - last_pos_z, current_real_x - last_pos_x, current_heading_degrees];
    % Updating last positions
    %last_pos_z = current_real_z;
    %last_pos_x = current_real_x;
    last_heading_degrees = current_heading_degrees;
    
    % Publish the message
    send(pub, msg);
    
    % Pause before sending the next message
    pause(0.1);
end
rosshutdown
%END OF NEW CODE
function [x, y, z, real_heading_degrees] = getRealPose(myClientObject, RigidBodyNameMemory)
    DataFrame = myClientObject.getFrame;    % 1 frame per cell
    x = DataFrame.RigidBodies(RigidBodyNameMemory).x;
    y = DataFrame.RigidBodies(RigidBodyNameMemory).y;
    z = DataFrame.RigidBodies(RigidBodyNameMemory).z;
    % Assuming you have quaternion values q_w, q_x, q_y, q_z
    quat_w = DataFrame.RigidBodies(RigidBodyNameMemory).qw;  % Replace with actual data extraction
    quat_x = DataFrame.RigidBodies(RigidBodyNameMemory).qx;
    quat_y = DataFrame.RigidBodies(RigidBodyNameMemory).qy;
    quat_z = DataFrame.RigidBodies(RigidBodyNameMemory).qz;
    Quaternion = [quat_w quat_x quat_y quat_z];
    
    % Define unit vectors
    xUnitVector = [1; 0; 0];  
    yUnitVector = [0; 1; 0];  
    % Convert quaternion to orthonormal rotation matrix
    RotationMat = quat2rotm(Quaternion);
    % Rotate the x-unit vector
    RotatedXVec = RotationMat * xUnitVector;
    % Project onto the XZ-plane
    ProjectedXVec = RotatedXVec;
    ProjectedXVec(2) = 0;  % Eliminate Y-component
    % Normalize the vector in the XZ-plane (Euclidean normalization)
    VecMagnitude = norm(ProjectedXVec);
    ProjectedXVec = ProjectedXVec / VecMagnitude;
    % Calculate the heading angle
    DotProduct = dot(ProjectedXVec, xUnitVector);
    % Handle floating-point precision issues in acosd
    DotProduct = min(max(DotProduct, -1), 1);
    Heading = acosd(DotProduct);    % heading angle in degrees
    % Determine the sign of the heading angle using the Z-component
    if ProjectedXVec(3) > 0
      Heading = 360-Heading;
    end
    real_heading_degrees = Heading;
end