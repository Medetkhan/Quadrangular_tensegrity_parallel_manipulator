bag = rosbag('./test150.bag');
%bagInfo = rosbag('info','./motor_positions_bagfiles/test1.bag');

bJoints = select(bag,'Topic','/dynamixel_workbench_controllers/joint_states');
msgJoints = readMessages(bJoints);
jointsP = zeros(length(msgJoints),4);
jointsV = zeros(length(msgJoints),4);
jointsE = zeros(length(msgJoints),4);
for i= 1:length(msgJoints)
    jointsP(i,:) = msgJoints{i,1}.Position;
    jointsV(i,:) = msgJoints{i,1}.Velocity;
    jointsE(i,:) = msgJoints{i,1}.Effort;
end

%plot(jointsP)


% Assuming you have already loaded the bag and extracted joint state data

% Define the file names
csvFileName = 'test150.csv';

% Create a table for the joint states
jointStatesTable = table(jointsP, 'VariableNames', {'Position'});

% Write the table to a CSV file
writetable(jointStatesTable, csvFileName);

disp(['Data saved as ' csvFileName]);
