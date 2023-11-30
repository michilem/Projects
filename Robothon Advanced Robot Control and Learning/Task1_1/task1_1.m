%% Reference is the Solution from Group D
clear all
close all
clc

sim=remApi('remoteApi'); 
sim.simxFinish(-1); 
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
        disp('Connected to remote API server');
        
        % retrive joint handles
        h = [0,0,0,0,0,0,0]; % 7 joints
        [r, h(1)] = sim.simxGetObjectHandle(clientID,'Franka_joint1',sim.simx_opmode_blocking);
        [r, h(2)] = sim.simxGetObjectHandle(clientID,'Franka_joint2',sim.simx_opmode_blocking);
        [r, h(3)] = sim.simxGetObjectHandle(clientID,'Franka_joint3',sim.simx_opmode_blocking);
        [r, h(4)] = sim.simxGetObjectHandle(clientID,'Franka_joint4',sim.simx_opmode_blocking);
        [r, h(5)] = sim.simxGetObjectHandle(clientID,'Franka_joint5',sim.simx_opmode_blocking);
        [r, h(6)] = sim.simxGetObjectHandle(clientID,'Franka_joint6',sim.simx_opmode_blocking);
        [r, h(7)] = sim.simxGetObjectHandle(clientID,'Franka_joint7',sim.simx_opmode_blocking);
        
        % Import joint pose data to check
        file = 'self_collision_specific_joint_poses.data';
        joint_data = importdata(file);
        n = size(joint_data,1);
        joint_poses = zeros(n,7);

        for i = 1 : n
            pose = extractBetween(joint_data(i),"[","]");
            joints = convertCharsToStrings(pose);
            joint_poses(i,:) = str2num(joints);
        end
        
        % Create results matrix for the csv-file output and for the
        % txt-file output
        results = strings(n,1);
        results_csv = strings(n+1,1);
        results_csv(1) = "Collision ; Groups"; % Header for the csv table

        for i=1:n
            curr_joint_pose = joint_poses(i,:);
            for j=1:7
                sim.simxSetJointPosition(clientID,h(j),curr_joint_pose(j),sim.simx_opmode_oneshot);
            end
            
            r = sim.simxSynchronous(clientID,0);
            [r, state_self] = sim.simxGetIntegerSignal(clientID,'co_self',sim.simx_opmode_blocking);
            if state_self
                [r, collisionP1] = sim.simxGetStringSignal(clientID,'cP1',sim.simx_opmode_blocking);
                [r, collisionP2] = sim.simxGetStringSignal(clientID,'cP2',sim.simx_opmode_blocking);
                % Create output in the required format for the txt-file
                s1 = '{ \n \t collision: true, \n';
                s2 = '\t collision groups : {' + string(i) + ' : ' + string(collisionP1) +  ' ' + string(collisionP2) + '} \n}';
                results(i) = s1 + s2;
                results(i) = compose(results(i));

                s1_csv = "true";
                s2_csv = string(i) + " : " + string(collisionP1) +  " " + string(collisionP2);
                results_csv(i+1) = s1_csv + ';' + s2_csv
            else
               results(i) = '{ \n \t collision: false, \n \t collision groups : { } \n}';
               results(i) = compose(results(i));

               results_csv(i+1) = "false"
            end

            pause(0.5)
        end
        writematrix(results,'self_collision_open_gripper_results.txt');
        writematrix(results_csv,'self_collision_open_gripper_results.csv');
else
        disp('Failed connecting to remote API server');
end
sim.delete();

disp('Program ended');
    

