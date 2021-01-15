clc;
clear all;

fprintf('Dynamic Path tracking of a 6-wheel steering mobile robot on a curved trajectory\n')
prompt = 'Choose the controller you would like to visualize. Press 1 for Waypoint controller and 2 for Stanley controller';
flag = input(prompt);
prompt = 'Press 1 if you want to give a constant velocity of 5m/s to the robot or Press 2 if you want the robot to move at a variable speed';
v = input(prompt);
z = [34.692383, 136.41483, 0, 0, 0, 0];
if flag==1
    if v==1
        w = load('ref_curve.txt');
        w(:,3) = 5;
        
        [log_error, log_z, log_torque, log_Md] = controller_WP(w,z);
        
        fprintf('The time taken to complete the trajectory in s is: \n ')
        display(log_error(size(log_error,1),1))
        fprintf('The maximum lateral deviation produced in m is: \n ')
        display(max(abs(log_error(:,8))))
        fprintf('The average lateral deviation produced in m is: \n ')
        display(mean(abs(log_error(:,8))))
        
        figure(1)   
        plot(w(:,1),w(:,2),'--b','LineWidth',2)
        hold on
        plot(log_z(:,2),log_z(:,3),'g','LineWidth',1)
        title('Curved trajectory')
        xlabel('X in m')
        ylabel('Y in m')
        legend('Desired','Simulated')
        grid on
        
        figure(2) 
        plot(log_error(:,1),log_error(:,5).*180/pi,'--b',log_error(:,1),log_error(:,6).*180/pi,'r')
        title('Heading Angle')
        xlabel('Time in sec')
        ylabel('Angle')
        legend('Desired','Simulated')
        grid on
        
        figure(3)   
        plot(log_error(:,1),log_error(:,8))
        title('Error Lateral')
        xlabel('Time in sec')
        ylabel('Lateral position error in m')
        grid on
        
        figure(4)
        subplot(2,1,1)
        plot(log_torque(:,1),log_torque(:,2),'g')
        hold on
        plot(log_torque(:,1),log_torque(:,3))
        title('Input torque to the front tire')
        xlabel('Time in sec')
        ylabel('Torque in Nm')
        legend('Front left tire','Front right tire')
        grid on
        subplot(2,1,2)
        plot(log_Md(:,1),log_Md(:,3),'b')
        title('Estimated Disturbance Yaw Moment Md')
        xlabel('Time in sec')
        ylabel('Disturbance moment in Nm')
        grid on
        

    elseif v==2
        
        w = load('ref_curve.txt');
        [log_error, log_z, log_torque, log_Md] = controller_WP(w,z);
        
        fprintf('The time taken to complete the trajectory in s is: \n ')
        display(log_error(size(log_error, 1),1))
        fprintf('The maximum lateral deviation produced in m is: \n ')
        display(max(abs(log_error(:,8))))
        fprintf('The average lateral deviation produced in m is: \n ')
        display(mean(abs(log_error(:,8))))
        
        figure(1)   
        plot(w(:,1),w(:,2),'--b','LineWidth',2)
        hold on
        plot(log_z(:,2),log_z(:,3),'g','LineWidth',1)
        title('Curved trajectory')
        xlabel('X in m')
        ylabel('Y in m')
        legend('Desired','Simulated')
        grid on
        
        figure(2)   
        subplot(2,1,1)
        plot(log_error(:,1),log_error(:,2),'--b',log_error(:,1),log_error(:,3),'r',log_error(:,1),log_error(:,4),'k')
        title('Longitudinal Velocity')
        xlabel('Time in sec')
        ylabel('Velocity')
        legend('Desired','Simulated')
        grid on
        subplot(2,1,2)
        plot(log_error(:,1),log_error(:,5).*180/pi,'--b',log_error(:,1),log_error(:,6).*180/pi,'r')
        title('Heading Angle')
        xlabel('Time in sec')
        ylabel('Angle')
        legend('Desired','Simulated')
        grid on
        
        figure(3)   
        plot(log_error(:,1),log_error(:,8))
        title('Error Lateral')
        xlabel('Time in sec')
        ylabel('Lateral position error in m')
        grid on
        
        figure(4) 
        subplot(2,1,1)
        plot(log_torque(:,1),log_torque(:,2),'g')
        hold on
        plot(log_torque(:,1),log_torque(:,3))
        title('Input torque to the front tire')
        xlabel('Time in sec')
        ylabel('Torque in Nm')
        legend('Front left tire','Front right tire')
        grid on
        subplot(2,1,2)
        plot(log_Md(:,1),log_Md(:,3),'b')
        title('Estimated Disturbance Yaw Moment Md')
        xlabel('Time in sec')
        ylabel('Disturbance moment in Nm')
        grid on
    else
        fprintf('Try again with a different choice :(')
    end
    
elseif flag==2
    if v==1
        
        w = load('ref_curve.txt');
        w(:,3) = 5;
        [log_error, log_z, log_torque, log_Md] = controller_stanley(w,z);
        
        fprintf('The time taken to complete the trajectory in s is: \n ')
        display(log_error(size(log_error,1),1))
        fprintf('The maximum lateral deviation produced in m is: \n ')
        display(max(abs(log_error(:,8))))
        fprintf('The average lateral deviation produced in m is: \n ')
        display(mean(abs(log_error(:,8))))
        
        figure(1)   
        plot(w(:,1),w(:,2),'--b','LineWidth',2)
        hold on
        plot(log_z(:,2),log_z(:,3),'g','LineWidth',1)
        title('Curved trajectory')
        xlabel('X in m')
        ylabel('Y in m')
        legend('Desired','Simulated')
        grid on
        
        figure(2) 
        plot(log_error(:,1),log_error(:,5).*180/pi,'--b',log_error(:,1),log_error(:,6).*180/pi,'r')
        title('Heading Angle')
        xlabel('Time in sec')
        ylabel('Angle')
        legend('Desired','Simulated')
        grid on
        
        figure(3)   
        plot(log_error(:,1),log_error(:,8))
        title('Error Lateral')
        xlabel('Time in sec')
        ylabel('Lateral position error in m')
        grid on
        
        figure(4)
        subplot(2,1,1)
        plot(log_torque(:,1),log_torque(:,2),'g')
        hold on
        plot(log_torque(:,1),log_torque(:,3))
        title('Input torque to the front tire')
        xlabel('Time in sec')
        ylabel('Torque in Nm')
        legend('Front left tire','Front right tire')
        grid on
        subplot(2,1,2)
        plot(log_Md(:,1),log_Md(:,3),'b')
        title('Estimated Disturbance Yaw Moment Md')
        xlabel('Time in sec')
        ylabel('Disturbance moment in Nm')
        grid on
        
    elseif v==2
        
        w = load('ref_curve.txt');
        [log_error, log_z, log_torque, log_Md] = controller_stanley(w,z);
        
        fprintf('The time taken to complete the trajectory in s is: \n ')
        display(log_error(size(log_error, 1),1))
        fprintf('The maximum lateral deviation produced in m is: \n ')
        display(max(abs(log_error(:,8))))
        fprintf('The average lateral deviation produced in m is: \n ')
        display(mean(abs(log_error(:,8))))
        
        figure(1)   
        plot(w(:,1),w(:,2),'--b','LineWidth',2)
        hold on
        plot(log_z(:,2),log_z(:,3),'g','LineWidth',1)
        title('Curved trajectory')
        xlabel('X in m')
        ylabel('Y in m')
        legend('Desired','Simulated')
        grid on
        
        figure(2)  
        subplot(2,1,1)
        plot(log_error(:,1),log_error(:,2),'--b',log_error(:,1),log_error(:,3),'r',log_error(:,1),log_error(:,4),'k')
        title('Longitudinal Velocity')
        xlabel('Time in sec')
        ylabel('Velocity')
        legend('Desired','Simulated')
        grid on
        subplot(2,1,2)
        plot(log_error(:,1),log_error(:,5).*180/pi,'--b',log_error(:,1),log_error(:,6).*180/pi,'r')
        title('Heading Angle')
        xlabel('Time in sec')
        ylabel('Angle')
        legend('Desired','Simulated')
        grid on
        
        figure(3)   
        plot(log_error(:,1),log_error(:,8))
        title('Error Lateral')
        xlabel('Time in sec')
        ylabel('Lateral position error in m')
        grid on
        
        figure(4) 
        subplot(2,1,1)
        plot(log_torque(:,1),log_torque(:,2),'g')
        hold on
        plot(log_torque(:,1),log_torque(:,3))
        title('Input torque to the front tire')
        xlabel('Time in sec')
        ylabel('Torque in Nm')
        legend('Front left tire','Front right tire')
        grid on
        subplot(2,1,2)
        plot(log_Md(:,1),log_Md(:,3),'b')
        title('Estimated Disturbance Yaw Moment Md')
        xlabel('Time in sec')
        ylabel('Disturbance moment in Nm')
        grid on
    else
        fprintf('Try again with a different choice :(')
    end
else
    fprintf('Try again with a different choice :(')
         
end





