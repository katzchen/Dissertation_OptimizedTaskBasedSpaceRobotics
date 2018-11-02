function PlotArms_structure(ArmVariables,TaskVariables,Kinematics,Dynamics,bot)

AllMovie = 0;
for iRobots = 1:length(ArmVariables)
    AllMovie = AllMovie + ArmVariables(iRobots).plotmovie;
end

if (AllMovie  == length(ArmVariables))
    'Plot All Arms'
    
    figure(15)
    axis([-10 10 -10 10 -10 10])
    
    q1 =  Kinematics(1).Arm.Angle;
    q2 =  Kinematics(2).Arm.Angle;
    b1 = SerialLink(bot(1),'name','arm1plot');
    b2 = SerialLink(bot(2),'name','arm2plot');
    
    b1.plot(q1(1,:),'jointdiam', .5,'linkcolor', ArmVariables(1).linkcolor,'noarrow','nobase','noshadow','noshading','notiles','noloop','trail','g o')
    hold all
 %   b2.plot(q2(1,:),'jointdiam', .5,'linkcolor', ArmVariables(2).linkcolor,'nobase','noshadow','noshading','notiles','noloop','trail','r s')

    for i = 1:length(q1)
        
       plotpayload_prism(Kinematics(1).Payload.T_xyz(i,:),Kinematics(1).Payload.T_rpy(i,:), TaskVariables.Lpayload,0); %0 = PLOT
        b1.plot(q1(i,:),'jointdiam', .5,'linkcolor', ArmVariables(1).linkcolor,'nobase','noshadow','noshading','notiles','noloop','trail','g o','noarrow','xyz')
    %    b2.plot(q2(i,:),'jointdiam', .5,'linkcolor', ArmVariables(2).linkcolor,'nobase','noshadow','noshading','notiles','noloop','trail','r s','noarrow','xyz')
    end
    
%     for iRobots = 1:length(ArmVariables)        
%         qi =  Kinematics(iRobots).Arm.Angle;
%         % MovieName = ['RobotArmMoving_' ArmVariables(iRobots).name '.mp4']
%         bot(iRobots).plot((qi),'jointdiam', .5,'linkcolor', ArmVariables(iRobots).linkcolor,'nobase','noshadow','noshading','notiles','top','noloop','trail','--','name', ArmVariables(iRobots).name)
%     end
end

    
for iRobots = 1:length(ArmVariables)
    maxerr_pos = max(Kinematics(iRobots).Arm.error);
    
    if TaskVariables.plot == 1
        figure
        njoints = length(ArmVariables(1).ArmLength);

        subplot(4,1,1)
        hold all
        for i = 1:njoints
            plot(Kinematics(iRobots).Arm.time_all,Kinematics(iRobots).Arm.Acc(:,i))
        end
        
        xlabel('time (s)')
        ylabel('Acceleration m/s')
        title(['Arm Trajectory for ' ArmVariables(iRobots).name])
        
        subplot(4,1,2)
        hold all
        for i = 1:njoints
            plot(Kinematics(iRobots).Arm.time_all, Kinematics(iRobots).Arm.Velocity(:,i))
        end
        xlabel('time (s)')
        ylabel('Velocity m/s')
        
        subplot(4,1,3)
        hold all
        for i = 1:njoints
            plot(Kinematics(iRobots).Arm.time_all, Kinematics(iRobots).Arm.Angle(:,i)*180/pi)
        end
        xlabel('time (s)')
        ylabel('Angle (degree)')
        
        subplot(4,1,4)
        hold all
        for i = 1:njoints
            plot(Kinematics(iRobots).Arm.time_all, Kinematics(iRobots).Arm.error)
        end
        xlabel('time (s)')
        ylabel('Position Error')
        
        
        if maxerr_pos < ArmVariables(iRobots).TotalArmLength/10
            figure
            hold all
            for i = 1:njoints
                plot(Kinematics(iRobots).Arm.time_all, Dynamics(iRobots).Arm.Torque)
            end
            xlabel('time (s)')
            ylabel('Torque (Nm)')
            title(['Joint Torques for ' ArmVariables(iRobots).name])
            
        end
    end
    
    if ArmVariables(iRobots).plotmovie == 1% && maxerr_pos < ArmVariables(iRobots).TotalArmLength/10
        figure
        qi =  Kinematics(iRobots).Arm.Angle;
        MovieName = ['RobotArmMoving_' ArmVariables(iRobots).name '.mp4']
        bot(iRobots).plot((qi),'jointdiam', .5,'linkcolor', ArmVariables(iRobots).linkcolor,'nobase','noshadow','noshading','notiles','noloop','movie',MovieName,'trail','--')
        
    end
    
    if ArmVariables(iRobots).plot == 1 && maxerr_pos < ArmVariables(iRobots).TotalArmLength/10
        
        qi =  Kinematics(iRobots).Arm.Angle;
        %If I have a more complex patter, It makes more sense to do the
        %Nviews PER SEGMENT than just overal, to more evenly divide things
        if TaskVariables.ComplexMotion.Code == 2%1 for circle, 2 for square, 
            Nsegments = TaskVariables.ComplexMotion.Nsegments;
        else
            Nsegments = 1;
        end
        
        
        points = length(qi(:,1));
        Nviews = ArmVariables(iRobots).NumberOfStillViews;
        if  Nviews > 2*TaskVariables.npoints;
            'Badly Defined Number of views, setting it equation to Npoints'
            Nviews = 2*TaskVariables.npoints;
        end
        indices = linspace(1,points,Nviews*Nsegments);
        
        figure(2)
        hold all
        for i = 1:length(indices)
            index =floor(indices(i));
            subplot(Nsegments,Nviews,i);
            hold all
            ifplot = 0;
            plotpayload_prism(Kinematics(iRobots).Payload.T_xyz(index,:),Kinematics(iRobots).Payload.T_rpy(index,:), TaskVariables.Lpayload,ifplot);
            bot1 = SerialLink(bot(iRobots),'name', [ArmVariables(iRobots).name 'step ' num2str(index)]);
            bot1.plot(qi(index,:),'jointdiam', .5,'linkcolor', ArmVariables(iRobots).linkcolor,'nobase','noshadow','noshading','notiles','noarrow','nowrist' )            
        end
        %
        %         %hold all
        %         subplot(1,Nviews,2)
        %         hold all
        %         plotpayload_prism(Kinematics(iRobots).Payload.T_xyz(dividby4,:),Kinematics(iRobots).Payload.T_rpy(dividby4,:), TaskVariables.Lpayload);
        %         bot1_point2 = SerialLink(bot,'name', [ArmVariables(iRobots).name 'point2']);
        %         bot1_point2.plot(qi(dividby4,:),'jointdiam', .5,'linkcolor', ArmVariables(iRobots).linkcolor,'nobase','noshadow','noshading','notiles','top')
        %         %hold all
        %
        %         subplot(1,4,3)
        %         hold all
        %         plotpayload_prism(Kinematics(iRobots).Payload.T_xyz(2*dividby4,:),Kinematics(iRobots).Payload.T_rpy(2*dividby4,:), TaskVariables.Lpayload);
        %         bot1_point3 = SerialLink(bot,'name', [ArmVariables(iRobots).name 'point3']);
        %         bot1_point3.plot(qi(2*dividby4,:),'jointdiam', .5,'linkcolor', ArmVariables(iRobots).linkcolor,'nobase','noshadow','noshading','notiles','top')
        %         %hold all
        %
        %         subplot(1,4,4)
        %         hold all
        %         plotpayload_prism(Kinematics(iRobots).Payload.T_xyz(tpoints*2,:),Kinematics(iRobots).Payload.T_rpy(tpoints*2,:), TaskVariables.Lpayload);
        %         bot1_end = SerialLink(bot,'name',[ArmVariables(iRobots).name 'end']);
        %         bot1_end.plot(qi(tpoints*2,:),'jointdiam', .5,'linkcolor', ArmVariables(iRobots).linkcolor,'nobase','noshadow','noshading','notiles','top')
        %
    end
end