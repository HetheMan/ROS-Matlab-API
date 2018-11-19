classdef Robot
    %Robot API.
    %Abstracts connection and provides methods to control a robot.
    %Properties are the scans of the sonars, lasers and odometry
    %initial position of the robot, initial orientation and the publishers. 
    %% Properties
        % -odometry Topic of robot movement.
        % -laser_0 Topic of laser sensor.
        % -sonar_X Topic of sonar sensors.
        % -pub_Motor_State Initialize real robot motors.
        % -msg_Motor_Enabler Motor state message to publish.
        % -pub_Movement odometry publisher
        % -msg_Movement odometry message to publich.
        % -init_Pos Starting position when we've created the robot.
        % -init_Ori Starting orientation when we've created the robot.
        % - rata Control Rate 10 Hz default.
     %% Methods
        % -move(Distance) Insert distance to move forward.
        % -rotate(Distance) Insert distance to rotate.
        % -get_Position() returns odometry.LatestMessage.Pose.Pose.Position
        % -get_Orientation() return odometry.LatestMessage.Pose.Pose.Orientation
    properties
        odom
        laser_0
        sonar_0
        sonar_1
        sonar_2
        sonar_3
        sonar_4
        sonar_5
        sonar_6
        sonar_7
		pub_Motor_State
        pub_Movement
        msg_Movement
		msg_Motor_Enabler
        init_Pos
        rate
        init_Ori
    end
    
    methods
        function obj = Robot(robot_Name,type)
            %Purpose: Constructor of the robot. Name of the robot must be the real name of the robots OR the robot's name inside the simulator.
			%Input: String robot_Name: The name of the robot, int type: 0 = Simulator, 1 = Real Robot. 
			%Output: robot object with the intialized sensors, the publishers and the message.
			switch type %#ok<ALIGN>
				case 0 %Simulador
					odom_Str = '/local_odom'; %local_odom if we have TF Tree simulator in ROS or /odom if we have default.
					laser_Str='/laser_0'; %Name of laser topic.
                    robot_Name = strcat('/',robot_Name); %Parser of name.
                otherwise  %Real AMIGOBOT.
					odom_Str = '/pose'; 
					laser_Str = '/scan';
                    cmd_Motor_State_Str = '/cmd_motor_state';
                    robot_Name = '';%No necesitamos nombre de robot.
            end
            %% Suscriber Generation
			 obj.odom = rossubscriber(strcat(robot_Name,odom_Str));
             obj.laser_0=rossubscriber(strcat(robot_Name,laser_Str)); 
             obj.sonar_0=rossubscriber(strcat(robot_Name,'/sonar_0')); 
             obj.sonar_1=rossubscriber(strcat(robot_Name,'/sonar_1'));
             obj.sonar_2=rossubscriber(strcat(robot_Name,'/sonar_2'));
             obj.sonar_3=rossubscriber(strcat(robot_Name,'/sonar_3'));
             obj.sonar_4=rossubscriber(strcat(robot_Name,'/sonar_4'));
             obj.sonar_5=rossubscriber(strcat(robot_Name,'/sonar_5'));
             obj.sonar_6=rossubscriber(strcat(robot_Name,'/sonar_6'));
             obj.sonar_7=rossubscriber(strcat(robot_Name,'/sonar_7'));
            %% Publisher Generation
             obj.pub_Movement = rospublisher(strcat(robot_Name,'/cmd_vel'), 'geometry_msgs/Twist');
             % GENERACIÓN DEL MENSAJE MOTOR ENABLER.
             if(type==1)
                obj.pub_Motor_State = rospublisher(cmd_Motor_State_Str,'std_msgs/Int32');
                obj.msg_Motor_Enabler = rosmessage(obj.pub_Motor_State);
                obj.msg_Motor_Enabler.Data = 1;
                send(obj.pub_Motor_State,obj.msg_Motor_Enabler);  
             end		       
            %% GENERACIÓN DEL MENSAJE TWIST.
            % Creamos un mensaje del tipo declarado en "pub" (geometry_obj.msg_Movements/Twist)
            obj.msg_Movement=rosmessage(obj.pub_Movement);
            obj.msg_Movement.Linear.X=0; 
            obj.msg_Movement.Linear.Y=0;
            obj.msg_Movement.Linear.Z=0;
            % Velocidades angulares (en robots diferenciales y entornos 2D solo se utilizarÃ¡ el valor Z) 
            obj.msg_Movement.Angular.X=0;
            obj.msg_Movement.Angular.Y=0;
            obj.msg_Movement.Angular.Z=0;
            %% Definimos la perodicidad del bucle (10 hz) //10 acciones por segundos
            obj.rate = robotics.Rate(10);
            %% Posiciones iniciales al coger robot.
            obj.init_Pos=obj.odom.LatestMessage.Pose.Pose.Position;
            obj.init_Ori = obj.odom.LatestMessage.Pose.Pose.Orientation;
        end 
        function move(obj,d)
            %Purpose: Command to move.
			%Input: Meters to move.
			%Output: VOID - Moves robot.
            pos=obj.odom.LatestMessage.Pose.Pose.Position;
            d = d + sqrt((obj.init_Pos.X-pos.X)^2+(obj.init_Pos.Y-pos.Y)^2); %Actualizamos distancia objetivo mas lo recorrido.;
            obj.msg_Movement.Linear.X=0.2;
            while(1)
                pos=obj.odom.LatestMessage.Pose.Pose.Position;
                plot(obj.laser_0.LatestMessage);
                dist=sqrt((obj.init_Pos.X-pos.X)^2+(obj.init_Pos.Y-pos.Y)^2); %Distancia euclidea en linea recta desde posición inicial a dónde estoy.
                disp("La distancia es: "+ dist)
                %% Si el robot se ha desplazado mÃ¡s de un metro detenemos el robot (velocidad lineal 0) y salimos del bucle
                if (dist>d)
                    obj.msg_Movement.Linear.X=0;
                    send(obj.pub_Movement,obj.msg_Movement);
                    break;
                else  % Comando de velocidad
                    send(obj.pub_Movement,obj.msg_Movement);
                end
                % Temporizador.
                waitfor(obj.rate)
            end
        end
        function rotate(obj,ang)
            %Purpose: Command to rotate.
			%Input: Meters to move.
			%Output: VOID - robot Rotado.
            obj.msg_Movement.Angular.Z=0.2;
            ori=obj.odom.LatestMessage.Pose.Pose.Orientation;
            init_Ang_Euler = quat2eul([obj.init_Ori.W,obj.init_Ori.X,obj.init_Ori.Y,obj.init_Ori.Z]);          
            ang_Euler =  quat2eul([ori.W,ori.X,ori.Y,ori.Z]);
            dist = ang_Euler(1) - init_Ang_Euler(1);
            ang = ang + dist; %Angulo objetivo más lo ya recorrido
            while(1)
                ori=obj.odom.LatestMessage.Pose.Pose.Orientation;
                ang_Euler =  quat2eul([ori.W,ori.X,ori.Y,ori.Z]);
                dist = ang_Euler(1) - init_Ang_Euler(1);
                %dist=sqrt((obj.init_Pos.X-pos.X)^2+(obj.init_Pos.Y-pos.Y)^2); %Distancia euclidea en linea recta desde posición inicial a dónde estoy.
                disp("Angulo girado:"+ dist)
                plot(obj.laser_0.LatestMessage);
                %% Si el robot se ha desplazado mÃ¡s de un metro detenemos el robot (velocidad lineal 0) y salimos del bucle
                if (dist>ang)
                    obj.msg_Movement.Angular.Z=0;
                    send(obj.pub_Movement,obj.msg_Movement);
                    break;
                else  % Comando de velocidad
                    send(obj.pub_Movement,obj.msg_Movement);
                end
                % Temporizador.
                waitfor(obj.rate)
          end
        end
        function y = get_Position(obj)
            y = obj.odom.LatestMessage.Pose.Pose.Position;
        end
        function y = get_Orientation(obj)
            ori=obj.odom.LatestMessage.Pose.Pose.Orientation;
            y = quat2eul([ori.W,ori.X,ori.Y,ori.Z]);
        end
    end
end

