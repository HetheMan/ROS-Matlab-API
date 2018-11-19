classdef Map_Manager
    %Manager of the map. Manages Localization, Mapping and connection.
    
    %% Constructor
    % MANAGER = map_Manager(ROS_MASTER_IP,LOCAL_IP) returns a map manager
    % object with the master IP with ROS_MASTER_IP and the ip where you are
    % running the matlab scripts with LOCAL_IP
    
    %% Properties
    % -resolution
    % -width
    % -height
    % -ROS_MASTER IP
    % -LOCAL_IP
    % -slamMap
    %% Methods
    % mapping_known_poses. Local Localization with only ray detection.
    % mapping_SLAM. Fill SLAM occupancy grid with trayectory rectification
    %   with loop closures. Obtain mapping with unknown localization.
    % localization_MONTE. Particle filters. Obtain location from grid
    %   particles with the montecarlo algorithm.
    properties
        resolution
        width
        height
        ROS_MASTER_IP %% Master IP
        LOCAL_IP %% IP of PC where matlab is running
        controlRate %% Needed so everything is smooth.
        slamMap %% Map of Slam.
    end
    methods
        function obj = Map_Manager(ROS_MASTER_IP,LOCAL_IP)
            %Constructor of the Map. This is the first object that gives 
            %functionality to the API. The functions of the map Manager
            %recieve a robot to and the robot functions are called from
            %within.
            
            %Connection Configuration
            obj.ROS_MASTER_IP = ROS_MASTER_IP;
            obj.LOCAL_IP = LOCAL_IP;
            %Occupancy map configuration
            obj.resolution = 20;
            obj.width = 40;
            obj.height = 40;
            %SLAM Configuration.
            obj.slamMap = robotics.LidarSLAM(obj.resolution,8);
            obj.slamMap.LoopClosureThreshold = 200;
            obj.slamMap.LoopClosureSearchRadius = 3;
            %Default control rate.
            obj.controlRate = robotics.Rate(10);
        end
        function conectar(obj)
            %% Conect to master node
            try
                rosinit(obj.ROS_MASTER_IP,'NodeHost',obj.LOCAL_IP);
            catch ME
                disp(ME);
                rosshutdown
                rosinit(obj.ROS_MASTER_IP,'NodeHost',obj.LOCAL_IP);
            end
        end
        function desconectar(obj)
            %Desconectar del nodo maestro.
            rosshutdown
        end
        function mapping_known_poses(obj,robot)
            %Purpose: Mapping with known poses.
			%Input: Robot to chase.
			%Output: VOID - Mapping with no correction on figure.
            map = robotics.OccupancyGrid(obj.width,obj.height,obj.resolution);
            map.GridLocationInWorld = [-5,-10]; %%Offset sobretodo para el local odom.
            figureHandle = figure('Name', 'Map');
            axesHandle = axes('Parent', figureHandle);
            mapHandle = show(map, 'Parent', axesHandle);
            title(axesHandle, 'OccupancyGrid: Update 0');
            updateCounter=0;
            while(1)
                updateCounter=updateCounter + 1;
                orientation = robot.get_Orientation();
                position = [robot.get_Position().X, robot.get_Position().Y];
                robotPose = [position, orientation(1)];
                scan = lidarScan(robot.laser_0.LatestMessage);
                %ranges(isinf(ranges)) = robot.laser_0.LatestMessage.RangeMax;
                modScan = lidarScan(scan.Ranges, scan.Angles); %Lidar Scan to insert on map.
                insertRay(map, robotPose, modScan,double(robot.laser_0.LatestMessage.RangeMax));
                if ~mod(updateCounter,50)
                    mapHandle.CData = occupancyMatrix(map);
                    title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
                end
                waitfor(obj.controlRate);
            end
        end
        function mapping_SLAM(obj,robot)
            %Purpose: Slam Mapping Algorithm
			%Input: Robot to chase.
			%Output: VOID - SLAM Mapping on figure.
            firstLoopClosure = false;
            %%Mapeamos hasta que salgamos y cuando salgamos imprimiremos.
            %El objeto de slam mapping se mantiene, lo tenemos que volver a lanzar
            %otra vez simplemente.
                for i=1:20
                     % Extract the laser scan
                        scan = lidarScan(robot.laser_0.LatestMessage);
                        modScan = lidarScan(scan.Ranges,scan.Angles);
                    [isScanAccepted, loopClosureInfo, optimizationInfo] = obj.slamMap.addScan(modScan);
                    if isScanAccepted
                        % Visualize how scans plot and poses are updated as robot navigate
                            show(obj.slamMap);
                    end
                    % Visualize the first detected loop closure
                    % firstLoopClosure flag is used to capture the first loop closure event
                    if optimizationInfo.IsPerformed && ~firstLoopClosure
                        firstLoopClosure = true;
                        show(obj.slamMap, 'Poses', 'off');
                        hold on;
                        show(obj.slamMap.PoseGraph);
                        hold off;
                        title('First loop closure');
                        snapnow
                    end
                   waitfor(obj.controlRate);
                end
                %%Execute when Ending process.
                % Plot the final built map after all scans are added to the |slamMap|
                % object.
                show(obj.slamMap, 'Poses', 'off'); 
                hold on;
                show(obj.slamMap.PoseGraph); 
                hold off;
                title({'Final Built Map of the Environment', 'Trajectory of the Robot'});
                %% Build Occupancy Grid Map
                % The optimized scans and poses can be used to generate a
                % |<docid:robotics_ref.bvaw60t-1 robotics.OccupancyGrid>| which represents
                % the environment as a probabilistic occupancy grid.
                [scans, optimizedPoses]  = scansAndPoses(obj.slamMap);
                map = buildMap(scans, optimizedPoses, obj.resolution, robot.laser_0.LatestMessage.RangeMax);
                %%
                % Visualize the occupancy grid map populated with the laser scans and the
                % optimized pose graph
                figure; 
                show(map);
                hold on
                show(obj.slamMap.PoseGraph, 'IDs', 'off');
                hold off
                title('Occupancy Grid Map Built Using Lidar SLAM');
        end
        function localization_MONTE(obj,robot,initial,path)
            %Purpose: Monte Carlo Algorithm
			%Input: Robot to chase, initial pose : Boolean. True = There is
                %known initial pose | False = There is not a known initial
                %pose ,path of map
			%Output: VOID - Monte Carlo localization on figure.
            map = obj.img_To_Map(path); %Input path
            map.GridLocationInWorld = [-7.977,-11.37];
            odometryModel = robotics.OdometryMotionModel;
            odometryModel.Noise = [0.2 0.2 0.2 0.2];
            rangeFinderModel = robotics.LikelihoodFieldSensorModel;
            rangeFinderModel.SensorLimits = [0.45 8];
            rangeFinderModel.Map = map;
            ori=robot.get_Orientation;
            rangeFinderModel.SensorPose = ...
             [robot.get_Position.X, robot.get_Position.Y,ori(1)];
            %% Montecarlo Localization Configuration
            amcl = robotics.MonteCarloLocalization;
            amcl.UseLidarScan = true;
            amcl.MotionModel = odometryModel;
            amcl.SensorModel = rangeFinderModel;   
            amcl.UpdateThresholds = [0.2,0.2,0.2];
            amcl.ResamplingInterval = 1;
            amcl.ParticleLimits = [300,5000];
            %mcl.InitialPose = [robot.get_Position.X -4 ,robot.get_Position.Y -8,0]; %Offsets desde donde empieza
            amcl.InitialCovariance = eye(3)*0.5;
            if ~initial
               amcl.GlobalLocalization = true; 
            end
            visualizationHelper = ExampleHelperAMCLVisualization(map);
            %Localization
            numUpdates = 60;
            i = 0;
            %% Start of loop scanning.
            while i < numUpdates
                % Receive laser scan and odometry message.
                scanMsg = robot.laser_0.LatestMessage;
                odompose = robot.odom.LatestMessage;

                % Create lidarScan object to pass to the AMCL object.
                scan = lidarScan(scanMsg);

                % For sensors that are mounted upside down, you need to reverse the
                % order of scan angle readings using 'flip' function.

                % Compute robot's pose [x,y,yaw] from odometry message.
                odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
                    odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
                odomRotation = quat2eul(odomQuat);
                pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

                % Update estimated robot's pose and covariance using new odometry and
                % sensor readings.
                [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);

                % Plot the robot's estimated pose, particles and laser scans on the map.
                if isUpdated
                    i = i + 1;
                    plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
                end
                waitfor(obj.controlRate)
            end
        end
        function y = img_To_Map(obj,path)
           %Mostly used as private function, but can be used public too.
           image = imread(path); 
           imageCropped = rgb2gray(image);
           imageNorm = double(imageCropped)/255;
           imageOccupancy = 1 - imageNorm;
           y = robotics.OccupancyGrid(imageOccupancy,417/27.947);  %pixels/metros(medidos antes de convertir a imagen)
        end
    end
    
end

