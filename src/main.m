%% IP's configuration
IP_MASTER = 'http://192.168.1.61:11311'; %Poner la IP del ROS Master
IP_LOCAL = '192.168.1.30'; % Poner IP Local
%% Object Creation.
m = Map_Manager(IP_MASTER,IP_LOCAL);
m.conectar(); 
r=Robot('robot0',0); %0 para simulador, 1 para Real
%% Methods to try.
%m.mapping_known_poses(r) %Mapping con posición conocida.
%m.mapping_SLAM(r) %Mapping SLAM algorithm
%m.localization_MONTE(r,true,'417,343.png') %TRUE = Initial Pose Kown,
%FALSE = Initial Pose UnKnown, path of map


