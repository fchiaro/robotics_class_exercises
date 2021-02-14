# Ciao

Doc funzione `RobotState::getGlobalLinkTransform` (da riga 1325 di [qui](https://github.com/ros-planning/moveit/blob/master/moveit_core/robot_state/include/moveit/robot_state/robot_state.h)):  
Get the link transform w.r.t. the root link (model frame) of the RobotModel. This is typically the root link of the URDF unless a virtual joint is present.

=> nel `frame_id` sia della richiesta a movit che della mia risposta ho messo `RobotModel::getModelFrame()`.

Serve che sia in esecuzione fanuc_moveit_config demo affinché funzioni.
