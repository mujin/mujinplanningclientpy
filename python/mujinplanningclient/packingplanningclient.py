# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 Mujin, Inc.
# AUTO GENERATED FILE! DO NOT EDIT!

# system imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from typing import Any, Dict, List, Optional, Tuple, Union # noqa: F401 # used in type check

# mujin imports
from . import zmq
from . import realtimerobotplanningclient

# logging
import logging
log = logging.getLogger(__name__)


class PackingPlanningClient(realtimerobotplanningclient.RealtimeRobotPlanningClient):
    """Mujin planning client for the Packing task"""

    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(
        self,
        
        taskzmqport=11000,
        taskheartbeatport=11001,
        taskheartbeattimeout=7.0,
        tasktype='packing',
        ctx=None,
        slaverequestid=None,
        controllerip='',
        controllerurl='',
        controllerusername='',
        controllerpassword='',
        scenepk='',
        callerid='',
        **ignoredArgs
    ):
        # type: (, int, int, float, str, Optional[zmq.Context], Optional[str], str, str, str, str, str, str, Any) -> None
        """Connects to the Mujin controller, initializes Packing task and sets up parameters

        Args:
            
            taskzmqport (int, optional): Port of the task's ZMQ server, e.g. 7110. (Default: 11000)
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher, e.g. 7111. (Default: 11001)
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: packing
            ctx (zmq.Context, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            slaverequestid:
            controllerip (str): IP or hostname of the mujin controller, e.g. 172.17.0.2 or controller123
            controllerurl (str, optional): (Deprecated. Use controllerip instead) URL of the mujin controller, e.g. http://controller14.
            controllerusername (str): Username for the Mujin controller, e.g. testuser
            controllerpassword (str): Password for the Mujin controller
            scenepk (str, optional): Primary key (pk) of the scene, e.g. irex_demo.mujin.dae
            callerid (str, optional): Caller identifier to send to server on every command
            ignoredArgs: Additional keyword args are not used, but allowed for easy initialization from a dictionary
        """
        
        super(PackingPlanningClient, self).__init__(
            
            taskzmqport=taskzmqport,
            taskheartbeatport=taskheartbeatport,
            taskheartbeattimeout=taskheartbeattimeout,
            tasktype=tasktype,
            ctx=ctx,
            slaverequestid=slaverequestid,
            controllerip=controllerip,
            controllerurl=controllerurl,
            controllerusername=controllerusername,
            controllerpassword=controllerpassword,
            scenepk=scenepk,
            callerid=callerid
        )


    #
    # Commands (generated from the spec)
    #

    def StartPackFormationComputationThread(
        self,
        timeout=10,  # type: float
        debuglevel=4,  # type: int
        toolname=None,  # type: Optional[str]
        dynamicEnvironmentState=None,  # type: Optional[types.StartPackFormationComputationThreadParametersDynamicEnvironmentState]
        minDetectionImageTimeMS=None,  # type: Optional[int]
        detectionInfos=None,  # type: Optional[list[types.StartPackFormationComputationThreadParametersDetectionInfosArrayElement]]
        pickLocationInfo=None,  # type: Optional[types.PickLocationInfo]
        placeLocationInfos=None,  # type: Optional[list[types.PlaceLocationInfo]]
        packLocationInfo=None,  # type: Optional[types.StartPackFormationComputationThreadParametersPackLocationInfo]
        predictDetectionInfo=None,  # type: Optional[types.PredictDetectionInfo]
        useLocationState=None,  # type: Optional[bool]
        sourcecontainername=None,  # type: Optional[str]
        bodyOcclusionInfos=None,  # type: Optional[list[types.StartPackFormationComputationThreadParametersBodyOcclusionInfosArrayElement]]
        forceWaitDestContainer=None,  # type: Optional[bool]
        waitUpdateStampTimeout=None,  # type: Optional[int]
        manipname=None,  # type: Optional[str]
        constraintToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        constraintToolOptions=None,  # type: Optional[int]
        envclearance=None,  # type: Optional[float]
        cameraOcclusionOffset=None,  # type: Optional[float]
        robotspeedmult=None,  # type: Optional[float]
        robotaccelmult=None,  # type: Optional[float]
        executionFilterFactor=None,  # type: Optional[float]
        executionConnectingTrajDecelMult=None,  # type: Optional[float]
        executionConnectingTrajReverseMult=None,  # type: Optional[float]
        executionReverseRecoveryDistance=None,  # type: Optional[float]
        locationCollisionInfos=None,  # type: Optional[list[types.StartPackFormationComputationThreadParametersLocationCollisionInfosArrayElement]]
        destSensorSelectionInfos=None,  # type: Optional[list[types.StartPackFormationComputationThreadParametersDestSensorSelectionInfosArrayElement]]
        finalPlanMode=None,  # type: Optional[str]
        pathPlannerParameters=None,  # type: Optional[types.PathPlannerParameters]
        jittererParameters=None,  # type: Optional[types.JittererParameters]
        forceTorqueBasedEstimatorParameters=None,  # type: Optional[types.ForceTorqueBasedEstimatorParameters]
        smootherParameters=None,  # type: Optional[types.SmoothingParameters]
        homePositionConfigurationName=None,  # type: Optional[str]
        cycleStartPositionConfigurationName=None,  # type: Optional[str]
        recoveryPositionConfigurationName=None,  # type: Optional[str]
        finalPlanPositionConfigurationName=None,  # type: Optional[str]
        ignoreStartPosition=None,  # type: Optional[bool]
        forceMoveToFinish=None,  # type: Optional[bool]
        ignoreFinishPosition=None,  # type: Optional[bool]
        ignoreFinishPositionUnlessPackFormationComplete=None,  # type: Optional[bool]
        justInTimeToolChangePlanning=None,  # type: Optional[types.StartPackFormationComputationThreadParametersJustInTimeToolChangePlanning]
        constraintDuringGrabbingToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        maxGrabbingManipSpeed=None,  # type: Optional[float]
        maxGrabbingManipAccel=None,  # type: Optional[float]
        maxFreeManipSpeed=None,  # type: Optional[float]
        maxFreeManipAccel=None,  # type: Optional[float]
        constraintToolInfo=None,  # type: Optional[types.ConstraintToolInfo]
        unit='mm',  # type: str
        unitMass='kg',  # type: str
        robotname=None,  # type: Optional[str]
        destcontainernames=None,  # type: Optional[list[str]]
        locationName=None,  # type: Optional[str]
        containername=None,  # type: Optional[str]
        packContainerType=None,  # type: Optional[str]
        packInputPartInfos=None,  # type: Optional[list[types.StartPackFormationComputationThreadParametersPackInputPartInfosArrayElement]]
        packFormationParameters=None,  # type: Optional[types.PackFormationParameters]
        dynamicGoalsGeneratorParameters=None,  # type: Optional[types.StartPackFormationComputationThreadParametersDynamicGoalsGeneratorParameters]
        distanceMeasurementInfo=None,  # type: Optional[types.DistanceMeasurementInfo]
        savePackingState=None,  # type: Optional[bool]
        checkObstacleNames=None,  # type: Optional[list[str]]
        targetMinBottomPaddingForInitialTransfer=40,  # type: float
        targetMinSafetyHeightForInitialTransfer=None,  # type: Optional[float]
        saveDynamicGoalGeneratorState=False,  # type: bool
        saveDynamicGoalGeneratorStateFailed=True,  # type: bool
        executionmode=None,  # type: Optional[str]
        csvHeader=_deprecated,  # type: Optional[list[str]]
        packContainerId=_deprecated,  # type: Optional[str]
        packLocationName=_deprecated,  # type: Optional[str]
        containerNameTemplate=_deprecated,  # type: Optional[str]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Starts a background loop to copmute packing formation.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: 4)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            minDetectionImageTimeMS: (Default: None)
            detectionInfos: (Default: None)
            pickLocationInfo: Describes the pick location and properties about it to initialize the cycle. (Default: None)
            placeLocationInfos: Describes the place locations and properties about them to initialize the cycle. (Default: None)
            packLocationInfo: (Default: None)
            predictDetectionInfo: Describes where to place predicted target before detection results. (Default: None)
            useLocationState: (Default: None)
            sourcecontainername: (Default: None)
            bodyOcclusionInfos: (Default: None)
            forceWaitDestContainer: (Default: None)
            waitUpdateStampTimeout: (Default: None)
            manipname: (Default: None)
            constraintToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            constraintToolOptions: (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            cameraOcclusionOffset: (Default: None)
            robotspeedmult: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            executionFilterFactor: (Default: None)
            executionConnectingTrajDecelMult: (Default: None)
            executionConnectingTrajReverseMult: (Default: None)
            executionReverseRecoveryDistance: (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            destSensorSelectionInfos: (Default: None)
            finalPlanMode: (Default: None)
            pathPlannerParameters: Parameters for robot path planning. (Default: None)
            jittererParameters: Parameters dealing with jittering the robot out of collisions. (Default: None)
            forceTorqueBasedEstimatorParameters: Parameters for state estimation features based on force torque sensor (Default: None)
            smootherParameters: Parameters dealing with getting smoother paths for the robot planning. (Default: None)
            homePositionConfigurationName: (Default: None)
            cycleStartPositionConfigurationName: (Default: None)
            recoveryPositionConfigurationName: (Default: None)
            finalPlanPositionConfigurationName: (Default: None)
            ignoreStartPosition: True if the robot should ignore going to start position (Default: None)
            forceMoveToFinish: If True, then the robot will add a finish position to the cycle even if "finish" is not present, unless "ignoreFinishPosition" is True in the order cycle command. "ignoreFinishPosition" overrides this parameter. (Default: None)
            ignoreFinishPosition: (Default: None)
            ignoreFinishPositionUnlessPackFormationComplete: (Default: None)
            justInTimeToolChangePlanning: (Default: None)
            constraintDuringGrabbingToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            maxGrabbingManipSpeed: (Default: None)
            maxGrabbingManipAccel: (Default: None)
            maxFreeManipSpeed: (Default: None)
            maxFreeManipAccel: (Default: None)
            constraintToolInfo: Constrain a direction on the tool to be within a certain angle with respect to a global direction. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            unitMass: (Default: 'kg')
            robotname: Name of the robot (Default: None)
            destcontainernames: (Default: None)
            locationName: (Default: None)
            containername: (Default: None)
            packContainerType: (Default: None)
            packInputPartInfos: (Default: None)
            packFormationParameters: Parameters controlling the packing behaviors and algorithms for startPackFormationComputation command. (Default: None)
            dynamicGoalsGeneratorParameters: If 'useDynamicGoals' is True, then will be dynamically generating goals based how to call on the packing algorithms. Internally, the packing algorithms parameters are managed by packFormation profiles.
             (Default: None)
            distanceMeasurementInfo: Parameters for measuring the height of a target object with a 1D distance sensor. Setting up these parameters will send timed IO values as the robot trajectory is executed. (Default: None)
            savePackingState: (Default: None)
            checkObstacleNames: (Default: None)
            targetMinBottomPaddingForInitialTransfer: The amount of padding that is added to the bottom of the target when moving the target out of the source container to the next position (dest, middest, or scan position). This is used to raise the part higher when moving out of the source container.

            Increasing this parameter increases the clearance at the bottom of the part when it is moved out of the source container.

            Cannot be used together with targetMinSafetyHeightForInitialTransfer.

            Only applied during the initial transfer out of the source container. Subsequent transfers ignore this setting. (Default: 40)
            targetMinSafetyHeightForInitialTransfer: Extends the height of the target to this value when moving the target out of the source container to the next position (dest, middest, or scan position). This is used to raise the part higher when moving out of the source container.

            Increasing this parameter increases the clearance at the bottom of the part when it is moved out of the source container.

            Cannot be used together with targetMinBottomPaddingForInitialTransfer.

            Only applied during the initial transfer out of the source container. Subsequent transfers ignore this setting. (Default: None)
            saveDynamicGoalGeneratorState: If True, will always save the dynamic goal generator state for later playerback. (Default: False)
            saveDynamicGoalGeneratorStateFailed: If true and logging level is info or higher, saves state of the _dynamicGoalsGenerator to the disk. (Default: True)
            executionmode: (Default: None)
            csvHeader: **deprecated** (Default: None)
            packContainerId: **deprecated** (Default: None)
            packLocationName: **deprecated** (Default: None)
            containerNameTemplate: **deprecated** (Default: None)
        """
        taskparameters = {
            'command': 'StartPackFormationComputationThread',
        }  # type: dict[str, Any]
        if debuglevel != 4:
            taskparameters['debuglevel'] = debuglevel
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if minDetectionImageTimeMS is not None:
            taskparameters['minDetectionImageTimeMS'] = minDetectionImageTimeMS
        if detectionInfos is not None:
            taskparameters['detectionInfos'] = detectionInfos
        if pickLocationInfo is not None:
            taskparameters['pickLocationInfo'] = pickLocationInfo
        if placeLocationInfos is not None:
            taskparameters['placeLocationInfos'] = placeLocationInfos
        if packLocationInfo is not None:
            taskparameters['packLocationInfo'] = packLocationInfo
        if predictDetectionInfo is not None:
            taskparameters['predictDetectionInfo'] = predictDetectionInfo
        if useLocationState is not None:
            taskparameters['useLocationState'] = useLocationState
        if sourcecontainername is not None:
            taskparameters['sourcecontainername'] = sourcecontainername
        if bodyOcclusionInfos is not None:
            taskparameters['bodyOcclusionInfos'] = bodyOcclusionInfos
        if forceWaitDestContainer is not None:
            taskparameters['forceWaitDestContainer'] = forceWaitDestContainer
        if waitUpdateStampTimeout is not None:
            taskparameters['waitUpdateStampTimeout'] = waitUpdateStampTimeout
        if manipname is not None:
            taskparameters['manipname'] = manipname
        if constraintToolDirection is not None:
            taskparameters['constraintToolDirection'] = constraintToolDirection
        if constraintToolOptions is not None:
            taskparameters['constraintToolOptions'] = constraintToolOptions
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if cameraOcclusionOffset is not None:
            taskparameters['cameraOcclusionOffset'] = cameraOcclusionOffset
        if robotspeedmult is not None:
            taskparameters['robotspeedmult'] = robotspeedmult
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if executionFilterFactor is not None:
            taskparameters['executionFilterFactor'] = executionFilterFactor
        if executionConnectingTrajDecelMult is not None:
            taskparameters['executionConnectingTrajDecelMult'] = executionConnectingTrajDecelMult
        if executionConnectingTrajReverseMult is not None:
            taskparameters['executionConnectingTrajReverseMult'] = executionConnectingTrajReverseMult
        if executionReverseRecoveryDistance is not None:
            taskparameters['executionReverseRecoveryDistance'] = executionReverseRecoveryDistance
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if destSensorSelectionInfos is not None:
            taskparameters['destSensorSelectionInfos'] = destSensorSelectionInfos
        if finalPlanMode is not None:
            taskparameters['finalPlanMode'] = finalPlanMode
        if pathPlannerParameters is not None:
            taskparameters['pathPlannerParameters'] = pathPlannerParameters
        if jittererParameters is not None:
            taskparameters['jittererParameters'] = jittererParameters
        if forceTorqueBasedEstimatorParameters is not None:
            taskparameters['forceTorqueBasedEstimatorParameters'] = forceTorqueBasedEstimatorParameters
        if smootherParameters is not None:
            taskparameters['smootherParameters'] = smootherParameters
        if homePositionConfigurationName is not None:
            taskparameters['homePositionConfigurationName'] = homePositionConfigurationName
        if cycleStartPositionConfigurationName is not None:
            taskparameters['cycleStartPositionConfigurationName'] = cycleStartPositionConfigurationName
        if recoveryPositionConfigurationName is not None:
            taskparameters['recoveryPositionConfigurationName'] = recoveryPositionConfigurationName
        if finalPlanPositionConfigurationName is not None:
            taskparameters['finalPlanPositionConfigurationName'] = finalPlanPositionConfigurationName
        if ignoreStartPosition is not None:
            taskparameters['ignoreStartPosition'] = ignoreStartPosition
        if forceMoveToFinish is not None:
            taskparameters['forceMoveToFinish'] = forceMoveToFinish
        if ignoreFinishPosition is not None:
            taskparameters['ignoreFinishPosition'] = ignoreFinishPosition
        if ignoreFinishPositionUnlessPackFormationComplete is not None:
            taskparameters['ignoreFinishPositionUnlessPackFormationComplete'] = ignoreFinishPositionUnlessPackFormationComplete
        if justInTimeToolChangePlanning is not None:
            taskparameters['justInTimeToolChangePlanning'] = justInTimeToolChangePlanning
        if constraintDuringGrabbingToolDirection is not None:
            taskparameters['constraintDuringGrabbingToolDirection'] = constraintDuringGrabbingToolDirection
        if maxGrabbingManipSpeed is not None:
            taskparameters['maxGrabbingManipSpeed'] = maxGrabbingManipSpeed
        if maxGrabbingManipAccel is not None:
            taskparameters['maxGrabbingManipAccel'] = maxGrabbingManipAccel
        if maxFreeManipSpeed is not None:
            taskparameters['maxFreeManipSpeed'] = maxFreeManipSpeed
        if maxFreeManipAccel is not None:
            taskparameters['maxFreeManipAccel'] = maxFreeManipAccel
        if constraintToolInfo is not None:
            taskparameters['constraintToolInfo'] = constraintToolInfo
        if unit != 'mm':
            taskparameters['unit'] = unit
        if unitMass != 'kg':
            taskparameters['unitMass'] = unitMass
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if destcontainernames is not None:
            taskparameters['destcontainernames'] = destcontainernames
        if locationName is not None:
            taskparameters['locationName'] = locationName
        if containername is not None:
            taskparameters['containername'] = containername
        if packContainerType is not None:
            taskparameters['packContainerType'] = packContainerType
        if packInputPartInfos is not None:
            taskparameters['packInputPartInfos'] = packInputPartInfos
        if packFormationParameters is not None:
            taskparameters['packFormationParameters'] = packFormationParameters
        if dynamicGoalsGeneratorParameters is not None:
            taskparameters['dynamicGoalsGeneratorParameters'] = dynamicGoalsGeneratorParameters
        if distanceMeasurementInfo is not None:
            taskparameters['distanceMeasurementInfo'] = distanceMeasurementInfo
        if savePackingState is not None:
            taskparameters['savePackingState'] = savePackingState
        if checkObstacleNames is not None:
            taskparameters['checkObstacleNames'] = checkObstacleNames
        if targetMinBottomPaddingForInitialTransfer != 40:
            taskparameters['targetMinBottomPaddingForInitialTransfer'] = targetMinBottomPaddingForInitialTransfer
        if targetMinSafetyHeightForInitialTransfer is not None:
            taskparameters['targetMinSafetyHeightForInitialTransfer'] = targetMinSafetyHeightForInitialTransfer
        if saveDynamicGoalGeneratorState != False:
            taskparameters['saveDynamicGoalGeneratorState'] = saveDynamicGoalGeneratorState
        if saveDynamicGoalGeneratorStateFailed != True:
            taskparameters['saveDynamicGoalGeneratorStateFailed'] = saveDynamicGoalGeneratorStateFailed
        if executionmode is not None:
            taskparameters['executionmode'] = executionmode
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, toolname=toolname)
    def StartSingleSKUPackFormationComputation(self, partType=None, userPackFormationParameters=None, toolname=None, patternName=None, packLocationInfo=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (Optional[Any], Optional[Any], Optional[Any], Optional[Any], Optional[Any], float, Optional[types.StartSingleSKUPackFormationComputationParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Starts a background loop to compute packing formation.

        Args:
            partType: (Default: None)
            userPackFormationParameters: (Default: None)
            toolname: (Default: None)
            patternName: (Default: None)
            packLocationInfo: (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'StartSingleSKUPackFormationComputation',
        }  # type: dict[str, Any]
        if partType is not None:
            taskparameters['partType'] = partType
        if userPackFormationParameters is not None:
            taskparameters['userPackFormationParameters'] = userPackFormationParameters
        if patternName is not None:
            taskparameters['patternName'] = patternName
        if packLocationInfo is not None:
            taskparameters['packLocationInfo'] = packLocationInfo
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout)
    def StopPackFormationComputationThread(self, finishStatus=None, finishMessage=None, timeout=10, fireandforget=False, blockwait=False, dynamicEnvironmentState=None, debuglevel=None, robotname=None, toolname=None, callerid=None, stamp=None, command=None, robotBridgeConnectionInfo=None, robotspeed=None, robotaccelmult=None, **kwargs):
        # type: (Optional[str], Optional[str], float, bool, bool, Optional[types.StopPackFormationComputationThreadParametersDynamicEnvironmentState], Optional[int], Optional[str], Optional[str], Optional[str], Optional[float], Optional[str], Optional[types.StopPackFormationComputationThreadParametersRobotBridgeConnectionInfo], Optional[float], Optional[float], Optional[Any]) -> Optional[Any]
        """
        Stops the packing computation thread thread started with StartPackFormationComputationThread

        Args:
            finishStatus: One of FinishX official statues to end the cycle with. (Default: None)
            finishMessage: Finish message to end the cycle with. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait: Same as fireandforget, except will be able to receive return later with WaitForCommandResponse. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            callerid: (Default: None)
            stamp: The timestamp of when the command was sent, in seconds. (Default: None)
            command: (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
        """
        taskparameters = {
            'command': 'StopPackFormationComputationThread',
        }  # type: dict[str, Any]
        if finishStatus is not None:
            taskparameters['finishStatus'] = finishStatus
        if finishMessage is not None:
            taskparameters['finishMessage'] = finishMessage
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if callerid is not None:
            taskparameters['callerid'] = callerid
        if stamp is not None:
            taskparameters['stamp'] = stamp
        if command is not None:
            taskparameters['command'] = command
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)
    def VisualizePackingState(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[types.VisualizePackingStateParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Stops the packing computation thread thread started with StartPackFormationComputationThread

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'VisualizePackingState',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)
    def GetPackFormationSolution(self, timeout=10, fireandforget=False, blockwait=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, bool, Optional[types.GetPackFormationSolutionParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Stops the packing computation thread thread started with StartPackFormationComputationThread

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait: Same as fireandforget, except will be able to receive return later with WaitForCommandResponse. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetPackFormationSolution',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)
    def SendPackFormationComputationResult(self, timeout=10, fireandforget=False, blockwait=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, bool, Optional[types.SendPackFormationComputationResultParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Stops the packing computation thread thread started with StartPackFormationComputationThread

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait: Same as fireandforget, except will be able to receive return later with WaitForCommandResponse. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SendPackFormationComputationResult',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)
    def GetLatestPackFormationResultList(self, timeout=10, fireandforget=False, blockwait=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, bool, Optional[types.GetLatestPackFormationResultListParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[list[types.PackFormation]]
        """
        Gets latest pack formation computation result

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait: Same as fireandforget, except will be able to receive return later with WaitForCommandResponse. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetLatestPackFormationResultList',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)
    def StartValidatePackFormation(self, packFormationResultList, timeout=10, fireandforget=False, blockwait=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (Any, float, bool, bool, Optional[types.StartValidatePackFormationParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> None
        """
        Validates pack formation result list and computes info (fillRatio, packageDimensions, packedItemsInfo, etc) about it.

        Args:
            packFormationResultList:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait: Same as fireandforget, except will be able to receive return later with WaitForCommandResponse. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'StartValidatePackFormation',
            'packFormationResultList': packFormationResultList,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)
    def ValidatePackFormationResultList(
        self,
        packFormationResultList,  # type: list[types.PackFormation]
        timeout=10,  # type: float
        fireandforget=False,  # type: bool
        dynamicEnvironmentState=None,  # type: Optional[types.ValidatePackFormationResultListParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        unitMass='kg',  # type: str
        robotname=None,  # type: Optional[str]
        toolname=None,  # type: Optional[str]
        destcontainernames=None,  # type: Optional[list[str]]
        packLocationInfo=None,  # type: Optional[types.ValidatePackFormationResultListParametersPackLocationInfo]
        locationName=None,  # type: Optional[str]
        containername=None,  # type: Optional[str]
        packContainerType=None,  # type: Optional[str]
        packInputPartInfos=None,  # type: Optional[list[types.ValidatePackFormationResultListParametersPackInputPartInfosArrayElement]]
        packFormationParameters=None,  # type: Optional[types.PackFormationParameters]
        dynamicGoalsGeneratorParameters=None,  # type: Optional[types.ValidatePackFormationResultListParametersDynamicGoalsGeneratorParameters]
        constraintToolInfo=None,  # type: Optional[types.ConstraintToolInfo]
        distanceMeasurementInfo=None,  # type: Optional[types.DistanceMeasurementInfo]
        savePackingState=None,  # type: Optional[bool]
        checkObstacleNames=None,  # type: Optional[list[str]]
        targetMinBottomPaddingForInitialTransfer=40,  # type: float
        targetMinSafetyHeightForInitialTransfer=None,  # type: Optional[float]
        saveDynamicGoalGeneratorState=False,  # type: bool
        saveDynamicGoalGeneratorStateFailed=True,  # type: bool
        doPlacementValidation=None,  # type: Optional[bool]
        forceValidatePackContainerType=None,  # type: Optional[bool]
        packLocationName=_deprecated,  # type: Optional[str]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[types.ValidatePackFormationResultListReturns]
        """
        Validates pack formation result list and compute info (fillRatio, packageDimensions, packedItemsInfo, etc) about it.
        
        kwargs are expected to be packing parameters.

        Args:
            packFormationResultList:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            unitMass: (Default: 'kg')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            destcontainernames: (Default: None)
            packLocationInfo: (Default: None)
            locationName: (Default: None)
            containername: (Default: None)
            packContainerType: (Default: None)
            packInputPartInfos: (Default: None)
            packFormationParameters: Parameters controlling the packing behaviors and algorithms for startPackFormationComputation command. (Default: None)
            dynamicGoalsGeneratorParameters: If 'useDynamicGoals' is True, then will be dynamically generating goals based how to call on the packing algorithms. Internally, the packing algorithms parameters are managed by packFormation profiles.
             (Default: None)
            constraintToolInfo: Constrain a direction on the tool to be within a certain angle with respect to a global direction. (Default: None)
            distanceMeasurementInfo: Parameters for measuring the height of a target object with a 1D distance sensor. Setting up these parameters will send timed IO values as the robot trajectory is executed. (Default: None)
            savePackingState: (Default: None)
            checkObstacleNames: (Default: None)
            targetMinBottomPaddingForInitialTransfer: The amount of padding that is added to the bottom of the target when moving the target out of the source container to the next position (dest, middest, or scan position). This is used to raise the part higher when moving out of the source container.

            Increasing this parameter increases the clearance at the bottom of the part when it is moved out of the source container.

            Cannot be used together with targetMinSafetyHeightForInitialTransfer.

            Only applied during the initial transfer out of the source container. Subsequent transfers ignore this setting. (Default: 40)
            targetMinSafetyHeightForInitialTransfer: Extends the height of the target to this value when moving the target out of the source container to the next position (dest, middest, or scan position). This is used to raise the part higher when moving out of the source container.

            Increasing this parameter increases the clearance at the bottom of the part when it is moved out of the source container.

            Cannot be used together with targetMinBottomPaddingForInitialTransfer.

            Only applied during the initial transfer out of the source container. Subsequent transfers ignore this setting. (Default: None)
            saveDynamicGoalGeneratorState: If True, will always save the dynamic goal generator state for later playerback. (Default: False)
            saveDynamicGoalGeneratorStateFailed: If true and logging level is info or higher, saves state of the _dynamicGoalsGenerator to the disk. (Default: True)
            doPlacementValidation: If True will do placmeent validation by calling FindNextFreePlacementGoals for each placed item. (Default: None)
            forceValidatePackContainerType: (Default: None)
            packLocationName: **deprecated** (Default: None)
        """
        taskparameters = {
            'command': 'ValidatePackFormationResultList',
            'packFormationResultList': packFormationResultList,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if unitMass != 'kg':
            taskparameters['unitMass'] = unitMass
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if destcontainernames is not None:
            taskparameters['destcontainernames'] = destcontainernames
        if packLocationInfo is not None:
            taskparameters['packLocationInfo'] = packLocationInfo
        if locationName is not None:
            taskparameters['locationName'] = locationName
        if containername is not None:
            taskparameters['containername'] = containername
        if packContainerType is not None:
            taskparameters['packContainerType'] = packContainerType
        if packInputPartInfos is not None:
            taskparameters['packInputPartInfos'] = packInputPartInfos
        if packFormationParameters is not None:
            taskparameters['packFormationParameters'] = packFormationParameters
        if dynamicGoalsGeneratorParameters is not None:
            taskparameters['dynamicGoalsGeneratorParameters'] = dynamicGoalsGeneratorParameters
        if constraintToolInfo is not None:
            taskparameters['constraintToolInfo'] = constraintToolInfo
        if distanceMeasurementInfo is not None:
            taskparameters['distanceMeasurementInfo'] = distanceMeasurementInfo
        if savePackingState is not None:
            taskparameters['savePackingState'] = savePackingState
        if checkObstacleNames is not None:
            taskparameters['checkObstacleNames'] = checkObstacleNames
        if targetMinBottomPaddingForInitialTransfer != 40:
            taskparameters['targetMinBottomPaddingForInitialTransfer'] = targetMinBottomPaddingForInitialTransfer
        if targetMinSafetyHeightForInitialTransfer is not None:
            taskparameters['targetMinSafetyHeightForInitialTransfer'] = targetMinSafetyHeightForInitialTransfer
        if saveDynamicGoalGeneratorState != False:
            taskparameters['saveDynamicGoalGeneratorState'] = saveDynamicGoalGeneratorState
        if saveDynamicGoalGeneratorStateFailed != True:
            taskparameters['saveDynamicGoalGeneratorStateFailed'] = saveDynamicGoalGeneratorStateFailed
        if doPlacementValidation is not None:
            taskparameters['doPlacementValidation'] = doPlacementValidation
        if forceValidatePackContainerType is not None:
            taskparameters['forceValidatePackContainerType'] = forceValidatePackContainerType
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)
    def ComputeSamePartPackResultBySimulation(self, timeout=100, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[types.ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Computes pack formation for single part type.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 100)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ComputeSamePartPackResultBySimulation',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)
    def HasDetectionObstacles(
        self,
        timeout=100,  # type: float
        dynamicEnvironmentState=None,  # type: Optional[types.HasDetectionObstaclesParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        minDetectionImageTimeMS=None,  # type: Optional[int]
        detectionInfos=None,  # type: Optional[list[types.HasDetectionObstaclesParametersDetectionInfosArrayElement]]
        pickLocationInfo=None,  # type: Optional[types.PickLocationInfo]
        placeLocationInfos=None,  # type: Optional[list[types.PlaceLocationInfo]]
        packLocationInfo=None,  # type: Optional[types.HasDetectionObstaclesParametersPackLocationInfo]
        predictDetectionInfo=None,  # type: Optional[types.PredictDetectionInfo]
        useLocationState=None,  # type: Optional[bool]
        sourcecontainername=None,  # type: Optional[str]
        bodyOcclusionInfos=None,  # type: Optional[list[types.HasDetectionObstaclesParametersBodyOcclusionInfosArrayElement]]
        forceWaitDestContainer=None,  # type: Optional[bool]
        waitUpdateStampTimeout=None,  # type: Optional[int]
        manipname=None,  # type: Optional[str]
        constraintToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        constraintToolOptions=None,  # type: Optional[int]
        envclearance=None,  # type: Optional[float]
        cameraOcclusionOffset=None,  # type: Optional[float]
        robotspeedmult=None,  # type: Optional[float]
        robotaccelmult=None,  # type: Optional[float]
        executionFilterFactor=None,  # type: Optional[float]
        executionConnectingTrajDecelMult=None,  # type: Optional[float]
        executionConnectingTrajReverseMult=None,  # type: Optional[float]
        executionReverseRecoveryDistance=None,  # type: Optional[float]
        locationCollisionInfos=None,  # type: Optional[list[types.HasDetectionObstaclesParametersLocationCollisionInfosArrayElement]]
        destSensorSelectionInfos=None,  # type: Optional[list[types.HasDetectionObstaclesParametersDestSensorSelectionInfosArrayElement]]
        finalPlanMode=None,  # type: Optional[str]
        pathPlannerParameters=None,  # type: Optional[types.PathPlannerParameters]
        jittererParameters=None,  # type: Optional[types.JittererParameters]
        forceTorqueBasedEstimatorParameters=None,  # type: Optional[types.ForceTorqueBasedEstimatorParameters]
        smootherParameters=None,  # type: Optional[types.SmoothingParameters]
        homePositionConfigurationName=None,  # type: Optional[str]
        cycleStartPositionConfigurationName=None,  # type: Optional[str]
        recoveryPositionConfigurationName=None,  # type: Optional[str]
        finalPlanPositionConfigurationName=None,  # type: Optional[str]
        ignoreStartPosition=None,  # type: Optional[bool]
        forceMoveToFinish=None,  # type: Optional[bool]
        ignoreFinishPosition=None,  # type: Optional[bool]
        ignoreFinishPositionUnlessPackFormationComplete=None,  # type: Optional[bool]
        justInTimeToolChangePlanning=None,  # type: Optional[types.HasDetectionObstaclesParametersJustInTimeToolChangePlanning]
        constraintDuringGrabbingToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        maxGrabbingManipSpeed=None,  # type: Optional[float]
        maxGrabbingManipAccel=None,  # type: Optional[float]
        maxFreeManipSpeed=None,  # type: Optional[float]
        maxFreeManipAccel=None,  # type: Optional[float]
        constraintToolInfo=None,  # type: Optional[types.ConstraintToolInfo]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Checks to see if the detection obstacles have all arrived.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 100)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            minDetectionImageTimeMS: (Default: None)
            detectionInfos: (Default: None)
            pickLocationInfo: Describes the pick location and properties about it to initialize the cycle. (Default: None)
            placeLocationInfos: Describes the place locations and properties about them to initialize the cycle. (Default: None)
            packLocationInfo: (Default: None)
            predictDetectionInfo: Describes where to place predicted target before detection results. (Default: None)
            useLocationState: (Default: None)
            sourcecontainername: (Default: None)
            bodyOcclusionInfos: (Default: None)
            forceWaitDestContainer: (Default: None)
            waitUpdateStampTimeout: (Default: None)
            manipname: (Default: None)
            constraintToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            constraintToolOptions: (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            cameraOcclusionOffset: (Default: None)
            robotspeedmult: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            executionFilterFactor: (Default: None)
            executionConnectingTrajDecelMult: (Default: None)
            executionConnectingTrajReverseMult: (Default: None)
            executionReverseRecoveryDistance: (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            destSensorSelectionInfos: (Default: None)
            finalPlanMode: (Default: None)
            pathPlannerParameters: Parameters for robot path planning. (Default: None)
            jittererParameters: Parameters dealing with jittering the robot out of collisions. (Default: None)
            forceTorqueBasedEstimatorParameters: Parameters for state estimation features based on force torque sensor (Default: None)
            smootherParameters: Parameters dealing with getting smoother paths for the robot planning. (Default: None)
            homePositionConfigurationName: (Default: None)
            cycleStartPositionConfigurationName: (Default: None)
            recoveryPositionConfigurationName: (Default: None)
            finalPlanPositionConfigurationName: (Default: None)
            ignoreStartPosition: True if the robot should ignore going to start position (Default: None)
            forceMoveToFinish: If True, then the robot will add a finish position to the cycle even if "finish" is not present, unless "ignoreFinishPosition" is True in the order cycle command. "ignoreFinishPosition" overrides this parameter. (Default: None)
            ignoreFinishPosition: (Default: None)
            ignoreFinishPositionUnlessPackFormationComplete: (Default: None)
            justInTimeToolChangePlanning: (Default: None)
            constraintDuringGrabbingToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            maxGrabbingManipSpeed: (Default: None)
            maxGrabbingManipAccel: (Default: None)
            maxFreeManipSpeed: (Default: None)
            maxFreeManipAccel: (Default: None)
            constraintToolInfo: Constrain a direction on the tool to be within a certain angle with respect to a global direction. (Default: None)
        """
        taskparameters = {
            'command': 'HasDetectionObstacles',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if minDetectionImageTimeMS is not None:
            taskparameters['minDetectionImageTimeMS'] = minDetectionImageTimeMS
        if detectionInfos is not None:
            taskparameters['detectionInfos'] = detectionInfos
        if pickLocationInfo is not None:
            taskparameters['pickLocationInfo'] = pickLocationInfo
        if placeLocationInfos is not None:
            taskparameters['placeLocationInfos'] = placeLocationInfos
        if packLocationInfo is not None:
            taskparameters['packLocationInfo'] = packLocationInfo
        if predictDetectionInfo is not None:
            taskparameters['predictDetectionInfo'] = predictDetectionInfo
        if useLocationState is not None:
            taskparameters['useLocationState'] = useLocationState
        if sourcecontainername is not None:
            taskparameters['sourcecontainername'] = sourcecontainername
        if bodyOcclusionInfos is not None:
            taskparameters['bodyOcclusionInfos'] = bodyOcclusionInfos
        if forceWaitDestContainer is not None:
            taskparameters['forceWaitDestContainer'] = forceWaitDestContainer
        if waitUpdateStampTimeout is not None:
            taskparameters['waitUpdateStampTimeout'] = waitUpdateStampTimeout
        if manipname is not None:
            taskparameters['manipname'] = manipname
        if constraintToolDirection is not None:
            taskparameters['constraintToolDirection'] = constraintToolDirection
        if constraintToolOptions is not None:
            taskparameters['constraintToolOptions'] = constraintToolOptions
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if cameraOcclusionOffset is not None:
            taskparameters['cameraOcclusionOffset'] = cameraOcclusionOffset
        if robotspeedmult is not None:
            taskparameters['robotspeedmult'] = robotspeedmult
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if executionFilterFactor is not None:
            taskparameters['executionFilterFactor'] = executionFilterFactor
        if executionConnectingTrajDecelMult is not None:
            taskparameters['executionConnectingTrajDecelMult'] = executionConnectingTrajDecelMult
        if executionConnectingTrajReverseMult is not None:
            taskparameters['executionConnectingTrajReverseMult'] = executionConnectingTrajReverseMult
        if executionReverseRecoveryDistance is not None:
            taskparameters['executionReverseRecoveryDistance'] = executionReverseRecoveryDistance
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if destSensorSelectionInfos is not None:
            taskparameters['destSensorSelectionInfos'] = destSensorSelectionInfos
        if finalPlanMode is not None:
            taskparameters['finalPlanMode'] = finalPlanMode
        if pathPlannerParameters is not None:
            taskparameters['pathPlannerParameters'] = pathPlannerParameters
        if jittererParameters is not None:
            taskparameters['jittererParameters'] = jittererParameters
        if forceTorqueBasedEstimatorParameters is not None:
            taskparameters['forceTorqueBasedEstimatorParameters'] = forceTorqueBasedEstimatorParameters
        if smootherParameters is not None:
            taskparameters['smootherParameters'] = smootherParameters
        if homePositionConfigurationName is not None:
            taskparameters['homePositionConfigurationName'] = homePositionConfigurationName
        if cycleStartPositionConfigurationName is not None:
            taskparameters['cycleStartPositionConfigurationName'] = cycleStartPositionConfigurationName
        if recoveryPositionConfigurationName is not None:
            taskparameters['recoveryPositionConfigurationName'] = recoveryPositionConfigurationName
        if finalPlanPositionConfigurationName is not None:
            taskparameters['finalPlanPositionConfigurationName'] = finalPlanPositionConfigurationName
        if ignoreStartPosition is not None:
            taskparameters['ignoreStartPosition'] = ignoreStartPosition
        if forceMoveToFinish is not None:
            taskparameters['forceMoveToFinish'] = forceMoveToFinish
        if ignoreFinishPosition is not None:
            taskparameters['ignoreFinishPosition'] = ignoreFinishPosition
        if ignoreFinishPositionUnlessPackFormationComplete is not None:
            taskparameters['ignoreFinishPositionUnlessPackFormationComplete'] = ignoreFinishPositionUnlessPackFormationComplete
        if justInTimeToolChangePlanning is not None:
            taskparameters['justInTimeToolChangePlanning'] = justInTimeToolChangePlanning
        if constraintDuringGrabbingToolDirection is not None:
            taskparameters['constraintDuringGrabbingToolDirection'] = constraintDuringGrabbingToolDirection
        if maxGrabbingManipSpeed is not None:
            taskparameters['maxGrabbingManipSpeed'] = maxGrabbingManipSpeed
        if maxGrabbingManipAccel is not None:
            taskparameters['maxGrabbingManipAccel'] = maxGrabbingManipAccel
        if maxFreeManipSpeed is not None:
            taskparameters['maxFreeManipSpeed'] = maxFreeManipSpeed
        if maxFreeManipAccel is not None:
            taskparameters['maxFreeManipAccel'] = maxFreeManipAccel
        if constraintToolInfo is not None:
            taskparameters['constraintToolInfo'] = constraintToolInfo
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)
    def GetPackingState(self, timeout=10, fireandforget=False, blockwait=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, bool, Optional[types.GetPackingStateParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetPackingStateReturns]
        """
        Get the current packing state.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait: Same as fireandforget, except will be able to receive return later with WaitForCommandResponse. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetPackingState',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)
