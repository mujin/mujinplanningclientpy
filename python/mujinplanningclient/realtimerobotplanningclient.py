# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 Mujin, Inc.
# AUTO GENERATED FILE! DO NOT EDIT!

# system imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from typing import Any, Optional, Union, Literal # noqa: F401 # used in type check
    import realtimerobotplanningclient_types as types

# mujin imports
from . import json
from . import zmq
from . import planningclient

# logging
import logging
log = logging.getLogger(__name__)


class RealtimeRobotPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the RealtimeRobot task"""

    _robotname = None  # type: Optional[str] # Optional name of the robot selected
    _robotspeed = None  # type: Optional[float] # Speed of the robot, e.g. 0.4
    _robotaccelmult = None  # type: Optional[float] # Current robot accel mult
    _envclearance = None  # type: float # type: ignore # Environment clearance in millimeters, e.g. 20
    _robotBridgeConnectionInfo = None  # type: Optional[str] # dict holding the connection info for the robot bridge.

    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(
        self,
        robotname=None,
        robotspeed=None,
        robotaccelmult=None,
        envclearance=10.0,
        robotBridgeConnectionInfo=None,
        taskzmqport=11000,
        taskheartbeatport=11001,
        taskheartbeattimeout=7.0,
        tasktype='realtimerobottask3',
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
        # type: (str, Optional[float], Optional[float], float, Optional[str], int, int, float, str, Optional[zmq.Context], Optional[str], str, str, str, str, str, str, Any) -> None
        """Connects to the Mujin controller, initializes RealtimeRobot task and sets up parameters

        Args:
            robotname (str, optional): Name of the robot, e.g. VP-5243I
            robotspeed (float, optional): Speed of the robot, e.g. 0.4
            robotaccelmult (float, optional): Optional multiplier for the robot acceleration.
            envclearance (float, optional): Environment clearance in millimeter, e.g. 20
            robotBridgeConnectionInfo (str, optional): dict holding the connection info for the robot bridge.
            taskzmqport (int, optional): Port of the task's ZMQ server, e.g. 7110. (Default: 11000)
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher, e.g. 7111. (Default: 11001)
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: realtimerobottask3
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
        self._robotname = robotname
        self._robotspeed = robotspeed
        self._robotaccelmult = robotaccelmult
        self._envclearance = envclearance
        self._robotBridgeConnectionInfo = robotBridgeConnectionInfo
        super(RealtimeRobotPlanningClient, self).__init__(
            
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

    def GetRobotConnectionInfo(self):
        # type: () -> Optional[str]
        """ """
        return self._robotBridgeConnectionInfo

    def SetRobotConnectionInfo(self, robotBridgeConnectionInfo):
        # type: (str) -> None
        """
        Args:
            robotBridgeConnectionInfo:
        """
        self._robotBridgeConnectionInfo = robotBridgeConnectionInfo

    def GetRobotName(self):
        # type: () -> Optional[str]
        """ """
        return self._robotname

    def SetRobotName(self, robotname):
        # type: (str) -> None
        """
        Args:
            robotname (str):
        """
        self._robotname = robotname

    def SetRobotSpeed(self, robotspeed):
        # type: (float) -> None
        """
        Args:
            robotspeed:
        """
        self._robotspeed = robotspeed

    def SetRobotAccelMult(self, robotaccelmult):
        # type: (float) -> None
        """
        Args:
            robotaccelmult:
        """
        self._robotaccelmult = robotaccelmult

    def ExecuteCommand(self, taskparameters, robotname=None, toolname=None, robotspeed=None, robotaccelmult=None, envclearance=None, timeout=10, fireandforget=False, respawnopts=None, forcereload=False, blockwait=True):
        # type: (dict, Optional[str], Optional[str], Optional[float], Optional[float], Optional[float], float, bool, Any, bool) -> Any
        """Wrapper to ExecuteCommand with robot info specified in taskparameters.

        Executes a command in the task.

        Args:
            taskparameters (dict): Specifies the arguments of the task/command being called.
            robotname (str, optional): Name of the robot
            robotaccelmult (float, optional):
            envclearance (float, optional):
            respawnopts (optional):
            toolname (str, optional): Name of the manipulator. Default: self.toolname
            timeout (float, optional):  (Default: 10)
            fireandforget (bool, optional):  (Default: False)
            robotspeed (float, optional):
            forcereload (bool): If True, then force re-load the scene before executing the task.

        Returns:
            dict: Contains:
                - robottype (str): robot type
                - currentjointvalues (list[float]): current joint values, vector length = DOF
                - elapsedtime (float): elapsed time in seconds
                - numpoints (int): the number of points
                - error (dict): optional error info
                - desc (str): error message
                - type (str): error type
                - errorcode (str): error code
        """
        if robotname is None:
            robotname = self._robotname

        # caller wants to use a different tool
        if toolname is not None:
            # set at the first level
            taskparameters['toolname'] = toolname

        if robotname is not None:
            taskparameters['robotname'] = robotname

        if 'robotspeed' not in taskparameters:
            if robotspeed is None:
                robotspeed = self._robotspeed
            if robotspeed is not None:
                taskparameters['robotspeed'] = robotspeed

        if 'robotaccelmult' not in taskparameters:
            if robotaccelmult is None:
                robotaccelmult = self._robotaccelmult
            if robotaccelmult is not None:
                taskparameters['robotaccelmult'] = robotaccelmult

        if self._robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = self._robotBridgeConnectionInfo

        if 'envclearance' not in taskparameters or taskparameters['envclearance'] is None:
            if envclearance is None:
                envclearance = self._envclearance
            if envclearance is not None:
                taskparameters['envclearance'] = envclearance

        return super(RealtimeRobotPlanningClient, self).ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, respawnopts=respawnopts, forcereload=forcereload)

    #
    # Commands (generated from the spec)
    #

    def Ping(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None):
        # type: (float, Optional[types.PingParametersDynamicEnvironmentState], Optional[int]) -> Optional[types.PingReturns]
        """
        Ping the server.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'Ping',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetJointValues(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, executetimeout=10, **kwargs):
        # type: (float, Optional[types.GetJointValuesParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.GetJointValuesParametersRobotBridgeConnectionInfo], Optional[list[types.GetJointValuesParametersLocationCollisionInfosArrayElement]], float, Optional[Any]) -> Optional[types.GetJointValuesReturns]
        """
        Gets the current robot joint values

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            executetimeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'GetJointValues',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if executetimeout != 10:
            taskparameters['executetimeout'] = executetimeout
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def MoveToolLinear(
        self,
        goaltype,  # type: Optional[str]
        goals,  # type: Optional[list[float]]
        toolname=None,  # type: Optional[str]
        timeout=10,  # type: float
        robotspeed=None,  # type: Optional[float]
        workmaxdeviationangle=None,  # type: Optional[float]
        workspeed=None,  # type: Optional[list[float]]
        workaccel=None,  # type: Optional[list[float]]
        worksteplength=None,  # type: Optional[float]
        plannername=None,  # type: Optional[str]
        numspeedcandidates=None,  # type: Optional[int]
        workminimumcompletetime=_deprecated,  # type: Optional[float]
        workminimumcompleteratio=_deprecated,  # type: Optional[float]
        workignorefirstcollisionee=None,  # type: Optional[float]
        workignorelastcollisionee=None,  # type: Optional[float]
        workignorefirstcollision=None,  # type: Optional[float]
        dynamicEnvironmentState=None,  # type: Optional[types.MoveToolLinearParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        robotname=None,  # type: Optional[str]
        robotBridgeConnectionInfo=None,  # type: Optional[types.MoveToolLinearParametersRobotBridgeConnectionInfo]
        locationCollisionInfos=None,  # type: Optional[list[types.MoveToolLinearParametersLocationCollisionInfosArrayElement]]
        speed=_deprecated,  # type: Optional[Any]
        robotaccelmult=None,  # type: Optional[float]
        ionames=None,  # type: Optional[list[Any]]
        ignoreGrabbingTarget=None,  # type: Optional[bool]
        currentlimitratios=None,  # type: Optional[list[float]]
        instobjectname=None,  # type: Optional[str]
        ikparamname=None,  # type: Optional[str]
        execute=None,  # type: Optional[int]
        moveStraightParams=None,  # type: Optional[types.MoveStraightParameters]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Moves the tool linearly in cartesian (3D) space.

        Args:
            goaltype: Type of the goal, e.g. translationdirection5d
            goals: Flat list of goals, e.g. two 5D ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]
            toolname: Name of the manipulator. Default: self.toolname (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            workmaxdeviationangle: How much the tool tip can rotationally deviate from the linear path. In deg. (Default: None)
            workspeed: [anglespeed, transspeed] in deg/s and mm/s (Default: None)
            workaccel: [angleaccel, transaccel] in deg/s^2 and mm/s^2 (Default: None)
            worksteplength: Discretization for planning MoveHandStraight, in seconds. (Default: None)
            plannername: (Default: None)
            numspeedcandidates: If speed/accel are not specified, the number of candiates to consider (Default: None)
            workminimumcompletetime: **deprecated** Unused. Set to trajduration - 0.016s. EMU_MUJIN example requires at least this much (Default: None)
            workminimumcompleteratio: **deprecated** Unused. In case the duration of the trajectory is now known, can specify in terms of [0,1]. 1 is complete everything. (Default: None)
            workignorefirstcollisionee: time, necessary in case initial is in collision, has to be multiples of step length? (Default: None)
            workignorelastcollisionee: time, necessary in case goal is in collision, has to be multiples of step length? (Default: None)
            workignorefirstcollision: (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
            ignoreGrabbingTarget: (Default: None)
            currentlimitratios: The joints' current limit ratios. (Default: None)
            instobjectname: If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position. (Default: None)
            ikparamname: If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position. (Default: None)
            execute: If 1, execute the motion. (Default: None)
            moveStraightParams: A set of parameters defining how the robot behaves during linear motions. (Default: None)
        """
        taskparameters = {
            'command': 'MoveToolLinear',
            'goaltype': goaltype,
            'goals': goals,
        }  # type: dict[str, Any]
        if workmaxdeviationangle is not None:
            taskparameters['workmaxdeviationangle'] = workmaxdeviationangle
        if workspeed is not None:
            taskparameters['workspeed'] = workspeed
        if workaccel is not None:
            taskparameters['workaccel'] = workaccel
        if worksteplength is not None:
            taskparameters['worksteplength'] = worksteplength
        if plannername is not None:
            taskparameters['plannername'] = plannername
        if numspeedcandidates is not None:
            taskparameters['numspeedcandidates'] = numspeedcandidates
        if workignorefirstcollisionee is not None:
            taskparameters['workignorefirstcollisionee'] = workignorefirstcollisionee
        if workignorelastcollisionee is not None:
            taskparameters['workignorelastcollisionee'] = workignorelastcollisionee
        if workignorefirstcollision is not None:
            taskparameters['workignorefirstcollision'] = workignorefirstcollision
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        if ignoreGrabbingTarget is not None:
            taskparameters['ignoreGrabbingTarget'] = ignoreGrabbingTarget
        if currentlimitratios is not None:
            taskparameters['currentlimitratios'] = currentlimitratios
        if instobjectname is not None:
            taskparameters['instobjectname'] = instobjectname
        if ikparamname is not None:
            taskparameters['ikparamname'] = ikparamname
        if execute is not None:
            taskparameters['execute'] = execute
        if moveStraightParams is not None:
            taskparameters['moveStraightParams'] = moveStraightParams
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout, robotspeed=robotspeed)

    def MoveToHandPosition(
        self,
        goaltype,  # type: Optional[str]
        goals,  # type: Optional[list[float]]
        toolname=None,  # type: Optional[str]
        envclearance=None,  # type: Optional[float]
        closegripper=0,  # type: int
        robotspeed=None,  # type: Optional[float]
        robotaccelmult=None,  # type: Optional[float]
        timeout=10,  # type: float
        dynamicEnvironmentState=None,  # type: Optional[types.MoveToHandPositionParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        robotname=None,  # type: Optional[str]
        robotBridgeConnectionInfo=None,  # type: Optional[types.MoveToHandPositionParametersRobotBridgeConnectionInfo]
        locationCollisionInfos=None,  # type: Optional[list[types.MoveToHandPositionParametersLocationCollisionInfosArrayElement]]
        speed=_deprecated,  # type: Optional[Any]
        ionames=None,  # type: Optional[list[Any]]
        constraintToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        departOffsetDir=None,  # type: Optional[tuple[float, float, float]]
        departMinimumCompleteRatio=None,  # type: Optional[float]
        departOffsetAwayFromGravity=None,  # type: Optional[float]
        trajname=None,  # type: Optional[str]
        disablebodies=None,  # type: Optional[bool]
        ignoreGrabbingTarget=None,  # type: Optional[bool]
        jointthresh=None,  # type: Optional[float]
        jitter=None,  # type: Optional[float]
        execute=None,  # type: Optional[int]
        executionFilterFactor=None,  # type: Optional[float]
        filtertraj=None,  # type: Optional[bool]
        currentlimitratios=None,  # type: Optional[list[float]]
        goalJointThreshold=None,  # type: Optional[list[float]]
        goalWorkspaceThreshold=None,  # type: Optional[float]
        calibrategripper=None,  # type: Optional[bool]
        departAccel=None,  # type: Optional[float]
        departOffsetInTool=None,  # type: Optional[bool]
        departSpeed=None,  # type: Optional[float]
        maxManipAccel=None,  # type: Optional[float]
        maxJitterLinkDist=None,  # type: Optional[float]
        pathPlannerParameters=None,  # type: Optional[types.PathPlannerParameters]
        moveStraightParams=None,  # type: Optional[types.MoveStraightParameters]
        forceTorqueBasedEstimatorParameters=None,  # type: Optional[types.ForceTorqueBasedEstimatorParameters]
        savetrajectorylog=None,  # type: Optional[bool]
        saveRobotFeedbackLog=None,  # type: Optional[bool]
        loadRobotFeedbackLog=None,  # type: Optional[bool]
        saveConcatenateTrajectoryLog=None,  # type: Optional[bool]
        saveFilterTrajectoryLog=None,  # type: Optional[bool]
        executionConnectingTrajReverseMult=None,  # type: Optional[float]
        executionReverseRecoveryDistance=None,  # type: Optional[float]
        jittererParameters=None,  # type: Optional[types.JittererParameters]
        gripperInfo=None,  # type: Optional[types.MoveToHandPositionParametersGripperInfoVariantItemPrefix0]
        positionConfigurationName=None,  # type: Optional[str]
        positionConfigurationCandidateNames=None,  # type: Optional[list[str]]
        robotspeedmult=None,  # type: Optional[float]
        robotJointNames=None,  # type: Optional[list[str]]
        jointindices=_deprecated,  # type: Optional[list[int]]
        startvalues=None,  # type: Optional[list[float]]
        startJointConfigurationStates=None,  # type: Optional[list[types.MoveToHandPositionParametersStartJointConfigurationStatesArrayElement]]
        goalJointConfigurationStates=None,  # type: Optional[list[types.MoveToHandPositionParametersGoalJointConfigurationStatesArrayElement]]
        goaljoints=None,  # type: Optional[list[float]]
        minimumgoalpaths=None,  # type: Optional[int]
        chuckgripper=None,  # type: Optional[bool]
        instobjectname=None,  # type: Optional[str]
        ikparamname=None,  # type: Optional[str]
        ikparamoffset=None,  # type: Optional[list[float]]
        smootherParameters=None,  # type: Optional[types.SmoothingParameters]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Computes the inverse kinematics and moves the manipulator to any one of the goals specified.

        Args:
            goaltype: Type of the goal, e.g. translationdirection5d
            goals: Flat list of goals, e.g. two 5D ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            closegripper: Whether to close gripper once the goal is reached. Boolean value represented by 0 or 1. (Default: 0)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
            constraintToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            departOffsetDir: Direction in which to apply the offset when departing from the pick/place operation. (Default: None)
            departMinimumCompleteRatio: The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0. (Default: None)
            departOffsetAwayFromGravity: The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir. (Default: None)
            trajname: (Default: None)
            disablebodies: (Default: None)
            ignoreGrabbingTarget: (Default: None)
            jointthresh: (Default: None)
            jitter: (Default: None)
            execute: If 1, execute the motion. (Default: None)
            executionFilterFactor: (Default: None)
            filtertraj: (Default: None)
            currentlimitratios: The joints' current limit ratios. (Default: None)
            goalJointThreshold: Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time. (Default: None)
            goalWorkspaceThreshold: Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful. (Default: None)
            calibrategripper: (Default: None)
            departAccel: (Default: None)
            departOffsetInTool: (Default: None)
            departSpeed: (Default: None)
            maxManipAccel: (Default: None)
            maxJitterLinkDist: mm, When jittering, the max distance any link on the robot can move. (Default: None)
            pathPlannerParameters: Parameters for robot path planning. (Default: None)
            moveStraightParams: A set of parameters defining how the robot behaves during linear motions. (Default: None)
            forceTorqueBasedEstimatorParameters: Parameters for state estimation features based on force torque sensor (Default: None)
            savetrajectorylog: If True, will save the commanded (input) trajectories before they are executed (Default: None)
            saveRobotFeedbackLog: If True, will tell robotbridge to save trajectory files (Default: None)
            loadRobotFeedbackLog: If True, will tell robotbridge to load the robot feedback log after trajectory ends (Default: None)
            saveConcatenateTrajectoryLog: If True, will save trajectories used for inputs of concatenate trajectory functions (Default: None)
            saveFilterTrajectoryLog: If True, will save trajectories used for filtering, such as SmartFilter (Default: None)
            executionConnectingTrajReverseMult: Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally. (Default: None)
            executionReverseRecoveryDistance: Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm). (Default: None)
            jittererParameters: Parameters dealing with jittering the robot out of collisions. (Default: None)
            gripperInfo: TODO(felixvd): Check if this really propagates. (Default: None)
            positionConfigurationName: If specified, the name of position configuration to move to. If it does not exist, will raise an error. (Default: None)
            positionConfigurationCandidateNames: If specified, goes to the first position that is defined for the robot. If no positions exist, returns without moving the robot. (Default: None)
            robotspeedmult: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotJointNames: (Default: None)
            jointindices: **deprecated** List of corresponding joint indices, default is range(len(jointvalues)) (Default: None)
            startvalues: The robot joint values to start the motion from. (Default: None)
            startJointConfigurationStates: List of dicts for each joint. (Default: None)
            goalJointConfigurationStates: List of dicts for each joint entry. (Default: None)
            goaljoints: List of joint values to move to. (Default: None)
            minimumgoalpaths: Number of solutions the planner must provide before it is allowed to finish. (Default: None)
            chuckgripper: (Default: None)
            instobjectname: If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position. (Default: None)
            ikparamname: If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position. (Default: None)
            ikparamoffset: (Default: None)
            smootherParameters: Parameters dealing with getting smoother paths for the robot planning. (Default: None)
        """
        taskparameters = {
            'command': 'MoveToHandPosition',
            'goaltype': goaltype,
            'goals': goals,
            'closegripper': closegripper,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if ionames is not None:
            taskparameters['ionames'] = ionames
        if constraintToolDirection is not None:
            taskparameters['constraintToolDirection'] = constraintToolDirection
        if departOffsetDir is not None:
            taskparameters['departOffsetDir'] = departOffsetDir
        if departMinimumCompleteRatio is not None:
            taskparameters['departMinimumCompleteRatio'] = departMinimumCompleteRatio
        if departOffsetAwayFromGravity is not None:
            taskparameters['departOffsetAwayFromGravity'] = departOffsetAwayFromGravity
        if trajname is not None:
            taskparameters['trajname'] = trajname
        if disablebodies is not None:
            taskparameters['disablebodies'] = disablebodies
        if ignoreGrabbingTarget is not None:
            taskparameters['ignoreGrabbingTarget'] = ignoreGrabbingTarget
        if jointthresh is not None:
            taskparameters['jointthresh'] = jointthresh
        if jitter is not None:
            taskparameters['jitter'] = jitter
        if execute is not None:
            taskparameters['execute'] = execute
        if executionFilterFactor is not None:
            taskparameters['executionFilterFactor'] = executionFilterFactor
        if filtertraj is not None:
            taskparameters['filtertraj'] = filtertraj
        if currentlimitratios is not None:
            taskparameters['currentlimitratios'] = currentlimitratios
        if goalJointThreshold is not None:
            taskparameters['goalJointThreshold'] = goalJointThreshold
        if goalWorkspaceThreshold is not None:
            taskparameters['goalWorkspaceThreshold'] = goalWorkspaceThreshold
        if calibrategripper is not None:
            taskparameters['calibrategripper'] = calibrategripper
        if departAccel is not None:
            taskparameters['departAccel'] = departAccel
        if departOffsetInTool is not None:
            taskparameters['departOffsetInTool'] = departOffsetInTool
        if departSpeed is not None:
            taskparameters['departSpeed'] = departSpeed
        if maxManipAccel is not None:
            taskparameters['maxManipAccel'] = maxManipAccel
        if maxJitterLinkDist is not None:
            taskparameters['maxJitterLinkDist'] = maxJitterLinkDist
        if pathPlannerParameters is not None:
            taskparameters['pathPlannerParameters'] = pathPlannerParameters
        if moveStraightParams is not None:
            taskparameters['moveStraightParams'] = moveStraightParams
        if forceTorqueBasedEstimatorParameters is not None:
            taskparameters['forceTorqueBasedEstimatorParameters'] = forceTorqueBasedEstimatorParameters
        if savetrajectorylog is not None:
            taskparameters['savetrajectorylog'] = savetrajectorylog
        if saveRobotFeedbackLog is not None:
            taskparameters['saveRobotFeedbackLog'] = saveRobotFeedbackLog
        if loadRobotFeedbackLog is not None:
            taskparameters['loadRobotFeedbackLog'] = loadRobotFeedbackLog
        if saveConcatenateTrajectoryLog is not None:
            taskparameters['saveConcatenateTrajectoryLog'] = saveConcatenateTrajectoryLog
        if saveFilterTrajectoryLog is not None:
            taskparameters['saveFilterTrajectoryLog'] = saveFilterTrajectoryLog
        if executionConnectingTrajReverseMult is not None:
            taskparameters['executionConnectingTrajReverseMult'] = executionConnectingTrajReverseMult
        if executionReverseRecoveryDistance is not None:
            taskparameters['executionReverseRecoveryDistance'] = executionReverseRecoveryDistance
        if jittererParameters is not None:
            taskparameters['jittererParameters'] = jittererParameters
        if gripperInfo is not None:
            taskparameters['gripperInfo'] = gripperInfo
        if positionConfigurationName is not None:
            taskparameters['positionConfigurationName'] = positionConfigurationName
        if positionConfigurationCandidateNames is not None:
            taskparameters['positionConfigurationCandidateNames'] = positionConfigurationCandidateNames
        if robotspeedmult is not None:
            taskparameters['robotspeedmult'] = robotspeedmult
        if robotJointNames is not None:
            taskparameters['robotJointNames'] = robotJointNames
        if startvalues is not None:
            taskparameters['startvalues'] = startvalues
        if startJointConfigurationStates is not None:
            taskparameters['startJointConfigurationStates'] = startJointConfigurationStates
        if goalJointConfigurationStates is not None:
            taskparameters['goalJointConfigurationStates'] = goalJointConfigurationStates
        if goaljoints is not None:
            taskparameters['goaljoints'] = goaljoints
        if minimumgoalpaths is not None:
            taskparameters['minimumgoalpaths'] = minimumgoalpaths
        if chuckgripper is not None:
            taskparameters['chuckgripper'] = chuckgripper
        if instobjectname is not None:
            taskparameters['instobjectname'] = instobjectname
        if ikparamname is not None:
            taskparameters['ikparamname'] = ikparamname
        if ikparamoffset is not None:
            taskparameters['ikparamoffset'] = ikparamoffset
        if smootherParameters is not None:
            taskparameters['smootherParameters'] = smootherParameters
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, envclearance=envclearance, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def Grab(self, targetname, toolname=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, Optional[str], float, Optional[types.GrabParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Grabs an object with tool

        Args:
            targetname: Name of the target object
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'Grab',
            'targetname': targetname,
        }  # type: dict[str, Any]
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def Release(self, targetname, timeout=10, dynamicEnvironmentState=None, debuglevel=None, toolname=None, **kwargs):
        # type: (str, float, Optional[types.ReleaseParametersDynamicEnvironmentState], Optional[int], Optional[str], Optional[Any]) -> Optional[Any]
        """
        Releases a grabbed object.

        Args:
            targetname: Name of the target object
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
        """
        taskparameters = {
            'command': 'Release',
            'targetname': targetname,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if toolname is not None:
            taskparameters['toolname'] = toolname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetGrabbed(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[types.GetGrabbedParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetGrabbedReturns]
        """
        Gets the names of the objects currently grabbed

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetGrabbed',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetTransform(self, targetname, connectedBodyName='', linkName='', geometryName='', geometryPk='', unit='mm', timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, str, str, str, str, str, float, Optional[types.GetTransformParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetTransformReturns]
        """
        Gets the transform of an object

        Args:
            targetname: OpenRave Kinbody name
            connectedBodyName: OpenRave connected body name (Default: '')
            linkName: OpenRave link name (Default: '')
            geometryName: OpenRave geometry id name (Default: '')
            geometryPk: OpenRave geometry primary key (pk) (Default: '')
            unit: The unit of the given values. (Default: 'mm')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            Transform of the object.
        """
        taskparameters = {
            'command': 'GetTransform',
            'targetname': targetname,
        }  # type: dict[str, Any]
        if connectedBodyName is not None:
            taskparameters['connectedBodyName'] = connectedBodyName
        if linkName is not None:
            taskparameters['linkName'] = linkName
        if geometryName is not None:
            taskparameters['geometryName'] = geometryName
        if geometryPk is not None:
            taskparameters['geometryPk'] = geometryPk
        if unit != 'mm':
            taskparameters['unit'] = unit
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetLinkParentInfo(self, objectName, linkName, unit='mm', timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, str, str, float, Optional[types.GetLinkParentInfoParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetLinkParentInfoReturns]
        """
        Gets the parent link transform and name.

        Args:
            objectName: OpenRave Kinbody name.
            linkName: OpenRave link name.
            unit: The unit of the given values. (Default: 'mm')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetLinkParentInfo',
            'objectName': objectName,
            'linkName': linkName,
        }  # type: dict[str, Any]
        if unit != 'mm':
            taskparameters['unit'] = unit
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetTransform(self, targetname, translation, unit='mm', rotationmat=None, quaternion=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, list[float], str, Optional[list[float]], Optional[list[float]], float, Optional[types.SetTransformParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Sets the transform of an object. Rotation can be specified by either quaternion or rotation matrix.

        Args:
            targetname: Name of the target object
            translation: List of x,y,z values of the object in millimeters.
            unit: The unit of the given values. (Default: 'mm')
            rotationmat: List specifying the rotation matrix in row major format, e.g. [1,0,0,0,1,0,0,0,1] (Default: None)
            quaternion: List specifying the quaternion in w,x,y,z format, e.g. [1,0,0,0]. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetTransform',
            'targetname': targetname,
            'translation': translation,
        }  # type: dict[str, Any]
        if unit != 'mm':
            taskparameters['unit'] = unit
        if rotationmat is not None:
            taskparameters['rotationmat'] = rotationmat
        if quaternion is not None:
            taskparameters['quaternion'] = quaternion
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        if rotationmat is None and quaternion is None:
            taskparameters['quaternion'] = [1, 0, 0, 0]
            log.warn('No rotation is specified. Using identity quaternion.')
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetOBB(self, targetname, unit='mm', timeout=10, linkname=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, str, float, Optional[str], Optional[types.GetOBBParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetOBBReturns]
        """
        Get the oriented bounding box (OBB) of object.

        Args:
            targetname: Name of the object
            unit: The unit of the given values. (Default: 'mm')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            linkname: Name of link to use for OBB. If not specified, uses entire target. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            A dictionary describing the OBB of the object.
        """
        taskparameters = {
            'command': 'GetOBB',
            'targetname': targetname,
        }  # type: dict[str, Any]
        if unit != 'mm':
            taskparameters['unit'] = unit
        if linkname is not None:
            taskparameters['linkname'] = linkname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetInnerEmptyRegionOBB(self, targetname, linkname=None, unit='mm', timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, Optional[str], str, float, Optional[types.GetInnerEmptyRegionOBBParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetInnerEmptyRegionOBBReturns]
        """
        Get the inner empty oriented bounding box (OBB) of a container.

        Args:
            targetname: Name of the object
            linkname: Name of link to use for OBB. If not specified, uses entire target. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            A dictionary describing the OBB of the object.
        """
        taskparameters = {
            'command': 'GetInnerEmptyRegionOBB',
            'targetname': targetname,
        }  # type: dict[str, Any]
        if linkname is not None:
            taskparameters['linkname'] = linkname
        if unit != 'mm':
            taskparameters['unit'] = unit
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetInstObjectAndSensorInfo(self, instobjectnames=None, sensornames=None, unit='mm', timeout=10, ignoreMissingObjects=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (Optional[list[str]], Optional[list[str]], str, float, Optional[bool], Optional[types.GetInstObjectAndSensorInfoParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Returns information about the inst objects and sensors that are a part of those inst objects.

        Args:
            instobjectnames: (Default: None)
            sensornames: (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            ignoreMissingObjects: If False, will raise an error if the object is not found in the scene. Default: True. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetInstObjectAndSensorInfo',
        }  # type: dict[str, Any]
        if instobjectnames is not None:
            taskparameters['instobjectnames'] = instobjectnames
        if sensornames is not None:
            taskparameters['sensornames'] = sensornames
        if unit != 'mm':
            taskparameters['unit'] = unit
        if ignoreMissingObjects is not None:
            taskparameters['ignoreMissingObjects'] = ignoreMissingObjects
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetInstObjectInfoFromURI(self, objecturi=None, unit='mm', timeout=10, instobjectpose=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (Optional[str], str, float, Optional[tuple[float, float, float, float, float, float, float]], Optional[types.GetInstObjectInfoFromURIParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetInstObjectInfoFromURIReturns]
        """
        Opens a URI and returns info about the internal/external and geometry info from it.

        Args:
            objecturi: (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            instobjectpose: If set, updates the inst object to have this pose (in global coordinates) and then computes the geometry data based on it. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetInstObjectInfoFromURI',
        }  # type: dict[str, Any]
        if objecturi is not None:
            taskparameters['objecturi'] = objecturi
        if unit != 'mm':
            taskparameters['unit'] = unit
        if instobjectpose is not None:
            taskparameters['instobjectpose'] = instobjectpose
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetAABB(self, targetname, unit='mm', timeout=10, linkname=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, str, float, Optional[str], Optional[types.GetAABBParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetAABBReturns]
        """
        Gets the axis-aligned bounding box (AABB) of an object.

        Args:
            targetname: Name of the object
            unit: The unit of the given values. (Default: 'mm')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            linkname: Name of link to use for the AABB. If not specified, uses entire target. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            AABB of the object.
        """
        taskparameters = {
            'command': 'GetAABB',
            'targetname': targetname,
        }  # type: dict[str, Any]
        if unit != 'mm':
            taskparameters['unit'] = unit
        if linkname is not None:
            taskparameters['linkname'] = linkname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetLocationTracking(self, timeout=10, fireandforget=False, cycleIndex=None, locationReplaceInfos=None, removeLocationNames=None, minRobotBridgeTimeStampUS=None, dynamicObstacleBaseName=None, targetUpdateBaseName=None, ioSignalsInfo=None, unit='mm', dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[Any], Optional[Any], Optional[list[str]], Optional[int], Optional[str], Optional[str], Optional[types.SetLocationTrackingParametersIoSignalsInfoVariantItemPrefix0], str, Optional[types.SetLocationTrackingParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Resets the tracking of specific containers

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            cycleIndex: The cycle index to track the locations for (Default: None)
            locationReplaceInfos: A dict that should have the keys: name, containerDynamicProperties, rejectContainerIds, uri, pose, cycleIndex (Default: None)
            removeLocationNames: (Default: None)
            minRobotBridgeTimeStampUS: The minimum expected time stamp. (Default: None)
            dynamicObstacleBaseName: (Default: None)
            targetUpdateBaseName: (Default: None)
            ioSignalsInfo: Struct for dictating if any IO signals should be written on receiving detection results (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetLocationTracking',
        }  # type: dict[str, Any]
        if cycleIndex is not None:
            taskparameters['cycleIndex'] = cycleIndex
        if locationReplaceInfos is not None:
            taskparameters['locationReplaceInfos'] = locationReplaceInfos
        if removeLocationNames is not None:
            taskparameters['removeLocationNames'] = removeLocationNames
        if minRobotBridgeTimeStampUS is not None:
            taskparameters['minRobotBridgeTimeStampUS'] = minRobotBridgeTimeStampUS
        if dynamicObstacleBaseName is not None:
            taskparameters['dynamicObstacleBaseName'] = dynamicObstacleBaseName
        if targetUpdateBaseName is not None:
            taskparameters['targetUpdateBaseName'] = targetUpdateBaseName
        if ioSignalsInfo is not None:
            taskparameters['ioSignalsInfo'] = ioSignalsInfo
        if unit != 'mm':
            taskparameters['unit'] = unit
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResetLocationTracking(self, timeout=10, fireandforget=False, resetAllLocations=None, resetLocationName=None, resetLocationNames=None, checkIdAndResetLocationName=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[bool], Optional[str], Optional[list[str]], Optional[Any], Optional[types.ResetLocationTrackingParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[list[str]]
        """
        Resets tracking updates for locations

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            resetAllLocations: If True, then will reset all the locations (Default: None)
            resetLocationName: Resets only the location with matching name (Default: None)
            resetLocationNames: Resets only locations with matching name (Default: None)
            checkIdAndResetLocationName: (locationName, containerId) - only reset the location if the container id matches (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            clearedLocationNames
        """
        taskparameters = {
            'command': 'ResetLocationTracking',
        }  # type: dict[str, Any]
        if resetAllLocations is not None:
            taskparameters['resetAllLocations'] = resetAllLocations
        if resetLocationName is not None:
            taskparameters['resetLocationName'] = resetLocationName
        if resetLocationNames is not None:
            taskparameters['resetLocationNames'] = resetLocationNames
        if checkIdAndResetLocationName is not None:
            taskparameters['checkIdAndResetLocationName'] = checkIdAndResetLocationName
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)['clearedLocationNames']

    def GetLocationTrackingInfos(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[types.GetLocationTrackingInfosParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetLocationTrackingInfosReturns]
        """
        Gets the active tracked locations

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            activeLocationTrackingInfos
        """
        taskparameters = {
            'command': 'GetLocationTrackingInfos',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)['activeLocationTrackingInfos']

    def UpdateLocationContainerIdType(self, locationName, containerName, containerId, containerType, trackingCycleIndex=None, timeout=10, fireandforget=False, unit='mm', dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, str, Optional[str], str, Optional[str], float, bool, str, Optional[types.UpdateLocationContainerIdTypeParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Resets the tracking of specific containers

        Args:
            locationName: Name of the location the container is in
            containerName: Name of the container
            containerId: ID of the container
            containerType: Type of the container
            trackingCycleIndex: If specified, then the cycle with same cycleIndex will update location tracking in the same call. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            unit: The unit of the given values. (Default: 'mm')
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'UpdateLocationContainerIdType',
            'locationName': locationName,
            'containerName': containerName,
            'containerId': containerId,
            'containerType': containerType,
        }  # type: dict[str, Any]
        if trackingCycleIndex is not None:
            taskparameters['trackingCycleIndex'] = trackingCycleIndex
        if unit != 'mm':
            taskparameters['unit'] = unit
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResetLocationTrackingContainerId(self, locationName, checkContainerId, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, str, float, bool, Optional[types.ResetLocationTrackingContainerIdParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Resets the containerId of self._activeLocationTrackingInfos if it matches checkContainerId.

        Args:
            locationName: The name of the location that may be reset.
            checkContainerId: If checkContainerId is specified and not empty and it matches the current containerId of the tracking location, then reset the current tracking location
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ResetLocationTrackingContainerId',
            'locationName': locationName,
            'checkContainerId': checkContainerId,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def RemoveObjectsWithPrefix(self, prefix=_deprecated, removeNamePrefixes=None, timeout=10, fireandforget=False, removeLocationNames=None, doRemoveOnlyDynamic=None, dynamicEnvironmentState=None, debuglevel=None, locationName=None, locationContainerId=None, imageStartTimeStampMS=None, callerid=None, robotname=None, envclearance=None, robotBridgeConnectionInfo=None, robotaccelmult=None, robotspeed=None, stamp=None, command=None, **kwargs):
        # type: (Optional[str], Optional[list[str]], float, bool, Optional[list[str]], Optional[bool], Optional[types.RemoveObjectsWithPrefixParametersDynamicEnvironmentState], Optional[int], Optional[str], Optional[Any], Optional[int], Optional[str], Optional[str], Optional[float], Optional[types.RemoveObjectsWithPrefixParametersRobotBridgeConnectionInfo], Optional[float], Optional[float], Optional[float], Optional[str], Optional[Any]) -> Optional[types.RemoveObjectsWithPrefixReturns]
        """
        Removes objects with prefix.

        Args:
            prefix: **deprecated** (Default: None)
            removeNamePrefixes: Names of prefixes to match with when removing items (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            removeLocationNames: (Default: None)
            doRemoveOnlyDynamic: If True, removes objects that were added through dynamic means such as UpdateObjects/UpdateEnvironmentState. Default: False (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            locationName: Name of the location to update. (Default: None)
            locationContainerId: (Default: None)
            imageStartTimeStampMS: (Default: None)
            callerid: The name of the caller (only used internally) (Default: None)
            robotname: Name of the robot (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            stamp: (Default: None)
            command: (Default: None)
        """
        taskparameters = {
            'command': 'RemoveObjectsWithPrefix',
        }  # type: dict[str, Any]
        if removeNamePrefixes is not None:
            taskparameters['removeNamePrefixes'] = removeNamePrefixes
        if removeLocationNames is not None:
            taskparameters['removeLocationNames'] = removeLocationNames
        if doRemoveOnlyDynamic is not None:
            taskparameters['doRemoveOnlyDynamic'] = doRemoveOnlyDynamic
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if locationName is not None:
            taskparameters['locationName'] = locationName
        if locationContainerId is not None:
            taskparameters['locationContainerId'] = locationContainerId
        if imageStartTimeStampMS is not None:
            taskparameters['imageStartTimeStampMS'] = imageStartTimeStampMS
        if callerid is not None:
            taskparameters['callerid'] = callerid
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if stamp is not None:
            taskparameters['stamp'] = stamp
        if command is not None:
            taskparameters['command'] = command
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetTrajectoryLog(self, timeout=10, startindex=None, num=None, includejointvalues=False, dynamicEnvironmentState=None, debuglevel=None, saverawtrajectories=None, **kwargs):
        # type: (float, Optional[int], Optional[int], bool, Optional[types.GetTrajectoryLogParametersDynamicEnvironmentState], Optional[int], Optional[bool], Optional[Any]) -> Optional[types.GetTrajectoryLogReturns]
        """
        Gets the recent trajectories executed on the binpicking server. The internal server keeps trajectories around for 10 minutes before clearing them.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            startindex: Start of the trajectory to get. If negative, will start counting from the end. For example, -1 is the last element, -2 is the second to last. Default: 0 (Default: None)
            num: Number of trajectories from startindex to return. If 0, will return all the trajectories starting from startindex. Default: 0 (Default: None)
            includejointvalues: If True, will include timedjointvalues. If False, will just give back the trajectories. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            saverawtrajectories: If True, will save the raw trajectories. (Default: None)
        """
        taskparameters = {
            'command': 'GetTrajectoryLog',
        }  # type: dict[str, Any]
        if startindex is not None:
            taskparameters['startindex'] = startindex
        if num is not None:
            taskparameters['num'] = num
        if includejointvalues != False:
            taskparameters['includejointvalues'] = includejointvalues
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if saverawtrajectories is not None:
            taskparameters['saverawtrajectories'] = saverawtrajectories
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ChuckGripper(self, robotname=None, grippername=None, timeout=10, toolname=None, dynamicEnvironmentState=None, debuglevel=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (Optional[str], str, float, Optional[str], Optional[types.ChuckGripperParametersDynamicEnvironmentState], Optional[int], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Chucks the manipulator

        Args:
            robotname: Name of the robot (Default: None)
            grippername: Name of the gripper. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'ChuckGripper',
            'grippername': grippername,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def UnchuckGripper(self, robotname=None, grippername=None, timeout=10, targetname=None, toolname=None, pulloutdist=None, deletetarget=None, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, releaseTargetData=None, useReleaseTargetData=None, **kwargs):
        # type: (Optional[str], str, float, Optional[str], Optional[str], Optional[float], Optional[int], Optional[types.UnchuckGripperParametersDynamicEnvironmentState], Optional[int], str, Optional[types.UnchuckGripperParametersRobotBridgeConnectionInfo], Optional[list[types.UnchuckGripperParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any], Optional[bool], Optional[Any]) -> Optional[Any]
        """
        Unchucks the manipulator and releases the target

        Args:
            robotname: Name of the robot (Default: None)
            grippername: Name of the gripper. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            targetname: Name of the target object. (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            pulloutdist: Distance to move away along the tool direction after releasing. (Default: None)
            deletetarget: If 1, removes the target object from the environment after releasing. (Default: 1) (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
            releaseTargetData: Keys representing what to release and where to release it to. (Default: None)
            useReleaseTargetData: (Default: None)
        """
        taskparameters = {
            'command': 'UnchuckGripper',
            'grippername': grippername,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if targetname is not None:
            taskparameters['targetname'] = targetname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if pulloutdist is not None:
            taskparameters['pulloutdist'] = pulloutdist
        if deletetarget is not None:
            taskparameters['deletetarget'] = deletetarget
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        if releaseTargetData is not None:
            taskparameters['releaseTargetData'] = releaseTargetData
        if useReleaseTargetData is not None:
            taskparameters['useReleaseTargetData'] = useReleaseTargetData
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def CalibrateGripper(self, robotname=None, grippername=None, timeout=10, fireandforget=False, toolname=None, dynamicEnvironmentState=None, debuglevel=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (Optional[str], str, float, bool, Optional[str], Optional[types.CalibrateGripperParametersDynamicEnvironmentState], Optional[int], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Goes through the gripper calibration procedure

        Args:
            robotname: Name of the robot (Default: None)
            grippername: Name of the gripper. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'CalibrateGripper',
            'grippername': grippername,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def StopGripper(self, robotname=None, grippername=None, timeout=10, fireandforget=False, toolname=None, dynamicEnvironmentState=None, debuglevel=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (Optional[str], str, float, bool, Optional[str], Optional[types.StopGripperParametersDynamicEnvironmentState], Optional[int], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Args:
            robotname: Name of the robot (Default: None)
            grippername: Name of the gripper. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'StopGripper',
            'grippername': grippername,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def MoveGripper(self, grippervalues, robotname=None, grippername=None, timeout=10, fireandforget=False, toolname=None, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (list[float], Optional[str], str, float, bool, Optional[str], Optional[types.MoveGripperParametersDynamicEnvironmentState], Optional[int], str, Optional[types.MoveGripperParametersRobotBridgeConnectionInfo], Optional[list[types.MoveGripperParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Moves the chuck of the manipulator to a given value.

        Args:
            grippervalues: Target value(s) of the chuck.
            robotname: Name of the robot (Default: None)
            grippername: Name of the gripper. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'MoveGripper',
            'grippervalues': grippervalues,
            'grippername': grippername,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ExecuteRobotProgram(self, robotProgramName, robotname=None, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, unit='mm', toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (str, Optional[str], float, bool, Optional[types.ExecuteRobotProgramParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[types.ExecuteRobotProgramParametersRobotBridgeConnectionInfo], Optional[list[types.ExecuteRobotProgramParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Execute a robot specific program by name

        Args:
            robotProgramName:
            robotname: Name of the robot (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'ExecuteRobotProgram',
            'robotProgramName': robotProgramName,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SaveScene(self, timeout=10, filename=None, preserveexternalrefs=None, externalref=None, saveclone=_deprecated, saveReferenceUriAsHint=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[str], Optional[bool], Optional[str], Optional[Any], Optional[bool], Optional[types.SaveSceneParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.SaveSceneReturns]
        """
        Saves the current scene to file

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            filename: e.g. /tmp/testscene.mujin.dae, if not specified, it will be saved with an auto-generated filename (Default: None)
            preserveexternalrefs: If True, any bodies that are currently being externally referenced from the environment will be saved as external references. (Default: None)
            externalref: If '*', then each of the objects will be saved as externally referencing their original filename. Otherwise will force saving specific bodies as external references. (Default: None)
            saveclone: **deprecated** If 1, will save the scenes for all the cloned environments (Default: None)
            saveReferenceUriAsHint: If True, use save the reference uris as referenceUriHint so that webstack does not get confused and deletes content (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            The filename the scene is saved to, in a json dictionary, e.g. {'filename': '2013-11-01-17-10-00-UTC.dae'}
        """
        taskparameters = {
            'command': 'SaveScene',
        }  # type: dict[str, Any]
        if filename is not None:
            taskparameters['filename'] = filename
        if preserveexternalrefs is not None:
            taskparameters['preserveexternalrefs'] = preserveexternalrefs
        if externalref is not None:
            taskparameters['externalref'] = externalref
        if saveReferenceUriAsHint is not None:
            taskparameters['saveReferenceUriAsHint'] = saveReferenceUriAsHint
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SaveGripper(self, timeout=10, robotname=None, filename=None, manipname=None, dynamicEnvironmentState=None, debuglevel=None, unit='mm', toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, **kwargs):
        # type: (float, Optional[str], Optional[str], Optional[str], Optional[types.SaveGripperParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[types.SaveGripperParametersRobotBridgeConnectionInfo], Optional[list[types.SaveGripperParametersLocationCollisionInfosArrayElement]], Optional[Any]) -> Optional[Any]
        """
        Separate gripper from a robot in a scene and save it.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            robotname: Name of the robot waiting to extract the hand from. (Default: None)
            filename: File name to save on the file system. e.g. /tmp/robotgripper/mujin.dae (Default: None)
            manipname: Name of the manipulator. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
        """
        taskparameters = {
            'command': 'SaveGripper',
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if filename is not None:
            taskparameters['filename'] = filename
        if manipname is not None:
            taskparameters['manipname'] = manipname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def MoveJointsToJointConfigurationStates(
        self,
        goalJointConfigurationStates,  # type: list[types.MoveJointsToJointConfigurationStatesParametersGoalJointConfigurationStatesArrayElement]
        robotname=None,  # type: Optional[str]
        robotspeed=None,  # type: Optional[float]
        robotaccelmult=None,  # type: Optional[float]
        execute=1,  # type: int
        startJointConfigurationStates=None,  # type: Optional[list[types.MoveJointsToJointConfigurationStatesParametersStartJointConfigurationStatesArrayElement]]
        envclearance=None,  # type: Optional[float]
        timeout=10,  # type: float
        jointStates=None,  # type: Optional[list[Any]]
        jointindices=_deprecated,  # type: Optional[list[int]]
        dynamicEnvironmentState=None,  # type: Optional[types.MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        toolname=None,  # type: Optional[str]
        robotBridgeConnectionInfo=None,  # type: Optional[types.MoveJointsToJointConfigurationStatesParametersRobotBridgeConnectionInfo]
        locationCollisionInfos=None,  # type: Optional[list[types.MoveJointsToJointConfigurationStatesParametersLocationCollisionInfosArrayElement]]
        speed=_deprecated,  # type: Optional[Any]
        ionames=None,  # type: Optional[list[Any]]
        constraintToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        departOffsetDir=None,  # type: Optional[tuple[float, float, float]]
        departMinimumCompleteRatio=None,  # type: Optional[float]
        departOffsetAwayFromGravity=None,  # type: Optional[float]
        trajname=None,  # type: Optional[str]
        disablebodies=None,  # type: Optional[bool]
        ignoreGrabbingTarget=None,  # type: Optional[bool]
        jointthresh=None,  # type: Optional[float]
        jitter=None,  # type: Optional[float]
        executionFilterFactor=None,  # type: Optional[float]
        filtertraj=None,  # type: Optional[bool]
        currentlimitratios=None,  # type: Optional[list[float]]
        goalJointThreshold=None,  # type: Optional[list[float]]
        goalWorkspaceThreshold=None,  # type: Optional[float]
        calibrategripper=None,  # type: Optional[bool]
        departAccel=None,  # type: Optional[float]
        departOffsetInTool=None,  # type: Optional[bool]
        departSpeed=None,  # type: Optional[float]
        maxManipAccel=None,  # type: Optional[float]
        maxJitterLinkDist=None,  # type: Optional[float]
        pathPlannerParameters=None,  # type: Optional[types.PathPlannerParameters]
        moveStraightParams=None,  # type: Optional[types.MoveStraightParameters]
        forceTorqueBasedEstimatorParameters=None,  # type: Optional[types.ForceTorqueBasedEstimatorParameters]
        savetrajectorylog=None,  # type: Optional[bool]
        saveRobotFeedbackLog=None,  # type: Optional[bool]
        loadRobotFeedbackLog=None,  # type: Optional[bool]
        saveConcatenateTrajectoryLog=None,  # type: Optional[bool]
        saveFilterTrajectoryLog=None,  # type: Optional[bool]
        executionConnectingTrajReverseMult=None,  # type: Optional[float]
        executionReverseRecoveryDistance=None,  # type: Optional[float]
        jittererParameters=None,  # type: Optional[types.JittererParameters]
        gripperInfo=None,  # type: Optional[types.MoveJointsToJointConfigurationStatesParametersGripperInfoVariantItemPrefix0]
        positionConfigurationName=None,  # type: Optional[str]
        positionConfigurationCandidateNames=None,  # type: Optional[list[str]]
        robotspeedmult=None,  # type: Optional[float]
        robotJointNames=None,  # type: Optional[list[str]]
        startvalues=None,  # type: Optional[list[float]]
        goaljoints=None,  # type: Optional[list[float]]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Moves the robot to desired joint angles specified in jointStates

        Args:
            goalJointConfigurationStates: List of dicts for each joint entry.
            robotname: Name of the robot (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            execute: If 1, execute the motion. (Default: 1)
            startJointConfigurationStates: List of dicts for each joint. (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            jointStates: List[{'jointName':str, 'jointValue':float}] (Default: None)
            jointindices: **deprecated** List of corresponding joint indices, default is range(len(jointvalues)) (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
            constraintToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            departOffsetDir: Direction in which to apply the offset when departing from the pick/place operation. (Default: None)
            departMinimumCompleteRatio: The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0. (Default: None)
            departOffsetAwayFromGravity: The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir. (Default: None)
            trajname: (Default: None)
            disablebodies: (Default: None)
            ignoreGrabbingTarget: (Default: None)
            jointthresh: (Default: None)
            jitter: (Default: None)
            executionFilterFactor: (Default: None)
            filtertraj: (Default: None)
            currentlimitratios: The joints' current limit ratios. (Default: None)
            goalJointThreshold: Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time. (Default: None)
            goalWorkspaceThreshold: Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful. (Default: None)
            calibrategripper: (Default: None)
            departAccel: (Default: None)
            departOffsetInTool: (Default: None)
            departSpeed: (Default: None)
            maxManipAccel: (Default: None)
            maxJitterLinkDist: mm, When jittering, the max distance any link on the robot can move. (Default: None)
            pathPlannerParameters: Parameters for robot path planning. (Default: None)
            moveStraightParams: A set of parameters defining how the robot behaves during linear motions. (Default: None)
            forceTorqueBasedEstimatorParameters: Parameters for state estimation features based on force torque sensor (Default: None)
            savetrajectorylog: If True, will save the commanded (input) trajectories before they are executed (Default: None)
            saveRobotFeedbackLog: If True, will tell robotbridge to save trajectory files (Default: None)
            loadRobotFeedbackLog: If True, will tell robotbridge to load the robot feedback log after trajectory ends (Default: None)
            saveConcatenateTrajectoryLog: If True, will save trajectories used for inputs of concatenate trajectory functions (Default: None)
            saveFilterTrajectoryLog: If True, will save trajectories used for filtering, such as SmartFilter (Default: None)
            executionConnectingTrajReverseMult: Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally. (Default: None)
            executionReverseRecoveryDistance: Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm). (Default: None)
            jittererParameters: Parameters dealing with jittering the robot out of collisions. (Default: None)
            gripperInfo: TODO(felixvd): Check if this really propagates. (Default: None)
            positionConfigurationName: If specified, the name of position configuration to move to. If it does not exist, will raise an error. (Default: None)
            positionConfigurationCandidateNames: If specified, goes to the first position that is defined for the robot. If no positions exist, returns without moving the robot. (Default: None)
            robotspeedmult: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotJointNames: (Default: None)
            startvalues: The robot joint values to start the motion from. (Default: None)
            goaljoints: List of joint values to move to. (Default: None)
        """
        taskparameters = {
            'command': 'MoveJointsToJointConfigurationStates',
            'goalJointConfigurationStates': goalJointConfigurationStates,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if execute != 1:
            taskparameters['execute'] = execute
        if startJointConfigurationStates is not None:
            taskparameters['startJointConfigurationStates'] = startJointConfigurationStates
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if jointStates is not None:
            taskparameters['jointStates'] = jointStates
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if ionames is not None:
            taskparameters['ionames'] = ionames
        if constraintToolDirection is not None:
            taskparameters['constraintToolDirection'] = constraintToolDirection
        if departOffsetDir is not None:
            taskparameters['departOffsetDir'] = departOffsetDir
        if departMinimumCompleteRatio is not None:
            taskparameters['departMinimumCompleteRatio'] = departMinimumCompleteRatio
        if departOffsetAwayFromGravity is not None:
            taskparameters['departOffsetAwayFromGravity'] = departOffsetAwayFromGravity
        if trajname is not None:
            taskparameters['trajname'] = trajname
        if disablebodies is not None:
            taskparameters['disablebodies'] = disablebodies
        if ignoreGrabbingTarget is not None:
            taskparameters['ignoreGrabbingTarget'] = ignoreGrabbingTarget
        if jointthresh is not None:
            taskparameters['jointthresh'] = jointthresh
        if jitter is not None:
            taskparameters['jitter'] = jitter
        if executionFilterFactor is not None:
            taskparameters['executionFilterFactor'] = executionFilterFactor
        if filtertraj is not None:
            taskparameters['filtertraj'] = filtertraj
        if currentlimitratios is not None:
            taskparameters['currentlimitratios'] = currentlimitratios
        if goalJointThreshold is not None:
            taskparameters['goalJointThreshold'] = goalJointThreshold
        if goalWorkspaceThreshold is not None:
            taskparameters['goalWorkspaceThreshold'] = goalWorkspaceThreshold
        if calibrategripper is not None:
            taskparameters['calibrategripper'] = calibrategripper
        if departAccel is not None:
            taskparameters['departAccel'] = departAccel
        if departOffsetInTool is not None:
            taskparameters['departOffsetInTool'] = departOffsetInTool
        if departSpeed is not None:
            taskparameters['departSpeed'] = departSpeed
        if maxManipAccel is not None:
            taskparameters['maxManipAccel'] = maxManipAccel
        if maxJitterLinkDist is not None:
            taskparameters['maxJitterLinkDist'] = maxJitterLinkDist
        if pathPlannerParameters is not None:
            taskparameters['pathPlannerParameters'] = pathPlannerParameters
        if moveStraightParams is not None:
            taskparameters['moveStraightParams'] = moveStraightParams
        if forceTorqueBasedEstimatorParameters is not None:
            taskparameters['forceTorqueBasedEstimatorParameters'] = forceTorqueBasedEstimatorParameters
        if savetrajectorylog is not None:
            taskparameters['savetrajectorylog'] = savetrajectorylog
        if saveRobotFeedbackLog is not None:
            taskparameters['saveRobotFeedbackLog'] = saveRobotFeedbackLog
        if loadRobotFeedbackLog is not None:
            taskparameters['loadRobotFeedbackLog'] = loadRobotFeedbackLog
        if saveConcatenateTrajectoryLog is not None:
            taskparameters['saveConcatenateTrajectoryLog'] = saveConcatenateTrajectoryLog
        if saveFilterTrajectoryLog is not None:
            taskparameters['saveFilterTrajectoryLog'] = saveFilterTrajectoryLog
        if executionConnectingTrajReverseMult is not None:
            taskparameters['executionConnectingTrajReverseMult'] = executionConnectingTrajReverseMult
        if executionReverseRecoveryDistance is not None:
            taskparameters['executionReverseRecoveryDistance'] = executionReverseRecoveryDistance
        if jittererParameters is not None:
            taskparameters['jittererParameters'] = jittererParameters
        if gripperInfo is not None:
            taskparameters['gripperInfo'] = gripperInfo
        if positionConfigurationName is not None:
            taskparameters['positionConfigurationName'] = positionConfigurationName
        if positionConfigurationCandidateNames is not None:
            taskparameters['positionConfigurationCandidateNames'] = positionConfigurationCandidateNames
        if robotspeedmult is not None:
            taskparameters['robotspeedmult'] = robotspeedmult
        if robotJointNames is not None:
            taskparameters['robotJointNames'] = robotJointNames
        if startvalues is not None:
            taskparameters['startvalues'] = startvalues
        if goaljoints is not None:
            taskparameters['goaljoints'] = goaljoints
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def MoveJoints(
        self,
        jointvalues,  # type: list[float]
        robotJointNames=None,  # type: Optional[list[str]]
        robotname=None,  # type: Optional[str]
        robotspeed=None,  # type: Optional[float]
        robotaccelmult=None,  # type: Optional[float]
        execute=1,  # type: int
        startvalues=None,  # type: Optional[list[float]]
        envclearance=None,  # type: Optional[float]
        timeout=10,  # type: float
        jointindices=_deprecated,  # type: Optional[list[int]]
        robotProgramName=None,  # type: Optional[str]
        goaljoints=None,  # type: Optional[list[float]]
        dynamicEnvironmentState=None,  # type: Optional[types.MoveJointsParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        toolname=None,  # type: Optional[str]
        robotBridgeConnectionInfo=None,  # type: Optional[types.MoveJointsParametersRobotBridgeConnectionInfo]
        locationCollisionInfos=None,  # type: Optional[list[types.MoveJointsParametersLocationCollisionInfosArrayElement]]
        speed=_deprecated,  # type: Optional[Any]
        ionames=None,  # type: Optional[list[Any]]
        constraintToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        departOffsetDir=None,  # type: Optional[tuple[float, float, float]]
        departMinimumCompleteRatio=None,  # type: Optional[float]
        departOffsetAwayFromGravity=None,  # type: Optional[float]
        trajname=None,  # type: Optional[str]
        disablebodies=None,  # type: Optional[bool]
        ignoreGrabbingTarget=None,  # type: Optional[bool]
        jointthresh=None,  # type: Optional[float]
        jitter=None,  # type: Optional[float]
        executionFilterFactor=None,  # type: Optional[float]
        filtertraj=None,  # type: Optional[bool]
        currentlimitratios=None,  # type: Optional[list[float]]
        goalJointThreshold=None,  # type: Optional[list[float]]
        goalWorkspaceThreshold=None,  # type: Optional[float]
        calibrategripper=None,  # type: Optional[bool]
        departAccel=None,  # type: Optional[float]
        departOffsetInTool=None,  # type: Optional[bool]
        departSpeed=None,  # type: Optional[float]
        maxManipAccel=None,  # type: Optional[float]
        maxJitterLinkDist=None,  # type: Optional[float]
        pathPlannerParameters=None,  # type: Optional[types.PathPlannerParameters]
        moveStraightParams=None,  # type: Optional[types.MoveStraightParameters]
        forceTorqueBasedEstimatorParameters=None,  # type: Optional[types.ForceTorqueBasedEstimatorParameters]
        savetrajectorylog=None,  # type: Optional[bool]
        saveRobotFeedbackLog=None,  # type: Optional[bool]
        loadRobotFeedbackLog=None,  # type: Optional[bool]
        saveConcatenateTrajectoryLog=None,  # type: Optional[bool]
        saveFilterTrajectoryLog=None,  # type: Optional[bool]
        executionConnectingTrajReverseMult=None,  # type: Optional[float]
        executionReverseRecoveryDistance=None,  # type: Optional[float]
        jittererParameters=None,  # type: Optional[types.JittererParameters]
        gripperInfo=None,  # type: Optional[types.MoveJointsParametersGripperInfoVariantItemPrefix0]
        positionConfigurationName=None,  # type: Optional[str]
        positionConfigurationCandidateNames=None,  # type: Optional[list[str]]
        robotspeedmult=None,  # type: Optional[float]
        startJointConfigurationStates=None,  # type: Optional[list[types.MoveJointsParametersStartJointConfigurationStatesArrayElement]]
        goalJointConfigurationStates=None,  # type: Optional[list[types.MoveJointsParametersGoalJointConfigurationStatesArrayElement]]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Moves the robot to desired joint angles specified in jointvalues

        Args:
            jointvalues: List of joint values to move to. Use goaljoints instead.
            robotJointNames: List of corresponding joint names for jointvalues. (Default: None)
            robotname: Name of the robot (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            execute: If 1, execute the motion. (Default: 1)
            startvalues: The robot joint values to start the motion from. (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            jointindices: **deprecated** List of corresponding joint indices, default is range(len(jointvalues)) (Default: None)
            robotProgramName: (Default: None)
            goaljoints: List of joint values to move to. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
            constraintToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            departOffsetDir: Direction in which to apply the offset when departing from the pick/place operation. (Default: None)
            departMinimumCompleteRatio: The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0. (Default: None)
            departOffsetAwayFromGravity: The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir. (Default: None)
            trajname: (Default: None)
            disablebodies: (Default: None)
            ignoreGrabbingTarget: (Default: None)
            jointthresh: (Default: None)
            jitter: (Default: None)
            executionFilterFactor: (Default: None)
            filtertraj: (Default: None)
            currentlimitratios: The joints' current limit ratios. (Default: None)
            goalJointThreshold: Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time. (Default: None)
            goalWorkspaceThreshold: Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful. (Default: None)
            calibrategripper: (Default: None)
            departAccel: (Default: None)
            departOffsetInTool: (Default: None)
            departSpeed: (Default: None)
            maxManipAccel: (Default: None)
            maxJitterLinkDist: mm, When jittering, the max distance any link on the robot can move. (Default: None)
            pathPlannerParameters: Parameters for robot path planning. (Default: None)
            moveStraightParams: A set of parameters defining how the robot behaves during linear motions. (Default: None)
            forceTorqueBasedEstimatorParameters: Parameters for state estimation features based on force torque sensor (Default: None)
            savetrajectorylog: If True, will save the commanded (input) trajectories before they are executed (Default: None)
            saveRobotFeedbackLog: If True, will tell robotbridge to save trajectory files (Default: None)
            loadRobotFeedbackLog: If True, will tell robotbridge to load the robot feedback log after trajectory ends (Default: None)
            saveConcatenateTrajectoryLog: If True, will save trajectories used for inputs of concatenate trajectory functions (Default: None)
            saveFilterTrajectoryLog: If True, will save trajectories used for filtering, such as SmartFilter (Default: None)
            executionConnectingTrajReverseMult: Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally. (Default: None)
            executionReverseRecoveryDistance: Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm). (Default: None)
            jittererParameters: Parameters dealing with jittering the robot out of collisions. (Default: None)
            gripperInfo: TODO(felixvd): Check if this really propagates. (Default: None)
            positionConfigurationName: If specified, the name of position configuration to move to. If it does not exist, will raise an error. (Default: None)
            positionConfigurationCandidateNames: If specified, goes to the first position that is defined for the robot. If no positions exist, returns without moving the robot. (Default: None)
            robotspeedmult: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            startJointConfigurationStates: List of dicts for each joint. (Default: None)
            goalJointConfigurationStates: List of dicts for each joint entry. (Default: None)
        """
        taskparameters = {
            'command': 'MoveJoints',
        }  # type: dict[str, Any]
        taskparameters['goaljoints'] = list(jointvalues)
        if robotJointNames is not None:
            taskparameters['robotJointNames'] = robotJointNames
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if execute != 1:
            taskparameters['execute'] = execute
        if startvalues is not None:
            taskparameters['startvalues'] = list(startvalues)
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if robotProgramName is not None:
            taskparameters['robotProgramName'] = list(robotProgramName)
        if goaljoints is not None:
            taskparameters['goaljoints'] = list(goaljoints)
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if ionames is not None:
            taskparameters['ionames'] = ionames
        if constraintToolDirection is not None:
            taskparameters['constraintToolDirection'] = constraintToolDirection
        if departOffsetDir is not None:
            taskparameters['departOffsetDir'] = departOffsetDir
        if departMinimumCompleteRatio is not None:
            taskparameters['departMinimumCompleteRatio'] = departMinimumCompleteRatio
        if departOffsetAwayFromGravity is not None:
            taskparameters['departOffsetAwayFromGravity'] = departOffsetAwayFromGravity
        if trajname is not None:
            taskparameters['trajname'] = trajname
        if disablebodies is not None:
            taskparameters['disablebodies'] = disablebodies
        if ignoreGrabbingTarget is not None:
            taskparameters['ignoreGrabbingTarget'] = ignoreGrabbingTarget
        if jointthresh is not None:
            taskparameters['jointthresh'] = jointthresh
        if jitter is not None:
            taskparameters['jitter'] = jitter
        if executionFilterFactor is not None:
            taskparameters['executionFilterFactor'] = executionFilterFactor
        if filtertraj is not None:
            taskparameters['filtertraj'] = filtertraj
        if currentlimitratios is not None:
            taskparameters['currentlimitratios'] = currentlimitratios
        if goalJointThreshold is not None:
            taskparameters['goalJointThreshold'] = goalJointThreshold
        if goalWorkspaceThreshold is not None:
            taskparameters['goalWorkspaceThreshold'] = goalWorkspaceThreshold
        if calibrategripper is not None:
            taskparameters['calibrategripper'] = calibrategripper
        if departAccel is not None:
            taskparameters['departAccel'] = departAccel
        if departOffsetInTool is not None:
            taskparameters['departOffsetInTool'] = departOffsetInTool
        if departSpeed is not None:
            taskparameters['departSpeed'] = departSpeed
        if maxManipAccel is not None:
            taskparameters['maxManipAccel'] = maxManipAccel
        if maxJitterLinkDist is not None:
            taskparameters['maxJitterLinkDist'] = maxJitterLinkDist
        if pathPlannerParameters is not None:
            taskparameters['pathPlannerParameters'] = pathPlannerParameters
        if moveStraightParams is not None:
            taskparameters['moveStraightParams'] = moveStraightParams
        if forceTorqueBasedEstimatorParameters is not None:
            taskparameters['forceTorqueBasedEstimatorParameters'] = forceTorqueBasedEstimatorParameters
        if savetrajectorylog is not None:
            taskparameters['savetrajectorylog'] = savetrajectorylog
        if saveRobotFeedbackLog is not None:
            taskparameters['saveRobotFeedbackLog'] = saveRobotFeedbackLog
        if loadRobotFeedbackLog is not None:
            taskparameters['loadRobotFeedbackLog'] = loadRobotFeedbackLog
        if saveConcatenateTrajectoryLog is not None:
            taskparameters['saveConcatenateTrajectoryLog'] = saveConcatenateTrajectoryLog
        if saveFilterTrajectoryLog is not None:
            taskparameters['saveFilterTrajectoryLog'] = saveFilterTrajectoryLog
        if executionConnectingTrajReverseMult is not None:
            taskparameters['executionConnectingTrajReverseMult'] = executionConnectingTrajReverseMult
        if executionReverseRecoveryDistance is not None:
            taskparameters['executionReverseRecoveryDistance'] = executionReverseRecoveryDistance
        if jittererParameters is not None:
            taskparameters['jittererParameters'] = jittererParameters
        if gripperInfo is not None:
            taskparameters['gripperInfo'] = gripperInfo
        if positionConfigurationName is not None:
            taskparameters['positionConfigurationName'] = positionConfigurationName
        if positionConfigurationCandidateNames is not None:
            taskparameters['positionConfigurationCandidateNames'] = positionConfigurationCandidateNames
        if robotspeedmult is not None:
            taskparameters['robotspeedmult'] = robotspeedmult
        if startJointConfigurationStates is not None:
            taskparameters['startJointConfigurationStates'] = startJointConfigurationStates
        if goalJointConfigurationStates is not None:
            taskparameters['goalJointConfigurationStates'] = goalJointConfigurationStates
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def MoveJointsToPositionConfiguration(
        self,
        positionConfigurationName=None,  # type: Optional[str]
        positionConfigurationCandidateNames=None,  # type: Optional[list[str]]
        robotname=None,  # type: Optional[str]
        robotspeed=None,  # type: Optional[float]
        robotaccelmult=None,  # type: Optional[float]
        execute=1,  # type: int
        startvalues=None,  # type: Optional[list[float]]
        envclearance=None,  # type: Optional[float]
        timeout=10,  # type: float
        dynamicEnvironmentState=None,  # type: Optional[types.MoveJointsToPositionConfigurationParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        toolname=None,  # type: Optional[str]
        robotBridgeConnectionInfo=None,  # type: Optional[types.MoveJointsToPositionConfigurationParametersRobotBridgeConnectionInfo]
        locationCollisionInfos=None,  # type: Optional[list[types.MoveJointsToPositionConfigurationParametersLocationCollisionInfosArrayElement]]
        speed=_deprecated,  # type: Optional[Any]
        ionames=None,  # type: Optional[list[Any]]
        constraintToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        departOffsetDir=None,  # type: Optional[tuple[float, float, float]]
        departMinimumCompleteRatio=None,  # type: Optional[float]
        departOffsetAwayFromGravity=None,  # type: Optional[float]
        trajname=None,  # type: Optional[str]
        disablebodies=None,  # type: Optional[bool]
        ignoreGrabbingTarget=None,  # type: Optional[bool]
        jointthresh=None,  # type: Optional[float]
        jitter=None,  # type: Optional[float]
        executionFilterFactor=None,  # type: Optional[float]
        filtertraj=None,  # type: Optional[bool]
        currentlimitratios=None,  # type: Optional[list[float]]
        goalJointThreshold=None,  # type: Optional[list[float]]
        goalWorkspaceThreshold=None,  # type: Optional[float]
        calibrategripper=None,  # type: Optional[bool]
        departAccel=None,  # type: Optional[float]
        departOffsetInTool=None,  # type: Optional[bool]
        departSpeed=None,  # type: Optional[float]
        maxManipAccel=None,  # type: Optional[float]
        maxJitterLinkDist=None,  # type: Optional[float]
        pathPlannerParameters=None,  # type: Optional[types.PathPlannerParameters]
        moveStraightParams=None,  # type: Optional[types.MoveStraightParameters]
        forceTorqueBasedEstimatorParameters=None,  # type: Optional[types.ForceTorqueBasedEstimatorParameters]
        savetrajectorylog=None,  # type: Optional[bool]
        saveRobotFeedbackLog=None,  # type: Optional[bool]
        loadRobotFeedbackLog=None,  # type: Optional[bool]
        saveConcatenateTrajectoryLog=None,  # type: Optional[bool]
        saveFilterTrajectoryLog=None,  # type: Optional[bool]
        executionConnectingTrajReverseMult=None,  # type: Optional[float]
        executionReverseRecoveryDistance=None,  # type: Optional[float]
        jittererParameters=None,  # type: Optional[types.JittererParameters]
        gripperInfo=None,  # type: Optional[types.MoveJointsToPositionConfigurationParametersGripperInfoVariantItemPrefix0]
        robotspeedmult=None,  # type: Optional[float]
        robotJointNames=None,  # type: Optional[list[str]]
        jointindices=_deprecated,  # type: Optional[list[int]]
        startJointConfigurationStates=None,  # type: Optional[list[types.MoveJointsToPositionConfigurationParametersStartJointConfigurationStatesArrayElement]]
        goalJointConfigurationStates=None,  # type: Optional[list[types.MoveJointsToPositionConfigurationParametersGoalJointConfigurationStatesArrayElement]]
        goaljoints=None,  # type: Optional[list[float]]
        robotProgramName=None,  # type: Optional[str]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[types.MoveJointsToPositionConfigurationReturns]
        """
        Moves the robot to desired position configuration specified in positionConfigurationName

        Args:
            positionConfigurationName: If specified, the name of position configuration to move to. If it does not exist, will raise an error. (Default: None)
            positionConfigurationCandidateNames: If specified, goes to the first position that is defined for the robot. If no positions exist, returns without moving the robot. (Default: None)
            robotname: Name of the robot (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            execute: If 1, execute the motion. (Default: 1)
            startvalues: The robot joint values to start the motion from. (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
            constraintToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            departOffsetDir: Direction in which to apply the offset when departing from the pick/place operation. (Default: None)
            departMinimumCompleteRatio: The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0. (Default: None)
            departOffsetAwayFromGravity: The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir. (Default: None)
            trajname: (Default: None)
            disablebodies: (Default: None)
            ignoreGrabbingTarget: (Default: None)
            jointthresh: (Default: None)
            jitter: (Default: None)
            executionFilterFactor: (Default: None)
            filtertraj: (Default: None)
            currentlimitratios: The joints' current limit ratios. (Default: None)
            goalJointThreshold: Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time. (Default: None)
            goalWorkspaceThreshold: Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful. (Default: None)
            calibrategripper: (Default: None)
            departAccel: (Default: None)
            departOffsetInTool: (Default: None)
            departSpeed: (Default: None)
            maxManipAccel: (Default: None)
            maxJitterLinkDist: mm, When jittering, the max distance any link on the robot can move. (Default: None)
            pathPlannerParameters: Parameters for robot path planning. (Default: None)
            moveStraightParams: A set of parameters defining how the robot behaves during linear motions. (Default: None)
            forceTorqueBasedEstimatorParameters: Parameters for state estimation features based on force torque sensor (Default: None)
            savetrajectorylog: If True, will save the commanded (input) trajectories before they are executed (Default: None)
            saveRobotFeedbackLog: If True, will tell robotbridge to save trajectory files (Default: None)
            loadRobotFeedbackLog: If True, will tell robotbridge to load the robot feedback log after trajectory ends (Default: None)
            saveConcatenateTrajectoryLog: If True, will save trajectories used for inputs of concatenate trajectory functions (Default: None)
            saveFilterTrajectoryLog: If True, will save trajectories used for filtering, such as SmartFilter (Default: None)
            executionConnectingTrajReverseMult: Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally. (Default: None)
            executionReverseRecoveryDistance: Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm). (Default: None)
            jittererParameters: Parameters dealing with jittering the robot out of collisions. (Default: None)
            gripperInfo: TODO(felixvd): Check if this really propagates. (Default: None)
            robotspeedmult: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotJointNames: (Default: None)
            jointindices: **deprecated** List of corresponding joint indices, default is range(len(jointvalues)) (Default: None)
            startJointConfigurationStates: List of dicts for each joint. (Default: None)
            goalJointConfigurationStates: List of dicts for each joint entry. (Default: None)
            goaljoints: List of joint values to move to. (Default: None)
            robotProgramName: (Default: None)

        Returns:
            Dictionary with keys goalPositionName and values goalConfiguration
        """
        taskparameters = {
            'command': 'MoveJointsToPositionConfiguration',
        }  # type: dict[str, Any]
        if positionConfigurationName:
            taskparameters['positionConfigurationName'] = positionConfigurationName
        if positionConfigurationCandidateNames:
            taskparameters['positionConfigurationCandidateNames'] = positionConfigurationCandidateNames
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if execute != 1:
            taskparameters['execute'] = execute
        if startvalues is not None:
            taskparameters['startvalues'] = list(startvalues)
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if ionames is not None:
            taskparameters['ionames'] = ionames
        if constraintToolDirection is not None:
            taskparameters['constraintToolDirection'] = constraintToolDirection
        if departOffsetDir is not None:
            taskparameters['departOffsetDir'] = departOffsetDir
        if departMinimumCompleteRatio is not None:
            taskparameters['departMinimumCompleteRatio'] = departMinimumCompleteRatio
        if departOffsetAwayFromGravity is not None:
            taskparameters['departOffsetAwayFromGravity'] = departOffsetAwayFromGravity
        if trajname is not None:
            taskparameters['trajname'] = trajname
        if disablebodies is not None:
            taskparameters['disablebodies'] = disablebodies
        if ignoreGrabbingTarget is not None:
            taskparameters['ignoreGrabbingTarget'] = ignoreGrabbingTarget
        if jointthresh is not None:
            taskparameters['jointthresh'] = jointthresh
        if jitter is not None:
            taskparameters['jitter'] = jitter
        if executionFilterFactor is not None:
            taskparameters['executionFilterFactor'] = executionFilterFactor
        if filtertraj is not None:
            taskparameters['filtertraj'] = filtertraj
        if currentlimitratios is not None:
            taskparameters['currentlimitratios'] = currentlimitratios
        if goalJointThreshold is not None:
            taskparameters['goalJointThreshold'] = goalJointThreshold
        if goalWorkspaceThreshold is not None:
            taskparameters['goalWorkspaceThreshold'] = goalWorkspaceThreshold
        if calibrategripper is not None:
            taskparameters['calibrategripper'] = calibrategripper
        if departAccel is not None:
            taskparameters['departAccel'] = departAccel
        if departOffsetInTool is not None:
            taskparameters['departOffsetInTool'] = departOffsetInTool
        if departSpeed is not None:
            taskparameters['departSpeed'] = departSpeed
        if maxManipAccel is not None:
            taskparameters['maxManipAccel'] = maxManipAccel
        if maxJitterLinkDist is not None:
            taskparameters['maxJitterLinkDist'] = maxJitterLinkDist
        if pathPlannerParameters is not None:
            taskparameters['pathPlannerParameters'] = pathPlannerParameters
        if moveStraightParams is not None:
            taskparameters['moveStraightParams'] = moveStraightParams
        if forceTorqueBasedEstimatorParameters is not None:
            taskparameters['forceTorqueBasedEstimatorParameters'] = forceTorqueBasedEstimatorParameters
        if savetrajectorylog is not None:
            taskparameters['savetrajectorylog'] = savetrajectorylog
        if saveRobotFeedbackLog is not None:
            taskparameters['saveRobotFeedbackLog'] = saveRobotFeedbackLog
        if loadRobotFeedbackLog is not None:
            taskparameters['loadRobotFeedbackLog'] = loadRobotFeedbackLog
        if saveConcatenateTrajectoryLog is not None:
            taskparameters['saveConcatenateTrajectoryLog'] = saveConcatenateTrajectoryLog
        if saveFilterTrajectoryLog is not None:
            taskparameters['saveFilterTrajectoryLog'] = saveFilterTrajectoryLog
        if executionConnectingTrajReverseMult is not None:
            taskparameters['executionConnectingTrajReverseMult'] = executionConnectingTrajReverseMult
        if executionReverseRecoveryDistance is not None:
            taskparameters['executionReverseRecoveryDistance'] = executionReverseRecoveryDistance
        if jittererParameters is not None:
            taskparameters['jittererParameters'] = jittererParameters
        if gripperInfo is not None:
            taskparameters['gripperInfo'] = gripperInfo
        if robotspeedmult is not None:
            taskparameters['robotspeedmult'] = robotspeedmult
        if robotJointNames is not None:
            taskparameters['robotJointNames'] = robotJointNames
        if startJointConfigurationStates is not None:
            taskparameters['startJointConfigurationStates'] = startJointConfigurationStates
        if goalJointConfigurationStates is not None:
            taskparameters['goalJointConfigurationStates'] = goalJointConfigurationStates
        if goaljoints is not None:
            taskparameters['goaljoints'] = goaljoints
        if robotProgramName is not None:
            taskparameters['robotProgramName'] = robotProgramName
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def StartMoveThread(
        self,
        timeout=10,  # type: float
        dynamicEnvironmentState=None,  # type: Optional[types.StartMoveThreadParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        constraintToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        departOffsetDir=None,  # type: Optional[tuple[float, float, float]]
        departMinimumCompleteRatio=None,  # type: Optional[float]
        departOffsetAwayFromGravity=None,  # type: Optional[float]
        trajname=None,  # type: Optional[str]
        disablebodies=None,  # type: Optional[bool]
        ignoreGrabbingTarget=None,  # type: Optional[bool]
        jointthresh=None,  # type: Optional[float]
        envclearance=None,  # type: Optional[float]
        jitter=None,  # type: Optional[float]
        execute=None,  # type: Optional[int]
        executionFilterFactor=None,  # type: Optional[float]
        filtertraj=None,  # type: Optional[bool]
        locationCollisionInfos=None,  # type: Optional[list[types.StartMoveThreadParametersLocationCollisionInfosArrayElement]]
        currentlimitratios=None,  # type: Optional[list[float]]
        goalJointThreshold=None,  # type: Optional[list[float]]
        goalWorkspaceThreshold=None,  # type: Optional[float]
        calibrategripper=None,  # type: Optional[bool]
        departAccel=None,  # type: Optional[float]
        departOffsetInTool=None,  # type: Optional[bool]
        departSpeed=None,  # type: Optional[float]
        maxManipAccel=None,  # type: Optional[float]
        maxJitterLinkDist=None,  # type: Optional[float]
        pathPlannerParameters=None,  # type: Optional[types.PathPlannerParameters]
        moveStraightParams=None,  # type: Optional[types.MoveStraightParameters]
        forceTorqueBasedEstimatorParameters=None,  # type: Optional[types.ForceTorqueBasedEstimatorParameters]
        savetrajectorylog=None,  # type: Optional[bool]
        saveRobotFeedbackLog=None,  # type: Optional[bool]
        loadRobotFeedbackLog=None,  # type: Optional[bool]
        saveConcatenateTrajectoryLog=None,  # type: Optional[bool]
        saveFilterTrajectoryLog=None,  # type: Optional[bool]
        executionConnectingTrajReverseMult=None,  # type: Optional[float]
        executionReverseRecoveryDistance=None,  # type: Optional[float]
        jittererParameters=None,  # type: Optional[types.JittererParameters]
        gripperInfo=None,  # type: Optional[types.StartMoveThreadParametersGripperInfoVariantItemPrefix0]
        robotspeed=None,  # type: Optional[float]
        speed=_deprecated,  # type: Optional[Any]
        robotaccelmult=None,  # type: Optional[float]
        ionames=None,  # type: Optional[list[Any]]
        positionConfigurationName=None,  # type: Optional[str]
        positionConfigurationCandidateNames=None,  # type: Optional[list[str]]
        robotname=None,  # type: Optional[str]
        toolname=None,  # type: Optional[str]
        robotspeedmult=None,  # type: Optional[float]
        robotJointNames=None,  # type: Optional[list[str]]
        jointindices=_deprecated,  # type: Optional[list[int]]
        startvalues=None,  # type: Optional[list[float]]
        startJointConfigurationStates=None,  # type: Optional[list[types.StartMoveThreadParametersStartJointConfigurationStatesArrayElement]]
        goalJointConfigurationStates=None,  # type: Optional[list[types.StartMoveThreadParametersGoalJointConfigurationStatesArrayElement]]
        goaljoints=None,  # type: Optional[list[float]]
        robotBridgeConnectionInfo=None  # type: Optional[types.StartMoveThreadParametersRobotBridgeConnectionInfo]
    ):
        # type: (...) -> Optional[Any]
        """
        Moves the robot to desired position configuration specified in positionConfigurationName.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            constraintToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            departOffsetDir: Direction in which to apply the offset when departing from the pick/place operation. (Default: None)
            departMinimumCompleteRatio: The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0. (Default: None)
            departOffsetAwayFromGravity: The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir. (Default: None)
            trajname: (Default: None)
            disablebodies: (Default: None)
            ignoreGrabbingTarget: (Default: None)
            jointthresh: (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            jitter: (Default: None)
            execute: If 1, execute the motion. (Default: None)
            executionFilterFactor: (Default: None)
            filtertraj: (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            currentlimitratios: The joints' current limit ratios. (Default: None)
            goalJointThreshold: Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time. (Default: None)
            goalWorkspaceThreshold: Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful. (Default: None)
            calibrategripper: (Default: None)
            departAccel: (Default: None)
            departOffsetInTool: (Default: None)
            departSpeed: (Default: None)
            maxManipAccel: (Default: None)
            maxJitterLinkDist: mm, When jittering, the max distance any link on the robot can move. (Default: None)
            pathPlannerParameters: Parameters for robot path planning. (Default: None)
            moveStraightParams: A set of parameters defining how the robot behaves during linear motions. (Default: None)
            forceTorqueBasedEstimatorParameters: Parameters for state estimation features based on force torque sensor (Default: None)
            savetrajectorylog: If True, will save the commanded (input) trajectories before they are executed (Default: None)
            saveRobotFeedbackLog: If True, will tell robotbridge to save trajectory files (Default: None)
            loadRobotFeedbackLog: If True, will tell robotbridge to load the robot feedback log after trajectory ends (Default: None)
            saveConcatenateTrajectoryLog: If True, will save trajectories used for inputs of concatenate trajectory functions (Default: None)
            saveFilterTrajectoryLog: If True, will save trajectories used for filtering, such as SmartFilter (Default: None)
            executionConnectingTrajReverseMult: Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally. (Default: None)
            executionReverseRecoveryDistance: Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm). (Default: None)
            jittererParameters: Parameters dealing with jittering the robot out of collisions. (Default: None)
            gripperInfo: TODO(felixvd): Check if this really propagates. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
            positionConfigurationName: If specified, the name of position configuration to move to. If it does not exist, will raise an error. (Default: None)
            positionConfigurationCandidateNames: If specified, goes to the first position that is defined for the robot. If no positions exist, returns without moving the robot. (Default: None)
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotspeedmult: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotJointNames: (Default: None)
            jointindices: **deprecated** List of corresponding joint indices, default is range(len(jointvalues)) (Default: None)
            startvalues: The robot joint values to start the motion from. (Default: None)
            startJointConfigurationStates: List of dicts for each joint. (Default: None)
            goalJointConfigurationStates: List of dicts for each joint entry. (Default: None)
            goaljoints: List of joint values to move to. (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
        """
        taskparameters = {
            'command': 'StartMoveThread',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if constraintToolDirection is not None:
            taskparameters['constraintToolDirection'] = constraintToolDirection
        if departOffsetDir is not None:
            taskparameters['departOffsetDir'] = departOffsetDir
        if departMinimumCompleteRatio is not None:
            taskparameters['departMinimumCompleteRatio'] = departMinimumCompleteRatio
        if departOffsetAwayFromGravity is not None:
            taskparameters['departOffsetAwayFromGravity'] = departOffsetAwayFromGravity
        if trajname is not None:
            taskparameters['trajname'] = trajname
        if disablebodies is not None:
            taskparameters['disablebodies'] = disablebodies
        if ignoreGrabbingTarget is not None:
            taskparameters['ignoreGrabbingTarget'] = ignoreGrabbingTarget
        if jointthresh is not None:
            taskparameters['jointthresh'] = jointthresh
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if jitter is not None:
            taskparameters['jitter'] = jitter
        if execute is not None:
            taskparameters['execute'] = execute
        if executionFilterFactor is not None:
            taskparameters['executionFilterFactor'] = executionFilterFactor
        if filtertraj is not None:
            taskparameters['filtertraj'] = filtertraj
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if currentlimitratios is not None:
            taskparameters['currentlimitratios'] = currentlimitratios
        if goalJointThreshold is not None:
            taskparameters['goalJointThreshold'] = goalJointThreshold
        if goalWorkspaceThreshold is not None:
            taskparameters['goalWorkspaceThreshold'] = goalWorkspaceThreshold
        if calibrategripper is not None:
            taskparameters['calibrategripper'] = calibrategripper
        if departAccel is not None:
            taskparameters['departAccel'] = departAccel
        if departOffsetInTool is not None:
            taskparameters['departOffsetInTool'] = departOffsetInTool
        if departSpeed is not None:
            taskparameters['departSpeed'] = departSpeed
        if maxManipAccel is not None:
            taskparameters['maxManipAccel'] = maxManipAccel
        if maxJitterLinkDist is not None:
            taskparameters['maxJitterLinkDist'] = maxJitterLinkDist
        if pathPlannerParameters is not None:
            taskparameters['pathPlannerParameters'] = pathPlannerParameters
        if moveStraightParams is not None:
            taskparameters['moveStraightParams'] = moveStraightParams
        if forceTorqueBasedEstimatorParameters is not None:
            taskparameters['forceTorqueBasedEstimatorParameters'] = forceTorqueBasedEstimatorParameters
        if savetrajectorylog is not None:
            taskparameters['savetrajectorylog'] = savetrajectorylog
        if saveRobotFeedbackLog is not None:
            taskparameters['saveRobotFeedbackLog'] = saveRobotFeedbackLog
        if loadRobotFeedbackLog is not None:
            taskparameters['loadRobotFeedbackLog'] = loadRobotFeedbackLog
        if saveConcatenateTrajectoryLog is not None:
            taskparameters['saveConcatenateTrajectoryLog'] = saveConcatenateTrajectoryLog
        if saveFilterTrajectoryLog is not None:
            taskparameters['saveFilterTrajectoryLog'] = saveFilterTrajectoryLog
        if executionConnectingTrajReverseMult is not None:
            taskparameters['executionConnectingTrajReverseMult'] = executionConnectingTrajReverseMult
        if executionReverseRecoveryDistance is not None:
            taskparameters['executionReverseRecoveryDistance'] = executionReverseRecoveryDistance
        if jittererParameters is not None:
            taskparameters['jittererParameters'] = jittererParameters
        if gripperInfo is not None:
            taskparameters['gripperInfo'] = gripperInfo
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        if positionConfigurationName is not None:
            taskparameters['positionConfigurationName'] = positionConfigurationName
        if positionConfigurationCandidateNames is not None:
            taskparameters['positionConfigurationCandidateNames'] = positionConfigurationCandidateNames
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotspeedmult is not None:
            taskparameters['robotspeedmult'] = robotspeedmult
        if robotJointNames is not None:
            taskparameters['robotJointNames'] = robotJointNames
        if startvalues is not None:
            taskparameters['startvalues'] = startvalues
        if startJointConfigurationStates is not None:
            taskparameters['startJointConfigurationStates'] = startJointConfigurationStates
        if goalJointConfigurationStates is not None:
            taskparameters['goalJointConfigurationStates'] = goalJointConfigurationStates
        if goaljoints is not None:
            taskparameters['goaljoints'] = goaljoints
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetRobotBridgeIOVariables(self, ioname=None, ionames=None, robotname=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, **kwargs):
        # type: (Optional[str], Optional[list[Any]], Optional[str], float, Optional[types.GetRobotBridgeIOVariablesParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[types.GetRobotBridgeIOVariablesParametersRobotBridgeConnectionInfo], Optional[list[types.GetRobotBridgeIOVariablesParametersLocationCollisionInfosArrayElement]], Optional[Any]) -> Optional[Any]
        """
        Returns the data of the IO in ASCII hex as a string

        Args:
            ioname: One IO name to read (Default: None)
            ionames: A list of the IO names to read (Default: None)
            robotname: Name of the robot (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
        """
        taskparameters = {
            'command': 'GetRobotBridgeIOVariables',
        }  # type: dict[str, Any]
        if ioname is not None and len(ioname) > 0:
            taskparameters['ioname'] = ioname
        if ionames is not None and len(ionames) > 0:
            taskparameters['ionames'] = ionames
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetRobotBridgeIOVariables(self, iovalues, robotname=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, forceasync=None, **kwargs):
        # type: (Any, Optional[str], float, Optional[types.SetRobotBridgeIOVariablesParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[types.SetRobotBridgeIOVariablesParametersRobotBridgeConnectionInfo], Optional[list[types.SetRobotBridgeIOVariablesParametersLocationCollisionInfosArrayElement]], Optional[bool], Optional[Any]) -> Optional[Any]
        """
        Sets a set of IO variables in the robot bridge.
        This should not lock self.env since it can happen during the runtime of a task and lock out other functions waiting in the queue.

        Args:
            iovalues: ioValue names with expected values.
            robotname: Name of the robot (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            forceasync: (Default: None)
        """
        taskparameters = {
            'command': 'SetRobotBridgeIOVariables',
        }  # type: dict[str, Any]
        taskparameters['iovalues'] = list(iovalues)
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if forceasync is not None:
            taskparameters['forceasync'] = forceasync
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ComputeIkParamPosition(self, name, robotname=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, toolname=None, unit='mm', jointvalues=None, **kwargs):
        # type: (str, Optional[str], float, Optional[types.ComputeIkParamPositionParametersDynamicEnvironmentState], Optional[int], Optional[str], str, Optional[list[float]], Optional[Any]) -> Optional[types.ComputeIkParamPositionReturns]
        """
        Given the name of a Kinbody, computes the manipulator (TCP) position in the Kinbody frame to generate values for an IKParameterization.

        Args:
            name: Name of the Kinbody (the robot).
            robotname: Name of the robot (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            jointvalues: If given, the robot's joints are set to these values before calculating the manipulator (TCP) position. If not set, uses the current values. (Default: None)
        """
        taskparameters = {
            'command': 'ComputeIkParamPosition',
            'name': name,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if unit != 'mm':
            taskparameters['unit'] = unit
        if jointvalues is not None:
            taskparameters['jointvalues'] = jointvalues
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ComputeIKFromParameters(
        self,
        toolname=None,  # type: Optional[str]
        timeout=10,  # type: float
        dynamicEnvironmentState=None,  # type: Optional[types.ComputeIKFromParametersParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        targetname=None,  # type: Optional[str]
        graspsetname=None,  # type: Optional[str]
        ikparamnames=None,  # type: Optional[list[str]]
        limit=None,  # type: Optional[float]
        useSolutionIndices=None,  # type: Optional[bool]
        disabletarget=None,  # type: Optional[bool]
        unit='mm',  # type: str
        randomBoxInfo=None,  # type: Optional[types.RandomBoxInfo]
        freeincvalue=None,  # type: Optional[float]
        freeinc=_deprecated,  # type: Optional[float]
        applyapproachoffset=None,  # type: Optional[bool]
        inPlaneAngleDeviation=None,  # type: Optional[float]
        outOfPlaneAngleDeviation=None,  # type: Optional[float]
        searchfreeparams=None,  # type: Optional[bool]
        returnClosestToCurrent=None,  # type: Optional[bool]
        filteroptionslist=None,  # type: Optional[list[str]]
        filteroptions=None,  # type: Optional[int]
        robotname=None,  # type: Optional[str]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[types.ComputeIKFromParametersReturns]
        """
        Args:
            toolname: Tool name (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            targetname: Name of the target object. (Default: None)
            graspsetname: Name of the grasp set to use (Default: None)
            ikparamnames: If graspset does not exist, use the ikparamnames to initialize the grasp. (Default: None)
            limit: Number of solutions to return (Default: None)
            useSolutionIndices: (Default: None)
            disabletarget: (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            randomBoxInfo: Info structure for maintaining grasp parameters for random box picking. Used when picking up randomized boxes (targetIsRandomBox is True). Keys: usefaces, dictFacePriorities, boxDirAngle, toolTranslationOffsets (Default: None)
            freeincvalue: The discretization of the free joints of the robot when computing ik. (Default: None)
            freeinc: **deprecated** The discretization of the free joints of the robot when computing ik. (Default: None)
            applyapproachoffset: (Default: None)
            inPlaneAngleDeviation: (Default: None)
            outOfPlaneAngleDeviation: (Default: None)
            searchfreeparams: (Default: None)
            returnClosestToCurrent: (Default: None)
            filteroptionslist: A list of filter option strings. Can be: CheckEnvCollisions, IgnoreCustomFilters, IgnoreEndEffectorCollisions, IgnoreEndEffectorEnvCollisions, IgnoreEndEffectorSelfCollisions, IgnoreJointLimits, IgnoreSelfCollisions. Overrides filteroptions. (Default: None)
            filteroptions: OpenRAVE IkFilterOptions bitmask. By default this is 1, which means all collisions are checked (Default: None)
            robotname: Name of the robot (Default: None)
        """
        taskparameters = {
            'command': 'ComputeIKFromParameters',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if targetname is not None:
            taskparameters['targetname'] = targetname
        if graspsetname is not None:
            taskparameters['graspsetname'] = graspsetname
        if ikparamnames is not None:
            taskparameters['ikparamnames'] = ikparamnames
        if limit is not None:
            taskparameters['limit'] = limit
        if useSolutionIndices is not None:
            taskparameters['useSolutionIndices'] = useSolutionIndices
        if disabletarget is not None:
            taskparameters['disabletarget'] = disabletarget
        if unit != 'mm':
            taskparameters['unit'] = unit
        if randomBoxInfo is not None:
            taskparameters['randomBoxInfo'] = randomBoxInfo
        if freeincvalue is not None:
            taskparameters['freeincvalue'] = freeincvalue
        if applyapproachoffset is not None:
            taskparameters['applyapproachoffset'] = applyapproachoffset
        if inPlaneAngleDeviation is not None:
            taskparameters['inPlaneAngleDeviation'] = inPlaneAngleDeviation
        if outOfPlaneAngleDeviation is not None:
            taskparameters['outOfPlaneAngleDeviation'] = outOfPlaneAngleDeviation
        if searchfreeparams is not None:
            taskparameters['searchfreeparams'] = searchfreeparams
        if returnClosestToCurrent is not None:
            taskparameters['returnClosestToCurrent'] = returnClosestToCurrent
        if filteroptionslist is not None:
            taskparameters['filteroptionslist'] = filteroptionslist
        if filteroptions is not None:
            taskparameters['filteroptions'] = filteroptions
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout)

    def ReloadModule(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, **kwargs):
        # type: (float, Optional[types.ReloadModuleParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.ReloadModuleParametersRobotBridgeConnectionInfo], Optional[list[types.ReloadModuleParametersLocationCollisionInfosArrayElement]], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
        """
        taskparameters = {
            'command': 'ReloadModule',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ShutdownRobotBridge(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, **kwargs):
        # type: (float, Optional[types.ShutdownRobotBridgeParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.ShutdownRobotBridgeParametersRobotBridgeConnectionInfo], Optional[list[types.ShutdownRobotBridgeParametersLocationCollisionInfosArrayElement]], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
        """
        taskparameters = {
            'command': 'ShutdownRobotBridge',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetRobotBridgeState(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, ionames=None, **kwargs):
        # type: (float, Optional[types.GetRobotBridgeStateParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.GetRobotBridgeStateParametersRobotBridgeConnectionInfo], Optional[list[types.GetRobotBridgeStateParametersLocationCollisionInfosArrayElement]], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'GetRobotBridgeState',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ClearRobotBridgeError(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, **kwargs):
        # type: (float, Optional[types.ClearRobotBridgeErrorParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.ClearRobotBridgeErrorParametersRobotBridgeConnectionInfo], Optional[list[types.ClearRobotBridgeErrorParametersLocationCollisionInfosArrayElement]], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
        """
        taskparameters = {
            'command': 'ClearRobotBridgeError',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetRobotBridgePause(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, **kwargs):
        # type: (float, Optional[types.SetRobotBridgePauseParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.SetRobotBridgePauseParametersRobotBridgeConnectionInfo], Optional[list[types.SetRobotBridgePauseParametersLocationCollisionInfosArrayElement]], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
        """
        taskparameters = {
            'command': 'SetRobotBridgePause',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetRobotBridgeResume(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, **kwargs):
        # type: (float, Optional[types.SetRobotBridgeResumeParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.SetRobotBridgeResumeParametersRobotBridgeConnectionInfo], Optional[list[types.SetRobotBridgeResumeParametersLocationCollisionInfosArrayElement]], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
        """
        taskparameters = {
            'command': 'SetRobotBridgeResume',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetRobotBridgeServoOn(self, isservoon, robotname=None, timeout=3, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None):
        # type: (bool, Optional[str], float, bool, Optional[types.SetRobotBridgeServoOnParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Args:
            isservoon: If True, turns servo on.
            robotname: Name of the robot (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 3)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetRobotBridgeServoOn',
            'isservoon': isservoon,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetRobotBridgeLockMode(self, islockmode, robotname=None, timeout=3, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None):
        # type: (bool, Optional[str], float, bool, Optional[types.SetRobotBridgeLockModeParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Args:
            islockmode: If True, turns on Lock Mode. During Lock Mode, all communication with the physical robot is turned off and the hardware will not move.
            robotname: Name of the robot (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 3)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetRobotBridgeLockMode',
            'islockmode': islockmode,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResetSafetyFault(self, timeout=3, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None):
        # type: (float, bool, Optional[types.ResetSafetyFaultParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 3)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ResetSafetyFault',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetRobotBridgeControlMode(self, controlMode, timeout=3, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None):
        # type: (str, float, bool, Optional[types.SetRobotBridgeControlModeParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Args:
            controlMode: The control mode to use, e.g. "Manual".
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 3)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetRobotBridgeControlMode',
            'controlMode': controlMode,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetDynamicObjects(self, timeout=1, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[types.GetDynamicObjectsParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Get a list of dynamically added objects in the scene, from vision detection and physics simulation.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 1)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetDynamicObjects',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ComputeRobotConfigsForGraspVisualization(self, targetname, graspname, robotname=None, toolname=None, unit='mm', timeout=10, dynamicEnvironmentState=None, debuglevel=None, approachoffset=None, departoffsetdir=None, departoffsetintool=None, shadowrobotname=None, shadowrobottoolname=None, **kwargs):
        # type: (str, str, Optional[str], Optional[str], str, float, Optional[types.ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentState], Optional[int], Optional[float], Optional[tuple[float, float, float]], Optional[list[float]], Optional[str], Optional[str], Optional[Any]) -> Optional[Any]
        """
        Returns robot configs for grasp visualization

        Args:
            targetname: Target object's name.
            graspname: Name of the grasp for which to visualize grasps.
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. (Default: "self.toolname") (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            approachoffset: (Default: None)
            departoffsetdir: Departure offset direction. mm (x,y,z) (Default: None)
            departoffsetintool: (Default: None)
            shadowrobotname: (Default: None)
            shadowrobottoolname: (Default: None)
        """
        taskparameters = {
            'command': 'ComputeRobotConfigsForGraspVisualization',
            'targetname': targetname,
            'graspname': graspname,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if unit is not None:
            taskparameters['unit'] = unit
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if approachoffset is not None:
            taskparameters['approachoffset'] = approachoffset
        if departoffsetdir is not None:
            taskparameters['departoffsetdir'] = departoffsetdir
        if departoffsetintool is not None:
            taskparameters['departoffsetintool'] = departoffsetintool
        if shadowrobotname is not None:
            taskparameters['shadowrobotname'] = shadowrobotname
        if shadowrobottoolname is not None:
            taskparameters['shadowrobottoolname'] = shadowrobottoolname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout)

    def ResetCacheTemplates(self, timeout=1, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[types.ResetCacheTemplatesParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Resets any cached templates

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 1)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ResetCacheTemplates',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetRobotBridgeExternalIOPublishing(self, enable, timeout=2, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (bool, float, bool, Optional[types.SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Enables publishing collision data to the robotbridge

        Args:
            enable: If True, collision data will be published to robotbridge.
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 2)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetRobotBridgeExternalIOPublishing',
        }  # type: dict[str, Any]
        taskparameters['enable'] = bool(enable)
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def RestoreSceneInitialState(self, timeout=1, preserverobotdofvalues=1, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, int, Optional[types.RestoreSceneInitialStateParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Restores the scene to the state on the filesystem

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 1)
            preserverobotdofvalues: A Boolean value represented by 0 or 1. (Default: 1)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'RestoreSceneInitialState',
        }  # type: dict[str, Any]
        if preserverobotdofvalues != 1:
            taskparameters['preserverobotdofvalues'] = preserverobotdofvalues
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningStepTest(self, jointName, amplitude, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, float, float, Optional[types.RunMotorControlTuningStepTestParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Runs step response test on specified joint and returns result

        Args:
            jointName: The name of the joint.
            amplitude: The amplitude.
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'RunMotorControlTuningStepTest',
            'jointName': jointName,
            'amplitude': amplitude,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        log.warn('sending taskparameters=%r', taskparameters)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningMaximulLengthSequence(self, jointName, amplitude, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (str, float, float, Optional[types.RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.RunMotorControlTuningMaximulLengthSequenceParametersRobotBridgeConnectionInfo], Optional[list[types.RunMotorControlTuningMaximulLengthSequenceParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Runs maximum length sequence test on specified joint and returns result

        Args:
            jointName: The name of the joint.
            amplitude: The amplitude.
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'RunMotorControlTuningMaximulLengthSequence',
            'jointName': jointName,
            'amplitude': amplitude,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningDecayingChirp(self, jointName, amplitude, freqMax, timeout=120, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (str, float, float, float, Optional[types.RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.RunMotorControlTuningDecayingChirpParametersRobotBridgeConnectionInfo], Optional[list[types.RunMotorControlTuningDecayingChirpParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        runs chirp test on specified joint and returns result

        Args:
            jointName: The name of the joint.
            amplitude: The amplitude.
            freqMax: The maximum frequency in Hz
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 120)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'RunMotorControlTuningDecayingChirp',
            'jointName': jointName,
            'amplitude': amplitude,
            'freqMax': freqMax,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningGaussianImpulse(self, jointName, amplitude, timeout=20, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (str, float, float, Optional[types.RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.RunMotorControlTuningGaussianImpulseParametersRobotBridgeConnectionInfo], Optional[list[types.RunMotorControlTuningGaussianImpulseParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Runs Gaussian Impulse test on specified joint and returns result

        Args:
            jointName: The name of the joint.
            amplitude: The amplitude.
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 20)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'RunMotorControlTuningGaussianImpulse',
            'jointName': jointName,
            'amplitude': amplitude,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningBangBangResponse(self, jointName, amplitude, timeout=60, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (str, float, float, Optional[types.RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.RunMotorControlTuningBangBangResponseParametersRobotBridgeConnectionInfo], Optional[list[types.RunMotorControlTuningBangBangResponseParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Runs bangbang trajectory in acceleration or jerk space and returns result

        Args:
            jointName: The name of the joint.
            amplitude: The amplitude.
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 60)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'RunMotorControlTuningBangBangResponse',
            'jointName': jointName,
            'amplitude': amplitude,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunDynamicsIdentificationTest(self, timeout=4, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (float, Optional[types.RunDynamicsIdentificationTestParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.RunDynamicsIdentificationTestParametersRobotBridgeConnectionInfo], Optional[list[types.RunDynamicsIdentificationTestParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 4)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'RunDynamicsIdentificationTest',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetTimeToRunDynamicsIdentificationTest(self, timeout=10, jointName=None, minJointAngle=None, maxJointAngle=None, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (float, Optional[str], Optional[float], Optional[float], Optional[types.GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.GetTimeToRunDynamicsIdentificationTestParametersRobotBridgeConnectionInfo], Optional[list[types.GetTimeToRunDynamicsIdentificationTestParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            jointName: The name of the joint. (Default: None)
            minJointAngle: The joint angle to start the dynamics identification test at. (Default: None)
            maxJointAngle: The joint angle to finish the dynamics identification test at. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'GetTimeToRunDynamicsIdentificationTest',
        }  # type: dict[str, Any]
        if jointName is not None:
            taskparameters['jointName'] = jointName
        if minJointAngle is not None:
            taskparameters['minJointAngle'] = minJointAngle
        if maxJointAngle is not None:
            taskparameters['maxJointAngle'] = maxJointAngle
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def CalculateTestRangeFromCollision(self, timeout=10, jointName=None, unit='mm', envclearance=None, dynamicEnvironmentState=None, debuglevel=None, robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (float, Optional[str], str, Optional[float], Optional[types.CalculateTestRangeFromCollisionParametersDynamicEnvironmentState], Optional[int], Optional[str], Optional[str], Optional[types.CalculateTestRangeFromCollisionParametersRobotBridgeConnectionInfo], Optional[list[types.CalculateTestRangeFromCollisionParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            jointName: The name of the joint. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            envclearance: Environment clearance in millimeters. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'CalculateTestRangeFromCollision',
        }  # type: dict[str, Any]
        if jointName is not None:
            taskparameters['jointName'] = jointName
        if unit != 'mm':
            taskparameters['unit'] = unit
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetMotorControlParameterSchema(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[types.GetMotorControlParameterSchemaParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Gets motor control parameter schema

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetMotorControlParameterSchema',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetMotorControlParameter(self, jointName, parameterName, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (str, str, float, Optional[types.GetMotorControlParameterParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.GetMotorControlParameterParametersRobotBridgeConnectionInfo], Optional[list[types.GetMotorControlParameterParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Gets motor control parameters as a name-value dict, e.g.: {'J1':{'KP':1}, 'J2':{'KV':2}}

        Args:
            jointName: The name of the joint.
            parameterName:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'GetMotorControlParameter',
            'jointName': jointName,
            'parameterName': parameterName,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetMotorControlParameters(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[types.GetMotorControlParametersParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Gets cached motor control parameters as name-value dict

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetMotorControlParameters',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetMotorControlParameter(self, jointName, parameterName, parameterValue, timeout=10, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotspeed=None, speed=_deprecated, robotaccelmult=None, ionames=None, **kwargs):
        # type: (str, str, Any, float, Optional[types.SetMotorControlParameterParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.SetMotorControlParameterParametersRobotBridgeConnectionInfo], Optional[list[types.SetMotorControlParameterParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[Any], Optional[float], Optional[list[Any]], Optional[Any]) -> Optional[Any]
        """
        Sets motor control parameter

        Args:
            jointName: The name of the joint.
            parameterName: The name of the parameter to set.
            parameterValue: The value to assign to the parameter.
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
        """
        taskparameters = {
            'command': 'SetMotorControlParameter',
            'jointName': jointName,
            'parameterName': parameterName,
            'parameterValue': parameterValue,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if ionames is not None:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def IsProfilingRunning(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None):
        # type: (float, Optional[types.IsProfilingRunningParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Queries if profiling is running on planning

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'IsProfilingRunning',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def StartProfiling(self, clocktype='cpu', timeout=10, dynamicEnvironmentState=None, debuglevel=None):
        # type: (str, float, Optional[types.StartProfilingParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Start profiling planning

        Args:
            clocktype: (Default: 'cpu')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'StartProfiling',
        }  # type: dict[str, Any]
        if clocktype != 'cpu':
            taskparameters['clocktype'] = clocktype
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def StopProfiling(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None):
        # type: (float, Optional[types.StopProfilingParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Stop profiling planning

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'StopProfiling',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ReplaceBodies(self, bodieslist, timeout=10, replaceInfos=None, testLocationName=None, testLocationContainerId=None, removeNamePrefixes=None, removeLocationNames=None, doRemoveOnlyDynamic=None, unit='mm', dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (Any, float, Optional[list[types.ReplaceBodiesParametersReplaceInfosArrayElement]], Optional[str], Optional[str], Optional[list[str]], Optional[list[str]], Optional[bool], str, Optional[types.ReplaceBodiesParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Replaces bodies in the environment with new uris

        Args:
            bodieslist: Used as replaceInfos if the replaceInfos is not defined. Used for backwards compatibility only.
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            replaceInfos: list of dicts with keys: name, uri, containerDynamicProperties (Default: None)
            testLocationName: If specified, will test if the container in this location matches testLocationContainerId, and only execute the replace if it matches and testLocationContainerId is not empty. (Default: None)
            testLocationContainerId: containerId used for testing logic with testLocationName (Default: None)
            removeNamePrefixes: Names of prefixes to match with when removing items (Default: None)
            removeLocationNames: (Default: None)
            doRemoveOnlyDynamic: If True, removes objects that were added through dynamic means such as UpdateObjects/UpdateEnvironmentState. Default: False (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ReplaceBodies',
        }  # type: dict[str, Any]
        taskparameters['replaceInfos'] = bodieslist
        taskparameters['bodieslist'] = bodieslist
        if replaceInfos is not None:
            taskparameters['replaceInfos'] = replaceInfos
        if testLocationName is not None:
            taskparameters['testLocationName'] = testLocationName
        if testLocationContainerId is not None:
            taskparameters['testLocationContainerId'] = testLocationContainerId
        if removeNamePrefixes is not None:
            taskparameters['removeNamePrefixes'] = removeNamePrefixes
        if removeLocationNames is not None:
            taskparameters['removeLocationNames'] = removeLocationNames
        if doRemoveOnlyDynamic is not None:
            taskparameters['doRemoveOnlyDynamic'] = doRemoveOnlyDynamic
        if unit != 'mm':
            taskparameters['unit'] = unit
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetState(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, robotaccelmult=None, callerid=None, stamp=None, command=None, robotspeed=None, **kwargs):
        # type: (float, bool, Optional[types.GetStateParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.GetStateParametersRobotBridgeConnectionInfo], Optional[list[types.GetStateParametersLocationCollisionInfosArrayElement]], Optional[float], Optional[str], Optional[float], Optional[str], Optional[float], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            callerid: (Default: None)
            stamp: The timestamp of when the command was sent, in seconds. (Default: None)
            command: (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
        """
        taskparameters = {
            'command': 'GetState',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if callerid is not None:
            taskparameters['callerid'] = callerid
        if stamp is not None:
            taskparameters['stamp'] = stamp
        if command is not None:
            taskparameters['command'] = command
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def EnsureSyncWithRobotBridge(self, syncTimeStampUS, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (int, float, bool, Optional[types.EnsureSyncWithRobotBridgeParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Ensures that planning has synchronized with robotbridge data that is newer than syncTimeStampUS

        Args:
            syncTimeStampUS: us (microseconds, linux time) of the timestamp
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'EnsureSyncWithRobotBridge',
            'syncTimeStampUS': syncTimeStampUS,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResetCachedRobotConfigurationState(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[types.ResetCachedRobotConfigurationStateParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Resets cached robot configuration (position of the robot) in the planning slave received from slave notification. Need to perform every time robot moved not from the task slaves.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ResetCachedRobotConfigurationState',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def StopMoveThread(self, timeout=10, fireandforget=False, initializeCameraPosition=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[Any], Optional[types.StopMoveThreadParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Stops the move thread. Should track move progress with "statusMove" and "statusDescMove" published messages.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            initializeCameraPosition: (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'StopMoveThread',
        }  # type: dict[str, Any]
        if initializeCameraPosition is not None:
            taskparameters['initializeCameraPosition'] = initializeCameraPosition
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetInstantaneousJointValues(self, objectName, jointvalues, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, unit='mm', **kwargs):
        # type: (str, list[float], float, bool, Optional[types.SetInstantaneousJointValuesParametersDynamicEnvironmentState], Optional[int], str, Optional[Any]) -> Optional[Any]
        """
        Args:
            objectName:
            jointvalues:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
        """
        taskparameters = {
            'command': 'SetInstantaneousJointValues',
            'objectName': objectName,
            'jointvalues': jointvalues,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if unit != 'mm':
            taskparameters['unit'] = unit
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetPackItemPoseInWorld(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, packFormationComputationResult=None, inputPartIndex=None, placeLocationNames=None, unit='mm', **kwargs):
        # type: (float, bool, Optional[types.GetPackItemPoseInWorldParametersDynamicEnvironmentState], Optional[int], Optional[types.PackFormation], Optional[int], Optional[list[str]], str, Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            packFormationComputationResult: A pack formation computed by Mujin. (Default: None)
            inputPartIndex: (Default: None)
            placeLocationNames: (Default: None)
            unit: The unit of the given values. (Default: 'mm')
        """
        taskparameters = {
            'command': 'GetPackItemPoseInWorld',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if packFormationComputationResult is not None:
            taskparameters['packFormationComputationResult'] = packFormationComputationResult
        if inputPartIndex is not None:
            taskparameters['inputPartIndex'] = inputPartIndex
        if placeLocationNames is not None:
            taskparameters['placeLocationNames'] = placeLocationNames
        if unit != 'mm':
            taskparameters['unit'] = unit
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def VisualizePackFormationResult(
        self,
        dynamicEnvironmentState=None,  # type: Optional[types.VisualizePackFormationResultParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        unitMass='kg',  # type: str
        robotname=None,  # type: Optional[str]
        toolname=None,  # type: Optional[str]
        destcontainernames=None,  # type: Optional[list[str]]
        packLocationInfo=None,  # type: Optional[types.VisualizePackFormationResultParametersPackLocationInfo]
        locationName=None,  # type: Optional[str]
        containername=None,  # type: Optional[str]
        packContainerType=None,  # type: Optional[str]
        packInputPartInfos=None,  # type: Optional[list[types.VisualizePackFormationResultParametersPackInputPartInfosArrayElement]]
        packFormationParameters=None,  # type: Optional[types.PackFormationParameters]
        dynamicGoalsGeneratorParameters=None,  # type: Optional[types.VisualizePackFormationResultParametersDynamicGoalsGeneratorParameters]
        constraintToolInfo=None,  # type: Optional[types.ConstraintToolInfo]
        distanceMeasurementInfo=None,  # type: Optional[types.DistanceMeasurementInfo]
        savePackingState=None,  # type: Optional[bool]
        checkObstacleNames=None,  # type: Optional[list[str]]
        targetMinBottomPaddingForInitialTransfer=40,  # type: float
        targetMinSafetyHeightForInitialTransfer=None,  # type: Optional[float]
        saveDynamicGoalGeneratorState=False,  # type: bool
        saveDynamicGoalGeneratorStateFailed=True,  # type: bool
        initializeCameraPosition=None,  # type: Optional[bool]
        packFormationResult=None,  # type: Optional[types.PackFormation]
        indicesToShow=None,  # type: Optional[list[int]]
        maxPlacedIndex=None,  # type: Optional[int]
        destContainerName=None,  # type: Optional[str]
        destcontainername=_deprecated,  # type: Optional[str]
        prefix=None,  # type: Optional[str]
        cameraRelativeToContainerPose=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        isEnabled=None,  # type: Optional[bool]
        isShowLastSolution=None,  # type: Optional[bool]
        isShowInnerContainerCoordinates=None,  # type: Optional[bool]
        normalizePackToEmptyRegion=None  # type: Optional[bool]
    ):
        # type: (...) -> Optional[types.VisualizePackFormationResultReturns]
        """
        Stops the packing computation thread thread started with StartPackFormationComputationThread

        Args:
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
            initializeCameraPosition: Reset camera position (Default: None)
            packFormationResult: A pack formation computed by Mujin. (Default: None)
            indicesToShow: (Default: None)
            maxPlacedIndex: (Default: None)
            destContainerName: (Default: None)
            destcontainername: **deprecated** Use `destContainerName` instead. (Default: None)
            prefix: (Default: None)
            cameraRelativeToContainerPose: If set, places the viewer's camera at this pose relative to the container's bottom face's center. (Default: None)
            isEnabled: (Default: None)
            isShowLastSolution: (Default: None)
            isShowInnerContainerCoordinates: (Default: None)
            normalizePackToEmptyRegion: (Default: None)
        """
        taskparameters = {
            'command': 'VisualizePackFormationResult',
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
        if initializeCameraPosition is not None:
            taskparameters['initializeCameraPosition'] = initializeCameraPosition
        if packFormationResult is not None:
            taskparameters['packFormationResult'] = packFormationResult
        if indicesToShow is not None:
            taskparameters['indicesToShow'] = indicesToShow
        if maxPlacedIndex is not None:
            taskparameters['maxPlacedIndex'] = maxPlacedIndex
        if destContainerName is not None:
            taskparameters['destContainerName'] = destContainerName
        if prefix is not None:
            taskparameters['prefix'] = prefix
        if cameraRelativeToContainerPose is not None:
            taskparameters['cameraRelativeToContainerPose'] = cameraRelativeToContainerPose
        if isEnabled is not None:
            taskparameters['isEnabled'] = isEnabled
        if isShowLastSolution is not None:
            taskparameters['isShowLastSolution'] = isShowLastSolution
        if isShowInnerContainerCoordinates is not None:
            taskparameters['isShowInnerContainerCoordinates'] = isShowInnerContainerCoordinates
        if normalizePackToEmptyRegion is not None:
            taskparameters['normalizePackToEmptyRegion'] = normalizePackToEmptyRegion
        return self.ExecuteCommand(taskparameters, )

    def ClearPackingStateVisualization(self, dynamicEnvironmentState=None, debuglevel=None, containername=None):
        # type: (Optional[types.ClearPackingStateVisualizationParametersDynamicEnvironmentState], Optional[int], Optional[str]) -> Optional[Any]
        """
        Clears packing visualization

        Args:
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            containername: (Default: None)
        """
        taskparameters = {
            'command': 'ClearPackingStateVisualization',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if containername is not None:
            taskparameters['containername'] = containername
        return self.ExecuteCommand(taskparameters, )

