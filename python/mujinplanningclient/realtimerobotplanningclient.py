# -*- coding: utf-8 -*-
# Copyright (C) 2012-2015 MUJIN Inc

# mujin imports
from . import json
from . import planningclient

# logging
import logging
log = logging.getLogger(__name__)


class RealtimeRobotPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the RealtimeRobot task"""

    _robotname = None  # Optional name of the robot selected
    _robotspeed = None  # Speed of the robot, e.g. 0.4
    _robotaccelmult = None  # Current robot accel mult
    _envclearance = None  # Environment clearance in millimeters, e.g. 20
    _robotBridgeConnectionInfo = None  # dict holding the connection info for the robot bridge.

    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(self, robotname=None, robotspeed=None, robotaccelmult=None, envclearance=10.0, robotBridgeConnectionInfo=None, **kwargs):
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
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: realtimerobot
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
        super(RealtimeRobotPlanningClient, self).__init__(**kwargs)

    def GetRobotConnectionInfo(self):
        """ """
        return self._robotBridgeConnectionInfo

    def SetRobotConnectionInfo(self, robotBridgeConnectionInfo):
        """
        Args:
            robotBridgeConnectionInfo:
        """
        self._robotBridgeConnectionInfo = robotBridgeConnectionInfo

    def GetRobotName(self):
        """ """
        return self._robotname

    def SetRobotName(self, robotname):
        """
        Args:
            robotname (str):
        """
        self._robotname = robotname

    def SetRobotSpeed(self, robotspeed):
        """
        Args:
            robotspeed:
        """
        self._robotspeed = robotspeed

    def SetRobotAccelMult(self, robotaccelmult):
        """
        Args:
            robotaccelmult:
        """
        self._robotaccelmult = robotaccelmult

    def ExecuteCommand(self, taskparameters, robotname=None, toolname=None, robotspeed=None, robotaccelmult=None, envclearance=None, timeout=10, fireandforget=False, respawnopts=None, forcereload=False):
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
    # Commands
    #

    def GetJointValues(self, timeout=10, **kwargs):
        """Gets the current robot joint values

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            executetimeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)

        Returns:
            dict: A dictionary with the structure:

                - currentjointvalues (list): Current joint values.
        """
        taskparameters = {
            'command': 'GetJointValues',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def MoveToolLinear(
        self,
        goaltype,
        goals,
        toolname=None,
        timeout=10,
        robotspeed=None,
        **kwargs
    ):
        """Moves the tool linear

        Args:
            goaltype (str): Type of the goal, e.g. translationdirection5d
            goals (list[float]): Flat list of goals, e.g. two 5D ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]
            toolname (str, optional): Name of the manipulator. Default: self.toolname
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            workmaxdeviationangle (float, optional): How much the tool tip can rotationally deviate from the linear path. In deg.
            workspeed (list[float], optional): [anglespeed, transspeed] in deg/s and mm/s
            workaccel (list[float], optional): [angleaccel, transaccel] in deg/s^2 and mm/s^2
            worksteplength (float, optional): Discretization for planning MoveHandStraight, in seconds.
            plannername (str, optional):
            numspeedcandidates (int, optional): If speed/accel are not specified, the number of candiates to consider
            workminimumcompletetime (float, optional): **deprecated** Unused. Set to trajduration - 0.016s. EMU_MUJIN example requires at least this much
            workminimumcompleteratio (float, optional): **deprecated** Unused. In case the duration of the trajectory is now known, can specify in terms of [0,1]. 1 is complete everything.
            workignorefirstcollisionee (float, optional): time, necessary in case initial is in collision, has to be multiples of step length?
            workignorelastcollisionee (float, optional): time, necessary in case goal is in collision, has to be multiples of step length?
            workignorefirstcollision (float, optional):
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
            ignoreGrabbingTarget (bool, optional):
            currentlimitratios (list[float], optional): The joints' current limit ratios.
            instobjectname (str, optional): If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position.
            ikparamname (str, optional): If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position.
            execute (int, optional): If 1, execute the motion.
            moveStraightParams (dict, optional): A set of parameters defining how the robot behaves during linear motions.
        """
        taskparameters = {
            'command': 'MoveToolLinear',
            'goaltype': goaltype,
            'goals': goals,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, toolname=toolname, timeout=timeout)

    def MoveToHandPosition(
        self,
        goaltype,
        goals,
        toolname=None,
        envclearance=None,
        closegripper=0,
        robotspeed=None,
        robotaccelmult=None,
        timeout=10,
        **kwargs
    ):
        """Computes the inverse kinematics and moves the manipulator to any one of the goals specified.

        Args:
            goaltype (str): Type of the goal, e.g. translationdirection5d
            goals (list[float]): Flat list of goals, e.g. two 5d ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            envclearance (float, optional): Environment clearance in millimeters.
            closegripper (int, optional): Whether to close gripper once the goal is reached. Boolean value represented by 0 or 1. (Default: 0)
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            speed: **deprecated** Use robotspeed instead.
            ionames (list, optional): A list of IO names to read/write
            constraintToolDirection (list[float], optional):
            departOffsetDir (list[float], optional): Direction in which to apply the offset when departing from the pick/place operation.
            departMinimumCompleteRatio (float, optional): The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0.
            departOffsetAwayFromGravity (float, optional): The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir.
            trajname (str, optional):
            disablebodies (bool, optional):
            ignoreGrabbingTarget (bool, optional):
            jointthresh (float, optional):
            jitter (float, optional):
            execute (int, optional): If 1, execute the motion.
            executionFilterFactor (float, optional):
            filtertraj (bool, optional):
            currentlimitratios (list[float], optional): The joints' current limit ratios.
            goalJointThreshold (list[float], optional): Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time.
            goalWorkspaceThreshold (float, optional): Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful.
            calibrategripper (bool, optional):
            departAccel (float, optional):
            maxManipAccel (float, optional):
            maxJitterLinkDist (float, optional): mm.
            pathPlannerParameters (dict, optional):
            moveStraightParams (dict, optional): A set of parameters defining how the robot behaves during linear motions.
            forceTorqueBasedEstimatorParameters (dict, optional): A set of parameters for force-torque based estimation.
            dynamicEnvironmentState (list[dict], optional): The dynamic objects in the environment that is to be used for planning/executing the task. A list of bodies.
            savetrajectorylog (bool, optional): If True, will save the commanded (input) trajectories before they are executed
            saveRobotFeedbackLog (bool, optional): If True, will tell robotbridge to save trajectory files
            loadRobotFeedbackLog (bool, optional): If True, will tell robotbridge to load the robot feedback log after trajectory ends
            saveConcatenateTrajectoryLog (bool, optional): If True, will save trajectories used for inputs of concatenate trajectory functions
            saveFilterTrajectoryLog (bool, optional): If True, will save trajectories used for filtering, such as SmartFilter
            executionConnectingTrajReverseMult (float, optional): Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally.
            executionReverseRecoveryDistance (float, optional): Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm).
            debuglevel (int, optional): Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE.
            jittererParameters (dict, optional):
            minimumgoalpaths (int, optional): Number of solutions the planner must provide before it is allowed to finish.
            chuckgripper (bool, optional):
            instobjectname (str, optional): If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position.
            ikparamname (str, optional): If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position.
            ikparamoffset (list[float], optional):
            smootherParameters (dict, optional):
        """
        taskparameters = {
            'command': 'MoveToHandPosition',
            'goaltype': goaltype,
            'goals': goals,
            'closegripper': closegripper,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, robotaccelmult=robotaccelmult, envclearance=envclearance, toolname=toolname, timeout=timeout)

    def UpdateObjects(
        self,
        envstate,
        targetname=None,
        state=None,
        unit='mm',
        timeout=10,
        **kwargs
    ):
        """Updates objects in the scene with the envstate

        Args:
            envstate: A list of dictionaries for each instance object in world frame. Quaternion is specified in w,x,y,z order. e.g. [{'name': 'target_0', 'translation_': [1,2,3], 'quat_': [1,0,0,0], 'object_uri':'mujin:/asdfas.mujin.dae'}, {'name': 'target_1', 'translation_': [2,2,3], 'quat_': [1,0,0,0]}]
            targetname (str, optional): Name of the target object
            state (dict, optional):
            unit (str, optional): The unit of the given values. (Default: 'mm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            locationName (str, optional): Name of the location to update.
            locationContainerId:
            imageStartTimeStampMS (int, optional):
            callerid (str, optional): The name of the caller (only used internally)
            object_uri (str, optional): Same as objectname, but in a Mujin URI format, e.g.: mujin:/OBJECTNAME.mujin.dae
            detectionResultState (dict, optional): Information about the detected objects (received from detectors)
            targetUpdateNamePrefix (str, optional):
            cameranames (list[str], optional):
            countOverlappingPoints (bool, optional):
            overlapUpAxis (list[float], optional): Values of ['x', 'y', 'z'] correspondingly.
            zthresholdmult (float, optional):
            addUnpickableRegionAcrossShortEdgeDist (bool, optional):
            sizeRoundUp (bool, optional): If False, then round down. (Default: True)
            sizePrecisionXYZ (list[float], optional): mm (x,y,z) for rounding up incoming boxes from the detector. This allows previous grasping models to be cached and re-used since the sizes will be multiples of the current precision.
            points (list[float], optional): The point cloud passed in along with the detection results. Used in selective cases to count point overlap of random box.
            pointsize (float, optional): Size of points in the point cloud.
            pointcloudid (str, optional):
            containerName (str, optional): Name of the container to update. Requires locationName to be set. If containerName is empty, will use the container in locationName.
            isFromStateSlaveNotify (bool, optional):
            imageEndTimeStampMS (int, optional):
            pointCloudSensorTimeStampMS (int, optional):
            belowBoxOverlap (float, optional): mm, Threshold on how much to ignore the relative heights of two neighboring targets to determine if the candidate is *below* the current pickup target. Positive value the pickup target is allowed to be under the other non-pickup targets by this amount, and still be pickable. When two targets are deemed to be overlapping on the face orthogonal to overlapUpAxis based on neighOverlapThresh, then check the heights of the targets to make sure that one target is really above the other. Sometimes detection error can cause two targets on the same height to be overlapped a little, but that doesn't mean that one target is on top of the other. (Default: 0)
            ignoreOverlapPointsFromWall (float, optional): mm, distance from the container inner walls within which pointcloud points do not count towards overlapping points (Default: 0)
            ignoreOverlapPointsFromNearbyTargets (float, optional): mm, amount of target extents reduction when counting the number of overlapping pointcloud points. This is so that pointcloud near the edges of the target (can come from noises from nearby targets, for example) can be ignored. (Default: 0)
            castPointCloudShadowFromCamera (bool, optional): If True, bottom parts of pointcloud obstacle are generated by casting shadow from camera. otherwise, vertical down (-z).
            pointsProjectedDirection (list[float], optional): The negative direction in which the points were projected when creating the obstacles. If specified, then take into account when computing the overlap. When container up is +Z, then pointsProjectedDirection will be (0,0,1).
            randomBoxOrigin (list[float], optional): Specifies where to place the origin of the incoming box detections. By default, this is [0,0,1], which means the origin will be at the center of the +Z (top) face.
            rollStepDegree (float, optional): Step of 6D grasp rotation around z axis in degrees, defaults to 45 degrees. (Default: 90)
            clampToContainer (bool, optional): If True, crop to container dimensions.
            medianFilterHalfSize (float, optional): If clampcontainer is True, this is used for filtering.
            useEmptyRegionForCropping (bool, optional): If clampcontainer is True, this is used for filtering.
            ioSignalsInfo (dict, optional): Struct for dictating if any IO signals should be written on receiving detection results
            addPointOffsetInfo (dict, optional): Special offsets from pointcloud
        """
        taskparameters = {
            'command': 'UpdateObjects',
            'envstate': envstate,
            'unit': unit,
        }
        if targetname is not None:
            taskparameters['object_uri'] = u'mujin:/%s.mujin.dae' % (targetname)
        taskparameters.update(kwargs)
        if state is not None:
            taskparameters['state'] = json.dumps(state)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def Grab(self, targetname, toolname=None, timeout=10, **kwargs):
        """Grabs an object with tool

        Args:
            targetname (str): Name of the target object
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'Grab',
            'targetname': targetname,
        }
        if toolname is not None:
            taskparameters['toolname'] = toolname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def Release(self, targetname, timeout=10, **kwargs):
        """Releases a grabbed object.

        Args:
            targetname (str): Name of the target object
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'Release',
            'targetname': targetname,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetGrabbed(self, timeout=10, **kwargs):
        """Gets the names of the objects currently grabbed

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)

        Returns:
            dict: A dictionary with the structure:

                - names (list[str]): Names of the grabbed object.
        """
        taskparameters = {
            'command': 'GetGrabbed',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetTransform(self, targetname, connectedBodyName='', linkName='', geometryName='', geometryPk='', unit='mm', timeout=10, **kwargs):
        """Gets the transform of an object

        Args:
            targetname (str): OpenRave Kinbody name
            connectedBodyName (str, optional): OpenRave connected body name (Default: '')
            linkName (str, optional): OpenRave link name (Default: '')
            geometryName (str, optional): OpenRave geometry id name (Default: '')
            geometryPk (str, optional): OpenRave geometry primary key (pk) (Default: '')
            unit (str, optional): The unit of the given values. (Default: 'mm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)

        Returns:
            dict: Transform of the object.

                A dictionary with the structure:

                - translation (list[float]): E.g. [100,200,300]
                - rotationmat (list[list[float]]): E.g. [[1,0,0],[0,1,0],[0,0,1]]
                - quaternion (list[float]): E.g. [1,0,0,0]
        """
        taskparameters = {
            'command': 'GetTransform',
            'targetname': targetname,
            'unit': unit,
        }
        if connectedBodyName is not None:
            taskparameters['connectedBodyName'] = connectedBodyName
        if linkName is not None:
            taskparameters['linkName'] = linkName
        if geometryName is not None:
            taskparameters['geometryName'] = geometryName
        if geometryPk is not None:
            taskparameters['geometryPk'] = geometryPk
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetLinkParentInfo(self, objectName, linkName, unit='mm', timeout=10, **kwargs):
        """Gets the parent link transform and name.

        Args:
            objectName (str): OpenRave Kinbody name.
            linkName (str): OpenRave link name.
            unit (str, optional): The unit of the given values. (Default: 'mm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)

        Returns:
            dict: A dictionary with the structure:

                - name (str)
                - translation (list)
                - rotationmat (list)
                - quaternion (list)
        """
        taskparameters = {
            'command': 'GetLinkParentInfo',
            'objectName': objectName,
            'linkName': linkName,
            'unit': unit,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetTransform(self, targetname, translation, unit='mm', rotationmat=None, quaternion=None, timeout=10, **kwargs):
        """Sets the transform of an object. Rotation can be specified by either quaternion or rotation matrix.

        Args:
            targetname (str): Name of the target object
            translation (list[float]): List of x,y,z values of the object in millimeters.
            unit (str, optional): The unit of the given values. (Default: 'mm')
            rotationmat (list[float], optional): List specifying the rotation matrix in row major format, e.g. [1,0,0,0,1,0,0,0,1]
            quaternion (list[float], optional): List specifying the quaternion in w,x,y,z format, e.g. [1,0,0,0].
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'SetTransform',
            'targetname': targetname,
            'translation': translation,
            'unit': unit,
        }
        if rotationmat is not None:
            taskparameters['rotationmat'] = rotationmat
        if quaternion is not None:
            taskparameters['quaternion'] = quaternion
        taskparameters.update(kwargs)
        if rotationmat is None and quaternion is None:
            taskparameters['quaternion'] = [1, 0, 0, 0]
            log.warn('No rotation is specified. Using identity quaternion.')
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetOBB(self, targetname, unit='mm', timeout=10, linkname=None, **kwargs):
        """Get the oriented bounding box (OBB) of object.

        Args:
            targetname (str): Name of the object
            unit (str, optional): The unit of the given values. (Default: 'mm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            linkname (str, optional): Name of link to use for OBB. If not specified, uses entire target.

        Returns:
            dict: A dictionary describing the OBB of the object.

                A dictionary with the structure:

                - extents
                - boxLocalTranslation
                - originalBodyTranslation
                - quaternion
                - rotationmat
                - translation
        """
        taskparameters = {
            'command': 'GetOBB',
            'targetname': targetname,
            'unit': unit,
        }
        if linkname is not None:
            taskparameters['linkname'] = linkname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetInnerEmptyRegionOBB(self, targetname, linkname=None, unit='mm', timeout=10, **kwargs):
        """Get the inner empty oriented bounding box (OBB) of a container.

        Args:
            targetname (str): Name of the object
            linkname (str, optional): Name of link to use for OBB. If not specified, uses entire target.
            unit (str, optional): The unit of the given values. (Default: 'mm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)

        Returns:
            dict: A dictionary describing the OBB of the object.

                A dictionary with the structure:

                - extents
                - boxLocalTranslation
                - originalBodyTranslation
                - quaternion
                - rotationmat
                - translation
        """
        taskparameters = {
            'command': 'GetInnerEmptyRegionOBB',
            'targetname': targetname,
            'unit': unit,
        }
        if linkname is not None:
            taskparameters['linkname'] = linkname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetInstObjectAndSensorInfo(self, instobjectnames=None, sensornames=None, unit='mm', timeout=10, **kwargs):
        """Returns information about the inst objects and sensors that are a part of those inst objects.

        Args:
            instobjectnames (list[str], optional):
            sensornames (list[str], optional):
            unit (str, optional): The unit of the given values. (Default: 'mm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            ignoreMissingObjects (bool, optional): If False, will raise an error if the object is not found in the scene. Default: True.
        """
        taskparameters = {
            'command': 'GetInstObjectAndSensorInfo',
            'unit': unit,
        }
        if instobjectnames is not None:
            taskparameters['instobjectnames'] = instobjectnames
        if sensornames is not None:
            taskparameters['sensornames'] = sensornames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetInstObjectInfoFromURI(self, instobjecturi=None, unit='mm', timeout=10, **kwargs):
        """Opens a URI and returns info about the internal/external and geometry info from it.

        Args:
            instobjecturi (str, optional):
            unit (str, optional): The unit of the given values. (Default: 'mm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            instobjectpose (list[float], optional): Pose to be assigned to the retrieved object. 7-element list
        """
        taskparameters = {
            'command': 'GetInstObjectInfoFromURI',
            'unit': unit,
        }
        if instobjecturi is not None:
            taskparameters['objecturi'] = instobjecturi
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetAABB(self, targetname, unit='mm', timeout=10, linkname=None, **kwargs):
        """Gets the axis-aligned bounding box (AABB) of an object.

        Args:
            targetname (str): Name of the object
            unit (str, optional): The unit of the given values. (Default: 'mm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            linkname (str, optional): Name of link to use for the AABB. If not specified, uses entire target.

        Returns:
            dict: AABB of the object.

                A dictionary with the structure:

                - pos (list): E.g. [1000,400,100]
                - extents (list): E.g. [100,200,50]
        """
        taskparameters = {
            'command': 'GetAABB',
            'targetname': targetname,
            'unit': unit,
        }
        if linkname is not None:
            taskparameters['linkname'] = linkname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetLocationTracking(self, timeout=10, fireandforget=False, **kwargs):
        """Resets the tracking of specific containers

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server.
            cycleIndex: The cycle index to track the locations for
            locationReplaceInfos: A dict that should have the keys: name, containerDynamicProperties, rejectContainerIds, uri, pose, cycleIndex
            removeLocationNames (list[str], optional):
            minRobotBridgeTimeStampUS (int, optional): The minimum expected time stamp.
            dynamicObstacleBaseName (str, optional):
            targetUpdateBaseName (str, optional):
            ioSignalsInfo (dict, optional): Struct for dictating if any IO signals should be written on receiving detection results
            unit (str, optional): The unit of the given values. (Default: 'mm')
        """
        taskparameters = {
            'command': 'SetLocationTracking',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResetLocationTracking(self, timeout=10, fireandforget=False, **kwargs):
        """Resets tracking updates for locations

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            resetAllLocations (bool, optional): If True, then will reset all the locations
            resetLocationName (str, optional): Resets only the location with matching name
            resetLocationNames (list[str], optional): Resets only locations with matching name
            checkIdAndResetLocationName: (locationName, containerId) - only reset the location if the container id matches

        Returns:
            dict: clearedLocationNames

                An unstructured dictionary.
        """
        taskparameters = {
            'command': 'ResetLocationTracking',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)['clearedLocationNames']

    def GetLocationTrackingInfos(self, timeout=10, fireandforget=False, **kwargs):
        """Gets the active tracked locations

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)

        Returns:
            dict: activeLocationTrackingInfos

                An unstructured dictionary.
        """
        taskparameters = {
            'command': 'GetLocationTrackingInfos',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)['activeLocationTrackingInfos']

    def UpdateLocationContainerIdType(self, locationName, containerName, containerId, containerType, trackingCycleIndex=None, timeout=10, fireandforget=False, **kwargs):
        """Resets the tracking of specific containers

        Args:
            locationName (str): Name of the location the container is in
            containerName (str): Name of the container
            containerId (str): ID of the container
            containerType (str): Type of the container
            trackingCycleIndex: If specified, then the cycle with same cycleIndex will update location tracking in the same call.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            unit (str, optional): The unit of the given values. (Default: 'mm')
        """
        taskparameters = {
            'command': 'UpdateLocationContainerIdType',
            'locationName': locationName,
            'containerName': containerName,
            'containerId': containerId,
            'containerType': containerType,
        }
        if trackingCycleIndex is not None:
            taskparameters['trackingCycleIndex'] = trackingCycleIndex
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResetLocationTrackingContainerId(self, locationName, checkContainerId, timeout=10, fireandforget=False, **kwargs):
        """Resets the containerId of self._activeLocationTrackingInfos if it matches checkContainerId.

        Args:
            locationName (str): The name of the location that may be reset.
            checkContainerId: If checkContainerId is specified and not empty and it matches the current containerId of the tracking location, then reset the current tracking location
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ResetLocationTrackingContainerId',
            'locationName': locationName,
            'checkContainerId': checkContainerId,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def RemoveObjectsWithPrefix(self, prefix=None, removeNamePrefixes=None, timeout=10, fireandforget=False, removeLocationNames=None, **kwargs):
        """Removes objects with prefix.

        Args:
            prefix (str, optional): **deprecated**
            removeNamePrefixes (list[str], optional): Names of prefixes to match with when removing items
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            removeLocationNames (list[str], optional):
            doRemoveOnlyDynamic (bool, optional): If True, removes objects that were added through dynamic means such as UpdateObjects/UpdateEnvironmentState. Default: False
            locationName (str, optional): Name of the location to update.
            locationContainerId:
            imageStartTimeStampMS (int, optional):
            callerid (str, optional): The name of the caller (only used internally)

        Returns:
            dict: A dictionary with the structure:

                - removedBodyNames: Key for the removed object names
        """
        taskparameters = {
            'command': 'RemoveObjectsWithPrefix',
        }
        taskparameters.update(kwargs)
        if prefix is not None:
            log.warn('prefix is deprecated')
            taskparameters['prefix'] = prefix
        if removeNamePrefixes is not None:
            taskparameters['removeNamePrefixes'] = removeNamePrefixes
        if removeLocationNames is not None:
            taskparameters['removeLocationNames'] = removeLocationNames
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetTrajectoryLog(self, timeout=10, **kwargs):
        """Gets the recent trajectories executed on the binpicking server. The internal server keeps trajectories around for 10 minutes before clearing them.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            startindex (int, optional): Start of the trajectory to get. If negative, will start counting from the end. For example, -1 is the last element, -2 is the second to last. Default: 0
            num (int, optional): Number of trajectories from startindex to return. If 0, will return all the trajectories starting from startindex. Default: 0
            includejointvalues (bool, optional): If True, will include timedjointvalues. If False, will just give back the trajectories. (Default: False)
            saverawtrajectories (bool, optional): If True, will save the raw trajectories.

        Returns:
            dict: A dictionary with the structure:

                - total (int)
                - trajectories (list[dict]): Contains a dictionary with the structure:

                    - timestarted (int)
                    - name (str)
                    - numpoints (int)
                    - duration (float)
                    - timedjointvalues (list): A list of joint values and the trajectory time. For a 3DOF robot sampled at 0.008s, this is: [J1, J2, J3, 0, J1, J2, J3, 0.008, J1, J2, J3, 0.016, ...]
        """
        taskparameters = {
            'command': 'GetTrajectoryLog',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ChuckGripper(self, robotname=None, grippername=None, timeout=10, **kwargs):
        """Chucks the manipulator

        Args:
            robotname (str, optional): Name of the robot
            grippername (str, optional): Name of the gripper.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'ChuckGripper',
            'grippername': grippername,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def UnchuckGripper(self, robotname=None, grippername=None, timeout=10, **kwargs):
        """Unchucks the manipulator and releases the target

        Args:
            robotname (str, optional): Name of the robot
            grippername (str, optional): Name of the gripper.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            targetname (str, optional): Name of the target object.
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            pulloutdist (float, optional): Distance to move away along the tool direction after releasing.
            deletetarget (int, optional): If 1, removes the target object from the environment after releasing. (Default: 1)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'UnchuckGripper',
            'grippername': grippername,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def CalibrateGripper(self, robotname=None, grippername=None, timeout=10, fireandforget=False, **kwargs):
        """Goes through the gripper calibration procedure

        Args:
            robotname (str, optional): Name of the robot
            grippername (str, optional): Name of the gripper.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'CalibrateGripper',
            'grippername': grippername,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def StopGripper(self, robotname=None, grippername=None, timeout=10, fireandforget=False, **kwargs):
        """
        Args:
            robotname (str, optional): Name of the robot
            grippername (str, optional): Name of the gripper.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'StopGripper',
            'grippername': grippername,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def MoveGripper(self, grippervalues, robotname=None, grippername=None, timeout=10, fireandforget=False, **kwargs):
        """Moves the chuck of the manipulator to a given value.

        Args:
            grippervalues (list[float]): Target value(s) of the chuck.
            robotname (str, optional): Name of the robot
            grippername (str, optional): Name of the gripper.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'MoveGripper',
            'grippername': grippername,
            'grippervalues': grippervalues,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ExecuteRobotProgram(self, robotProgramName, robotname=None, timeout=10, fireandforget=False, **kwargs):
        """Execute a robot specific program by name

        Args:
            robotProgramName (str):
            robotname (str, optional): Name of the robot
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'ExecuteRobotProgram',
            'robotProgramName': robotProgramName,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SaveScene(self, timeout=10, **kwargs):
        """Saves the current scene to file

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            filename (str, optional): e.g. /tmp/testscene.mujin.dae, if not specified, it will be saved with an auto-generated filename
            preserveexternalrefs (bool, optional): If True, any bodies that are currently being externally referenced from the environment will be saved as external references.
            externalref (str, optional): If '*', then each of the objects will be saved as externally referencing their original filename. Otherwise will force saving specific bodies as external references.
            saveclone: **deprecated** If 1, will save the scenes for all the cloned environments
            saveReferenceUriAsHint (bool, optional): If True, use save the reference uris as referenceUriHint so that webstack does not get confused and deletes content

        Returns:
            dict: The filename the scene is saved to, in a json dictionary, e.g. {'filename': '2013-11-01-17-10-00-UTC.dae'}

                An unstructured dictionary.
        """
        taskparameters = {
            'command': 'SaveScene',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SaveGripper(self, timeout=10, robotname=None, **kwargs):
        """Separate gripper from a robot in a scene and save it.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            robotname (str, optional): Name of the robot waiting to extract the hand from.
            filename (str, optional): File name to save on the file system. e.g. /tmp/robotgripper/mujin.dae
            manipname (str, optional): Name of the manipulator.
            unit (str, optional): The unit of the given values. (Default: 'mm')
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
        """
        taskparameters = {
            'command': 'SaveGripper',
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def MoveZac(self, linkName, robotname=None, robotspeed=None, robotaccelmult=None, execute=1, startJointConfigurationStates=None, envclearance=None, timeout=100, **kwargs):
        taskparameters = {
            'command': 'MoveAndSampleForceTorqueSensorDataForCalibration',
            'execute': execute,
            'ftSensorLinkName': linkName
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname

        if envclearance is not None:
            taskparameters['envclearance'] = envclearance

        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def MoveJointsToJointConfigurationStates(self, jointConfigurationStates, robotname=None, robotspeed=None, robotaccelmult=None, execute=1, startJointConfigurationStates=None, envclearance=None, timeout=10, **kwargs):

        """Moves the robot to desired joint angles specified in jointStates

        Args:
            jointConfigurationStates:
            robotname (str, optional): Name of the robot
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            execute (int, optional): If 1, execute the motion. (Default: 1)
            startJointConfigurationStates:
            envclearance (float, optional): Environment clearance in millimeters.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            jointStates (list, optional): List[{'jointName':str, 'jointValue':float}]
            jointindices (list[int], optional): List of corresponding joint indices, default is range(len(jointvalues))
            unit (str, optional): The unit of the given values. (Default: 'mm')
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            speed: **deprecated** Use robotspeed instead.
            ionames (list, optional): A list of IO names to read/write
            constraintToolDirection (list[float], optional):
            departOffsetDir (list[float], optional): Direction in which to apply the offset when departing from the pick/place operation.
            departMinimumCompleteRatio (float, optional): The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0.
            departOffsetAwayFromGravity (float, optional): The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir.
            trajname (str, optional):
            disablebodies (bool, optional):
            ignoreGrabbingTarget (bool, optional):
            jointthresh (float, optional):
            jitter (float, optional):
            executionFilterFactor (float, optional):
            filtertraj (bool, optional):
            currentlimitratios (list[float], optional): The joints' current limit ratios.
            goalJointThreshold (list[float], optional): Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time.
            goalWorkspaceThreshold (float, optional): Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful.
            calibrategripper (bool, optional):
            departAccel (float, optional):
            maxManipAccel (float, optional):
            maxJitterLinkDist (float, optional): mm.
            pathPlannerParameters:
            moveStraightParams (dict, optional): A set of parameters defining how the robot behaves during linear motions.
            forceTorqueBasedEstimatorParameters (dict, optional): A set of parameters for force-torque based estimation.
            dynamicEnvironmentState (list[dict], optional): The dynamic objects in the environment that is to be used for planning/executing the task. A list of bodies.
            savetrajectorylog (bool, optional): If True, will save the commanded (input) trajectories before they are executed
            saveRobotFeedbackLog (bool, optional): If True, will tell robotbridge to save trajectory files
            loadRobotFeedbackLog (bool, optional): If True, will tell robotbridge to load the robot feedback log after trajectory ends
            saveConcatenateTrajectoryLog (bool, optional): If True, will save trajectories used for inputs of concatenate trajectory functions
            saveFilterTrajectoryLog (bool, optional): If True, will save trajectories used for filtering, such as SmartFilter
            executionConnectingTrajReverseMult (float, optional): Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally.
            executionReverseRecoveryDistance (float, optional): Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm).
            debuglevel (int, optional): Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE.
            jittererParameters (dict, optional):
        """
        taskparameters = {
            'command': 'MoveJointsToJointConfigurationStates',
            'goalJointConfigurationStates': jointConfigurationStates,
            'execute': execute,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname

        if envclearance is not None:
            taskparameters['envclearance'] = envclearance

        if startJointConfigurationStates is not None:
            taskparameters['startJointConfigurationStates'] = startJointConfigurationStates

        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def MoveJoints(
        self,
        jointvalues,
        robotJointNames=None,
        robotname=None,
        robotspeed=None,
        robotaccelmult=None,
        execute=1,
        startvalues=None,
        envclearance=None,
        timeout=10,
        **kwargs
    ):
        """Moves the robot to desired joint angles specified in jointvalues

        Args:
            jointvalues (list[float]): List of joint values to move to. Use goaljoints instead.
            robotJointNames (str, optional): List of corresponding joint names for jointvalues.
            robotname (str, optional): Name of the robot
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            execute (int, optional): If 1, execute the motion. (Default: 1)
            startvalues (list[float], optional): The robot joint values to start the motion from.
            envclearance (float, optional): Environment clearance in millimeters.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            jointindices (list[int], optional): List of corresponding joint indices, default is range(len(jointvalues))
            robotProgramName (str, optional):
            goaljoints (list[float], optional): List of joint values to move to.
            unit (str, optional): The unit of the given values. (Default: 'mm')
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            speed: **deprecated** Use robotspeed instead.
            ionames (list, optional): A list of IO names to read/write
            constraintToolDirection (list[float], optional):
            departOffsetDir (list[float], optional): Direction in which to apply the offset when departing from the pick/place operation.
            departMinimumCompleteRatio (float, optional): The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0.
            departOffsetAwayFromGravity (float, optional): The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir.
            trajname (str, optional):
            disablebodies (bool, optional):
            ignoreGrabbingTarget (bool, optional):
            jointthresh (float, optional):
            jitter (float, optional):
            executionFilterFactor (float, optional):
            filtertraj (bool, optional):
            currentlimitratios (list[float], optional): The joints' current limit ratios.
            goalJointThreshold (list[float], optional): Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time.
            goalWorkspaceThreshold (float, optional): Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful.
            calibrategripper (bool, optional):
            departAccel (float, optional):
            maxManipAccel (float, optional):
            maxJitterLinkDist (float, optional): mm.
            pathPlannerParameters:
            moveStraightParams (dict, optional): A set of parameters defining how the robot behaves during linear motions.
            forceTorqueBasedEstimatorParameters (dict, optional): A set of parameters for force-torque based estimation.
            dynamicEnvironmentState (list[dict], optional): The dynamic objects in the environment that is to be used for planning/executing the task. A list of bodies.
            savetrajectorylog (bool, optional): If True, will save the commanded (input) trajectories before they are executed
            saveRobotFeedbackLog (bool, optional): If True, will tell robotbridge to save trajectory files
            loadRobotFeedbackLog (bool, optional): If True, will tell robotbridge to load the robot feedback log after trajectory ends
            saveConcatenateTrajectoryLog (bool, optional): If True, will save trajectories used for inputs of concatenate trajectory functions
            saveFilterTrajectoryLog (bool, optional): If True, will save trajectories used for filtering, such as SmartFilter
            executionConnectingTrajReverseMult (float, optional): Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally.
            executionReverseRecoveryDistance (float, optional): Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm).
            debuglevel (int, optional): Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE.
            jittererParameters (dict, optional):
        """
        taskparameters = {
            'command': 'MoveJoints',
            'goaljoints': list(jointvalues),
            'execute': execute,
        }
        if robotJointNames is not None:
            taskparameters['robotJointNames'] = robotJointNames
        if robotname is not None:
            taskparameters['robotname'] = robotname
        
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        
        if startvalues is not None:
            taskparameters['startvalues'] = list(startvalues)
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def MoveJointsToPositionConfiguration(
        self,
        positionConfigurationName=None,
        positionConfigurationCandidateNames=None,
        robotname=None,
        robotspeed=None,
        robotaccelmult=None,
        execute=1,
        startvalues=None,
        envclearance=None,
        timeout=10,
        **kwargs
    ):
        """Moves the robot to desired position configuration specified in positionConfigurationName

        Args:
            positionConfigurationName (str, optional): If specified, the name of position configuration to move to. If it does not exist, will raise an error.
            positionConfigurationCandidateNames (list[str], optional): If specified, goes to the first position that is defined for the robot. If no positions exist, returns without moving the robot.
            robotname (str, optional): Name of the robot
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            execute (int, optional): If 1, execute the motion. (Default: 1)
            startvalues (list[float], optional): The robot joint values to start the motion from.
            envclearance (float, optional): Environment clearance in millimeters.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            speed: **deprecated** Use robotspeed instead.
            ionames (list, optional): A list of IO names to read/write
            constraintToolDirection (list[float], optional):
            departOffsetDir (list[float], optional): Direction in which to apply the offset when departing from the pick/place operation.
            departMinimumCompleteRatio (float, optional): The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0.
            departOffsetAwayFromGravity (float, optional): The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir.
            trajname (str, optional):
            disablebodies (bool, optional):
            ignoreGrabbingTarget (bool, optional):
            jointthresh (float, optional):
            jitter (float, optional):
            executionFilterFactor (float, optional):
            filtertraj (bool, optional):
            currentlimitratios (list[float], optional): The joints' current limit ratios.
            goalJointThreshold (list[float], optional): Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time.
            goalWorkspaceThreshold (float, optional): Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful.
            calibrategripper (bool, optional):
            departAccel (float, optional):
            maxManipAccel (float, optional):
            maxJitterLinkDist (float, optional): mm.
            pathPlannerParameters:
            moveStraightParams (dict, optional): A set of parameters defining how the robot behaves during linear motions.
            forceTorqueBasedEstimatorParameters (dict, optional): A set of parameters for force-torque based estimation.
            dynamicEnvironmentState (list[dict], optional): The dynamic objects in the environment that is to be used for planning/executing the task. A list of bodies.
            savetrajectorylog (bool, optional): If True, will save the commanded (input) trajectories before they are executed
            saveRobotFeedbackLog (bool, optional): If True, will tell robotbridge to save trajectory files
            loadRobotFeedbackLog (bool, optional): If True, will tell robotbridge to load the robot feedback log after trajectory ends
            saveConcatenateTrajectoryLog (bool, optional): If True, will save trajectories used for inputs of concatenate trajectory functions
            saveFilterTrajectoryLog (bool, optional): If True, will save trajectories used for filtering, such as SmartFilter
            executionConnectingTrajReverseMult (float, optional): Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally.
            executionReverseRecoveryDistance (float, optional): Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm).
            debuglevel (int, optional): Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE.
            jittererParameters (dict, optional):
            startJointConfigurationStates (list[dict], optional): List of dicts for each joint.
            robotProgramName (str, optional):

        Returns:
            dict: Dictionary with keys goalPositionName and values goalConfiguration

                An unstructured dictionary.
        """
        taskparameters = {
            'command': 'MoveJointsToPositionConfiguration',
            'execute': execute,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if positionConfigurationName:
            taskparameters['positionConfigurationName'] = positionConfigurationName
        if positionConfigurationCandidateNames:
            taskparameters['positionConfigurationCandidateNames'] = positionConfigurationCandidateNames
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if startvalues is not None:
            taskparameters['startvalues'] = list(startvalues)
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def GetRobotBridgeIOVariables(self, ioname=None, ionames=None, robotname=None, timeout=10, **kwargs):
        """Returns the data of the IO in ASCII hex as a string

        Args:
            ioname (str, optional): One IO name to read
            ionames (list, optional): A list of the IO names to read
            robotname (str, optional): Name of the robot
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
        """
        taskparameters = {
            'command': 'GetRobotBridgeIOVariables',
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname

        if ioname is not None and len(ioname) > 0:
            taskparameters['ioname'] = ioname
        if ionames is not None and len(ionames) > 0:
            taskparameters['ionames'] = ionames
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetRobotBridgeIOVariables(self, iovalues, robotname=None, timeout=10, **kwargs):
        """Sets a set of IO variables in the robot bridge.
        This should not lock self.env since it can happen during the runtime of a task and lock out other functions waiting in the queue.

        Args:
            iovalues:
            robotname (str, optional): Name of the robot
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            forceasync (bool, optional):
        """
        taskparameters = {
            'command': 'SetRobotBridgeIOVariables',
            'iovalues': list(iovalues)
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ComputeIkParamPosition(self, name, robotname=None, timeout=10, **kwargs):
        """Given the name of a Kinbody, computes the manipulator (TCP) position in the Kinbody frame to generate values for an IKParameterization.

        Args:
            name (str): Name of the Kinbody (the robot).
            robotname (str, optional): Name of the robot
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            unit (str, optional): The unit of the given values. (Default: 'mm')
            jointvalues (list[float], optional): If given, the robot's joints are set to these values before calculating the manipulator (TCP) position. If not set, uses the current values.

        Returns:
            dict: A dictionary with the structure:

                - translation (list[float]): List of x,y,z values of the object in millimeters.
                - quaternion (list[float]): List specifying the quaternion in w,x,y,z format, e.g. [1,0,0,0].
                - direction (list[float]): The global direction of the manipulator (assuming that the direction of the manipulator is the positive Z-axis).
                - angleXZ (float)
                - angleYX (float)
                - angleZY (float)
                - angleX (float)
                - angleY (float)
                - angleZ (float)
        """
        taskparameters = {
            'command': 'ComputeIkParamPosition',
            'name': name,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ComputeIKFromParameters(
        self,
        toolname=None,
        timeout=10,
        **kwargs
    ):
        """
        Args:
            toolname (str, optional): Tool name
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            targetname (str, optional): Name of the target object
            graspsetname (str, optional): Name of the grasp set to use
            ikparamnames (list[str], optional): If graspset does not exist, use the ikparamnames to initialize the grasp.
            limit (float, optional): Number of solutions to return
            useSolutionIndices (bool, optional):
            disabletarget (bool, optional):
            unit (str, optional): The unit of the given values. (Default: 'mm')
            randomBoxInfo (dict, optional): Info structure for maintaining grasp parameters for random box picking. Used when picking up randomized boxes (targetIsRandomBox is True). Keys: usefaces, dictFacePriorities, boxDirAngle, toolTranslationOffsets
            freeincvalue (float, optional): The discretization of the free joints of the robot when computing ik.
            freeinc (float, optional): **deprecated** The discretization of the free joints of the robot when computing ik.
            applyapproachoffset (bool, optional):
            inPlaneAngleDeviation (float, optional):
            outOfPlaneAngleDeviation (float, optional):
            searchfreeparams (bool, optional):
            returnClosestToCurrent (bool, optional):
            filteroptionslist (list[str], optional): A list of filter option strings. Can be: CheckEnvCollisions, IgnoreCustomFilters, IgnoreEndEffectorCollisions, IgnoreEndEffectorEnvCollisions, IgnoreEndEffectorSelfCollisions, IgnoreJointLimits, IgnoreSelfCollisions. Overrides filteroptions.
            filteroptions (int, optional): OpenRAVE IkFilterOptions bitmask. By default this is 1, which means all collisions are checked
            robotname (str, optional): Name of the robot

        Returns:
            dict: A dictionary with the structure:

                - solutions (list[dict]): Array of IK solutions (each of which is an array of DOF values), sorted by minimum travel distance and truncated to match the limit
                - errors (list[dict]): If no solutions found, the field 'errors' will contain reasons for the failure.
        """
        taskparameters = {
            'command': 'ComputeIKFromParameters',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout)

    def ReloadModule(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
        """
        taskparameters = {
            'command': 'ReloadModule',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ShutdownRobotBridge(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
        """
        taskparameters = {
            'command': 'ShutdownRobotBridge',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetRobotBridgeState(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'GetRobotBridgeState',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ClearRobotBridgeError(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
        """
        taskparameters = {
            'command': 'ClearRobotBridgeError',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetRobotBridgePause(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
        """
        taskparameters = {
            'command': 'SetRobotBridgePause',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetRobotBridgeResume(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
        """
        taskparameters = {
            'command': 'SetRobotBridgeResume',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetRobotBridgeServoOn(self, servoon, robotname=None, timeout=3, fireandforget=False):
        """
        Args:
            servoon (bool): If True, turns servo on.
            robotname (str, optional): Name of the robot
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 3)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'SetRobotBridgeServoOn',
            'isservoon': servoon,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetRobotBridgeLockMode(self, islockmode, robotname=None, timeout=3, fireandforget=False):
        """
        Args:
            islockmode (bool): If True, turns on Lock Mode. During Lock Mode, all communication with the physical robot is turned off and the hardware will not move.
            robotname (str, optional): Name of the robot
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 3)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'SetRobotBridgeLockMode',
            'islockmode': islockmode,
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResetSafetyFault(self, timeout=3, fireandforget=False):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 3)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ResetSafetyFault',
        }
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetRobotBridgeControlMode(self, controlMode, timeout=3, fireandforget=False):
        """
        Args:
            controlMode (str): The control mode to use, e.g. "Manual".
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 3)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'SetRobotBridgeControlMode',
            'controlMode': controlMode,
        }
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetDynamicObjects(self, timeout=1, **kwargs):
        """Get a list of dynamically added objects in the scene, from vision detection and physics simulation.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 1)
        """
        taskparameters = {
            'command': 'GetDynamicObjects',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ComputeRobotConfigsForGraspVisualization(self, targetname, graspname, robotname=None, toolname=None, unit='mm', timeout=10, **kwargs):
        """Returns robot configs for grasp visualization

        Args:
            targetname (str): Target object's name.
            graspname (str): Name of the grasp for which to visualize grasps.
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. (Default: "self.toolname")
            unit (str, optional): The unit of the given values. (Default: 'mm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            approachoffset (float, optional):
            departoffsetdir (list[float], optional): Direction in which to apply the offset when departing from the pick/place operation.
            departoffsetintool (list[float], optional):
            shadowrobotname (str, optional):
            shadowrobottoolname (str, optional):
        """
        taskparameters = {
            'command': 'ComputeRobotConfigsForGraspVisualization',
            'targetname': targetname,
            'graspname': graspname
        }
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if unit is not None:
            taskparameters['unit'] = unit
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout)

    def ResetCacheTemplates(self, timeout=1, fireandforget=False, **kwargs):
        """Resets any cached templates

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 1)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ResetCacheTemplates',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetRobotBridgeExternalIOPublishing(self, enable, timeout=2, fireandforget=False, **kwargs):
        """Enables publishing collision data to the robotbridge

        Args:
            enable (bool): If True, collision data will be published to robotbridge.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 2)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'SetRobotBridgeExternalIOPublishing',
            'enable': bool(enable),
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def RestoreSceneInitialState(self, timeout=1, **kwargs):
        """Restores the scene to the state on the filesystem

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 1)
            preserverobotdofvalues (int, optional): A Boolean value represented by 0 or 1. (Default: 1)
        """
        taskparameters = {
            'command': 'RestoreSceneInitialState',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningStepTest(self, jointName, amplitude, timeout=10, **kwargs):
        """Runs step response test on specified joint and returns result

        Args:
            jointName (str): The name of the joint.
            amplitude (float): The amplitude.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'RunMotorControlTuningStepTest',
            'jointName': jointName,
            'amplitude': amplitude,
        }
        taskparameters.update(kwargs)
        log.warn('sending taskparameters=%r', taskparameters)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningMaximulLengthSequence(self, jointName, amplitude, timeout=10, **kwargs):
        """Runs maximum length sequence test on specified joint and returns result

        Args:
            jointName (str): The name of the joint.
            amplitude (float): The amplitude.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'RunMotorControlTuningMaximulLengthSequence',
            'jointName': jointName,
            'amplitude': amplitude,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningDecayingChirp(self, jointName, amplitude, freqMax, timeout=120, **kwargs):
        """runs chirp test on specified joint and returns result

        Args:
            jointName (str): The name of the joint.
            amplitude (float): The amplitude.
            freqMax (float): The maximum frequency in Hz
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 120)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'RunMotorControlTuningDecayingChirp',
            'jointName': jointName,
            'freqMax': freqMax,
            'amplitude': amplitude,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningGaussianImpulse(self, jointName, amplitude, timeout=20, **kwargs):
        """Runs Gaussian Impulse test on specified joint and returns result

        Args:
            jointName (str): The name of the joint.
            amplitude (float): The amplitude.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 20)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'RunMotorControlTuningGaussianImpulse',
            'jointName': jointName,
            'amplitude': amplitude,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunMotorControlTuningBangBangResponse(self, jointName, amplitude, timeout=60, **kwargs):
        """Runs bangbang trajectory in acceleration or jerk space and returns result

        Args:
            jointName (str): The name of the joint.
            amplitude (float): The amplitude.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 60)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'RunMotorControlTuningBangBangResponse',
            'jointName': jointName,
            'amplitude': amplitude,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def RunDynamicsIdentificationTest(self, timeout, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 4)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'RunDynamicsIdentificationTest',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetTimeToRunDynamicsIdentificationTest(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            jointName (str, optional): The name of the joint.
            minJointAngle (float, optional): The joint angle to start the dynamics identification test at.
            maxJointAngle (float, optional): The joint angle to finish the dynamics identification test at.
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'GetTimeToRunDynamicsIdentificationTest',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def CalculateTestRangeFromCollision(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            jointName (str, optional): The name of the joint.
            unit (str, optional): The unit of the given values. (Default: 'mm')
            envclearance (float, optional): Environment clearance in millimeters.
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'CalculateTestRangeFromCollision',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetMotorControlParameterSchema(self, timeout=10, **kwargs):
        """Gets motor control parameter schema

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'GetMotorControlParameterSchema',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetMotorControlParameter(self, jointName, parameterName, timeout=10, **kwargs):
        """Gets motor control parameters as a name-value dict, e.g.: {'J1':{'KP':1}, 'J2':{'KV':2}}

        Args:
            jointName (str): The name of the joint.
            parameterName (str):
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'GetMotorControlParameter',
            'jointName': jointName,
            'parameterName': parameterName,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetMotorControlParameters(self, timeout=10, **kwargs):
        """Gets cached motor control parameters as name-value dict

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'GetMotorControlParameters',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetMotorControlParameter(self, jointName, parameterName, parameterValue, timeout=10, **kwargs):
        """Sets motor control parameter

        Args:
            jointName (str): The name of the joint.
            parameterName (str): The name of the parameter to set.
            parameterValue: The value to assign to the parameter.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            speed: **deprecated** Use robotspeed instead.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            ionames (list, optional): A list of IO names to read/write
        """
        taskparameters = {
            'command': 'SetMotorControlParameter',
            'jointName': jointName,
            'parameterName': parameterName,
            'parameterValue': parameterValue,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def IsProfilingRunning(self, timeout=10):
        """Queries if profiling is running on planning

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'IsProfilingRunning',
        }
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def StartProfiling(self, clocktype='cpu', timeout=10):
        """Start profiling planning

        Args:
            clocktype (str, optional): (Default: 'cpu')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'StartProfiling',
            'clocktype': clocktype,
        }
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def StopProfiling(self, timeout=10):
        """Stop profiling planning

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'StopProfiling',
        }
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ReplaceBodies(self, bodieslist, timeout=10, replaceInfos=None, **kwargs):
        """Replaces bodies in the environment with new uris

        Args:
            bodieslist: Used as replaceInfos if the replaceInfos is not defined. Used for backwards compatibility only.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            replaceInfos (list[dict], optional): list of dicts with keys: name, uri, containerDynamicProperties
            testLocationName (str, optional): If specified, will test if the container in this location matches testLocationContainerId, and only execute the replace if it matches and testLocationContainerId is not empty.
            testLocationContainerId (str, optional): containerId used for testing logic with testLocationName
            removeNamePrefixes (list[str], optional): Names of prefixes to match with when removing items
            removeLocationNames (list[str], optional):
            doRemoveOnlyDynamic (bool, optional): If True, removes objects that were added through dynamic means such as UpdateObjects/UpdateEnvironmentState. Default: False
            unit (str, optional): The unit of the given values. (Default: 'mm')
        """
        taskparameters = {
            'command': 'ReplaceBodies',
            'bodieslist': bodieslist, # for back compatibility for now
        }
        taskparameters['replaceInfos'] = bodieslist
        if replaceInfos is not None:
            taskparameters['replaceInfos'] = replaceInfos
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetState(self, timeout=10, fireandforget=False, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10.0)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
        """
        taskparameters = {
            'command': 'GetState',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def EnsureSyncWithRobotBridge(self, syncTimeStampUS, timeout=10, fireandforget=False, **kwargs):
        """Ensures that planning has synchronized with robotbridge data that is newer than syncTimeStampUS

        Args:
            syncTimeStampUS: us (microseconds, linux time) of the timestamp
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'EnsureSyncWithRobotBridge',
            'syncTimeStampUS': syncTimeStampUS,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResetCachedRobotConfigurationState(self, timeout=10, fireandforget=False, **kwargs):
        """Resets cached robot configuration (position of the robot) in the planning slave received from slave notification. Need to perform every time robot moved not from the task slaves.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ResetCachedRobotConfigurationState',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)
