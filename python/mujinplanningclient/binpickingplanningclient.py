# -*- coding: utf-8 -*-
# Copyright (C) 2013-2015 MUJIN Inc.
# Mujin planning client for binpicking task

# mujin imports
from . import realtimerobotplanningclient

# logging
import logging
log = logging.getLogger(__name__)


class BinpickingPlanningClient(realtimerobotplanningclient.RealtimeRobotPlanningClient):
    """Mujin planning client for the Binpicking task"""

    tasktype = 'binpicking'

    regionname = None  # The default region for Pick/Place calls

    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(self, regionname=None, **kwargs):
        """Connects to the Mujin controller, initializes Binpicking task and sets up parameters

        Args:
            regionname (str, optional): Name of the bin, e.g. container1
            robotname (str, optional): Name of the robot, e.g. VP-5243I
            robotspeed (float, optional): Speed of the robot, e.g. 0.4
            robotaccelmult (float, optional): Optional multiplier for the robot acceleration.
            envclearance (str, optional): Environment clearance in millimeter, e.g. 20
            robotBridgeConnectionInfo (str, optional): dict holding the connection info for the robot bridge.
            targetname (str, optional): Name of the target, e.g. plasticnut-center
            toolname (str, optional): Name of the manipulator, e.g. 2BaseZ
            taskzmqport (int, optional): Port of the task's ZMQ server, e.g. 7110. (Default: 11000)
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher, e.g. 7111. (Default: 11001)
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: binpicking
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
        self.regionname = regionname
        super(BinpickingPlanningClient, self).__init__(tasktype=self.tasktype, **kwargs)


    #
    # Commands
    #


    def PickAndPlace(
        self,
        goaltype=_deprecated,
        goals=_deprecated,
        targetnamepattern=None,
        approachoffset=30,
        departoffsetdir=[0, 0, 50],
        destdepartoffsetdir=[0, 0, 30],
        deletetarget=0,
        debuglevel=4,
        movetodestination=1,
        freeinc=[0.08],
        worksteplength=None,
        densowavearmgroup=5,
        regionname=None,
        cameranames=None,
        envclearance=None,
        toolname=None,
        robotspeed=None,
        timeout=1000,
        **kwargs
    ):
        """Picks up an object with the targetnamepattern and places it down at one of the goals. First computes the entire plan from robot moving to a grasp and then moving to its destination, then runs it on the real robot. Task finishes once the real robot is at the destination.

        Args:
            goaltype (str, optional): **deprecated** Type of the goal, e.g. translationdirection5d or transform6d
            goals (str, optional): **deprecated** Flat list of goals, e.g. two 5d ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]
            targetnamepattern (str, optional): regular expression describing the name of the object. No default will be provided, caller must set this. See https://docs.python.org/2/library/re.html
            approachoffset (float, optional): Distance in millimeters to move straight to the grasp point, e.g. 30 mm (Default: 30)
            departoffsetdir (list[float], optional): The direction and distance in mm to move the part in global frame (usually along negative gravity) after it is grasped, e.g. [0,0,50] (Default: [0, 0, 50])
            destdepartoffsetdir (list[float], optional): The direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system. (Default: [0, 0, 30])
            deletetarget (int, optional): whether to delete target after pick and place is done (Default: 0)
            debuglevel (int, optional): Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: 4)
            movetodestination (int, optional): planning parameter (Default: 1)
            freeinc (list[float], optional): planning parameter (Default: [0.08])
            worksteplength (float, optional): planning parameter
            densowavearmgroup (int, optional): planning parameter (Default: 5)
            regionname (str, optional): name of the region of the objects
            cameranames (list[str], optional): The names of the cameras to avoid occlusions with the robot
            envclearance (float, optional): Environment clearance in millimeters.
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 1000)
            leaveoffsetintool (int, optional): If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0.
            desttargetname (str, optional): The destination target name where the destination goal ikparams come from
            destikparamnames (list[list[str]], optional): A list of lists of ikparam names for the destinations of the target. Only destikparamnames[0] is looked at and tells the system to place the part in any of the ikparams in destikparamnames[0]
            graspsetname (str, optional): the name of the grasp set belong to the target objects to use for the target. Grasp sets are a list of ikparams
        """
        if worksteplength is None:
            worksteplength = 0.01
        assert (targetnamepattern is not None)
        if regionname is None:
            regionname = self.regionname
        taskparameters = {
            'command': 'PickAndPlace',
            'goaltype': goaltype,
            'envclearance': envclearance,
            'movetodestination': movetodestination,
            'goals': goals,
            'approachoffset': approachoffset,
            'departoffsetdir': departoffsetdir,
            'destdepartoffsetdir': destdepartoffsetdir,
            'freeinc': freeinc,
            'worksteplength': worksteplength,
            'targetnamepattern': targetnamepattern,
            'deletetarget': deletetarget,
            'debuglevel': debuglevel,
        }
        if regionname is not None:
            taskparameters['containername'] = regionname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, toolname=toolname, timeout=timeout)

    def StartPickAndPlaceThread(
        self,
        goaltype=None,
        goals=None,
        targetnamepattern=None,
        approachoffset=30,
        departoffsetdir=[0, 0, 50],
        destdepartoffsetdir=[0, 0, 30],
        deletetarget=0,
        debuglevel=4,
        movetodestination=1,
        worksteplength=None,
        regionname=None,
        envclearance=None,
        toolname=None,
        robotspeed=None,
        timeout=10,
        **kwargs
    ):
        """Start a background loop to continuously pick up objects with the targetnamepattern and place them down at the goals. The loop will check new objects arriving in and move the robot as soon as it finds a feasible grasp. The thread can be quit with StopPickPlaceThread.

        Args:
            goaltype (str, optional): type of the goal, e.g. translationdirection5d
            goals (list, optional): flat list of goals, e.g. two 5d ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]
            targetnamepattern (str, optional): regular expression describing the name of the object, no default will be provided, caller must set this. See https://docs.python.org/2/library/re.html
            approachoffset (float, optional): distance in millimeters to move straight to the grasp point, e.g. 30 mm (Default: 30)
            departoffsetdir (list[float], optional): the direction and distance in mm to move the part in global frame (usually along negative gravity) after it is grasped, e.g. [0,0,50] (Default: [0, 0, 50])
            destdepartoffsetdir (list[float], optional): the direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system. (Default: [0, 0, 30])
            deletetarget (int, optional): whether to delete target after pick and place is done (Default: 0)
            debuglevel (int, optional): Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: 4)
            movetodestination (int, optional): planning parameter (Default: 1)
            worksteplength (float, optional): planning parameter
            regionname (str, optional): name of the region of the objects
            envclearance (float, optional): Environment clearance in millimeters.
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            densowavearmgroup (int, optional): robot parameters (Default: 5)
            cameranames (list[str], optional): the names of the cameras to avoid occlusions with the robot
            cycledests (int, optional): When finished cycling through all destikparamnames, will delete all the targets and start from the first index again doing this for cycledests times. By default it is 1.
            desttargetname (str, optional): The destination target name where the destination goal ikparams come from
            destikparamnames (list[list[str]], optional): A list of lists of ikparam names for the ordered destinations of the target. destikparamnames[0] is where the first picked up part goes, desttargetname[1] is where the second picked up target goes.
            leaveoffsetintool (int, optional): If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0.
            graspsetname: the name of the grasp set belong to the target objects to use for the target. Grasp sets are a list of ikparams
            useworkspaceplanner (int, optional): If 1 is set, will try the workspace planner for moving the hand straight. If 2 is set, will try the RRT for moving straight. Can set 3 for trying both.
            forceStartRobotValues (list[float], optional): planning loop should always start from these values rather than reading from robot
            initiallyDisableRobotBridge (bool, optional): If True, stops any communication with the robotbridge until robot bridge is enabled. (Default: False)
        """
        if worksteplength is None:
            worksteplength = 0.01
        assert (targetnamepattern is not None)
        if regionname is None:
            regionname = self.regionname
        taskparameters = {
            'command': 'StartPickAndPlaceThread',
            'envclearance': envclearance,
            'movetodestination': movetodestination,
            'approachoffset': approachoffset,
            'departoffsetdir': departoffsetdir,
            'destdepartoffsetdir': destdepartoffsetdir,
            'worksteplength': worksteplength,
            'targetnamepattern': targetnamepattern,
            'containername': regionname,
            'deletetarget': deletetarget,
            'debuglevel': debuglevel,
        }
        if goals is not None:
            taskparameters['orderedgoals'] = goals
            taskparameters['goaltype'] = goaltype
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, toolname=toolname, timeout=timeout)

    def StopPickPlaceThread(self, resetExecutionState=True, resetStatusPickPlace=False, finishCode=None, finishMessage=None, timeout=10, fireandforget=False, **kwargs):
        """stops the pick and place thread started with StartPickAndPlaceThread

        Args:
            resetExecutionState (bool, optional): if True, then reset the order state variables. By default True (Default: True)
            resetStatusPickPlace (bool, optional): if True, then reset the statusPickPlace field of hte planning slave. (Default: False)
            finishCode (str, optional): optional finish code to end the cycle with (if it doesn't end with something else beforehand)
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'StopPickPlaceThread',
            'resetExecutionState': resetExecutionState,
            'resetStatusPickPlace': resetStatusPickPlace,
            'finishCode': finishCode,
            'finishMessage': finishMessage,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetPickPlaceStatus(self, timeout=10, **kwargs):
        """Gets the status of the pick and place thread

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)

        Returns:
            dict: Status of the pick and place thread.

                A dictionary with the structure:

                - status (int): One for the following. 0: not running, 1: no error, 2: error
                - error (str)
        """
        taskparameters = {'command': 'GetPickPlaceStatus'}
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ComputeIK(self, toolname=None, timeout=10, **kwargs):
        """
        Args:
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            iktype (str, optional): grasp (but basically the just the ikparam)
            quaternion (list[float], optional): List specifying the quaternion in w,x,y,z format, e.g. [1,0,0,0].
            translation (list[float], optional): List of x,y,z values of the object in millimeters.
            direction (list, optional): grasp (but basically the just the ikparam) direction in world cooordinates
            angle (float, optional): grasp (but basically the just the ikparam) angle in world cooordinates
            freeincvalue (float, optional): The discretization of the free joints of the robot when computing ik.
            filteroptions (int, optional): OpenRAVE IkFilterOptions bitmask. By default this is 1, which means all collisions are checked
            limit (int, optional): number of solutions to return
            preshape (list[float], optional): If the tool has fingers after the end effector, specify their values. The gripper DOFs come from **gripper_dof_pks** field from the tool.

        Returns:
            dict: A dictionary with the structure:

                - solutions (list): array of IK solutions (each of which is an array of DOF values), sorted by minimum travel distance and truncated to match the limit
        """
        taskparameters = {'command': 'ComputeIK'}
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout)

    def InitializePartsWithPhysics(self, timeout=10, **kwargs):
        """Start a physics simulation where the parts drop down into the bin. The method returns as soon as the physics is initialized, user has to wait for the "duration" or call StopPhysicsThread command.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            targeturi: the target uri to initialize the scene with
            numtargets: the number of targets to create
            regionname (str, optional): The container name to drop the parts into.
            duration: the duration in seconds to continue the physics until it is stopped.
            basename: The basename to give to all the new target names. Numbers are suffixed at the end, like basename+'0134'. If not specified, will use a basename derived from the targeturi.
            deleteprevious: if True, will delete all the previous targets in the scene. By default this is True.
            forcegravity: if not None, the gravity with which the objects should fall with. If None, then uses the scene's gravity
        """
        taskparameters = {'command': 'InitializePartsWithPhysics'}
        taskparameters.update(kwargs)
        if 'containername' not in taskparameters:
            taskparameters['containername'] = self.regionname
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def StopPhysicsThread(self, timeout=10, **kwargs):
        """stops the physics simulation started with InitializePartsWithPhysics

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {'command': 'StopPhysicsThread'}
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def JitterPartUntilValidGrasp(self, toolname=None, timeout=10, **kwargs):
        """Select a part that wasn't able to be grasped and jitter its location such that a grasp set is found for it that will take it to the destination.

        Args:
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            targetname (str, optional): Name of the target object
            graspsetname (str, optional): Name of the grasp set to use
            approachoffset: The approach distance for simulating full grasp.
            departoffsetdir: The depart distance for simulating full grasp.
            destdepartoffsetdir: the direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system.
            leaveoffsetintool: If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0.
            desttargetname: The destination target name where the destination goal ikparams come from. If no name is specified, then robot won't consider putting the target into the destination when it searches for grasps.
            destikparamnames (list[list[str]], optional): A list of lists of ikparam names for the ordered destinations of the target. destikparamnames[0] is where the first picked up part goes, desttargetname[1] is where the second picked up target goes.
            jitterangle: Amount to jitter the target object's orientation angle
            jitteriters: Number of times to try jittering before giving up.
            jitterdist: Amount to jitter the target object translation by

        Returns:
            If failed, an empty dictionary. If succeeded, a dictionary with the following keys:

                - translation: the new translation of the target part
                - quaternion: the new quaternion of the target part
                - jointvalues: robot joint values that are grasping the part (fingers are at their preshape).
                - graspname: the grasp name used for jointvalues. If empty, then no grasp was found.
                - destikname: the name of the destination ikparam that was chosen with the grasp
                - destjointvalues: robot joint values at one of the specified destinations (fingers are at their final positions).
                - desttranslation: the new translation of the target part
                - destquaternion: the new quaternion of the target part
        """
        taskparameters = {'command': 'JitterPartUntilValidGrasp'}
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout)

    def MoveToDropOff(self, dropOffInfo, robotname=None, robotspeed=None, robotaccelmult=None, execute=1, startvalues=None, envclearance=None, timeout=10, **kwargs):
        """Moves the robot to desired joint angles.

        Args:
            dropOffInfo:
            robotname (str, optional): Name of the robot
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            execute (int, optional): If 1, execute the motion. (Default: 1)
            startvalues (list[float], optional):
            envclearance (float, optional): Environment clearance in millimeters.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'MoveToDropOff',
            'dropOffInfo': dropOffInfo,
            'execute': execute,
        }
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if startvalues is not None:
            taskparameters['startvalues'] = list(startvalues)
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotname=robotname, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def IsRobotOccludingBody(self, bodyname, cameraname, timeout=10, **kwargs):
        """returns if the robot is occluding body in the view of the specified camera

        Args:
            bodyname: Name of the object
            cameraname: Name of the camera
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)

        Returns:
            dict: The occlusion state in a json dictionary, e.g. {'occluded': 0}

                An unstructured dictionary.
        """
        taskparameters = {
            'command': 'IsRobotOccludingBody',
            'bodyname': bodyname,
            'cameraname': cameraname,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetPickedPositions(self, unit='m', timeout=10, **kwargs):
        """returns the poses and the timestamps of the picked objects

        Args:
            unit (str, optional): The unit of the given values. (Default: 'm')
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)

        Returns:
            The positions and the timestamps of the picked objects in a json dictionary, info of each object has the format of quaternion (w,x,y,z) followed by x,y,z translation (in mm) followed by timestamp in milisecond e.g. {'positions': [[1,0,0,0,100,200,300,1389774818.8366449],[1,0,0,0,200,200,300,1389774828.8366449]]}
        """
        taskparameters = {
            'command': 'GetPickedPositions',
            'unit': unit,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetPickAndPlaceLog(self, timeout=10, **kwargs):
        """Gets the recent pick-and-place log executed on the binpicking server. The internal server keeps the log around until the next Pick-and-place command is executed.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            startindex (int, optional): Start of the trajectory to get. If negative, will start counting from the end. For example, -1 is the last element, -2 is the second to last element.
            num (int, optional): Number of trajectories from startindex to return. If 0 will return all the trajectories starting from startindex

        Returns:
            A dictionary with keys, for example:

                {

                    total: 10
                    messages: [{

                            "message":"message1",
                            "type":"",
                            "level":0,
                            "data": {

                                "jointvalues":[0,0,0,0,0,0]
                            }
                        },
                        ...
                    ]
                }
        """
        taskparameters = {
            'command': 'GetPickAndPlaceLog',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)
    
    def MoveRobotOutOfCameraOcclusion(self, regionname=None, robotspeed=None, toolname=None, timeout=10, **kwargs):
        """Moves the robot out of camera occlusion and deletes targets if it was in occlusion.

        Args:
            regionname (str, optional): name of the region of the objects
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            cameranames (list[str], optional): The names of the cameras to avoid occlusions with the robot
        """
        if regionname is None:
            regionname = self.regionname
        taskparameters = {
            'command': 'MoveRobotOutOfCameraOcclusion',
            'containername': regionname,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, toolname=toolname, timeout=timeout)

    def PausePickPlace(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'PausePickPlace',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ResumePickPlace(self, timeout=10, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'ResumePickPlace',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SendStateTrigger(self, stateTrigger, timeout=10, fireandforget=False, **kwargs):
        """
        Args:
            stateTrigger (str): a string that represents a unique trigger
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'SendStateTrigger',
            'stateTrigger': stateTrigger,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetBinpickingState(self, timeout=10, fireandforget=False, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {'command': 'GetBinpickingState'}
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)
    
    def SetStopPickPlaceAfterExecutionCycle(self, timeout=10, **kwargs):
        """Sets the cycle for stopping after the current pick cycle finishes.

        If robot has not grabbed a part yet, then will stop the robot immediately.
        On proper finish of the pick cycle, robot should go back to the finish position.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            finishCode: The finish code to end with. If not specified, will be 'FinishedCycleStopped'
        """
        taskparameters = {
            'command': 'SetStopPickPlaceAfterExecutionCycle',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def PutPartsBack(self, trajectoryxml, numparts, toolname=None, grippervalues=None, timeout=100, **kwargs):
        """Runs saved planningresult trajectories.

        Args:
            trajectoryxml (str):
            numparts (int):
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            grippervalues (list, optional):
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 100)
        """
        taskparameters = {
            'command': 'PutPartsBack',
            'trajectory': trajectoryxml,
            'numparts': numparts,
            'toolname': toolname,
        }
        if grippervalues is not None:
            taskparameters['grippervalues'] = grippervalues
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GenerateGraspModelFromIkParams(self, graspsetname, targeturi, toolname, robotname=None,
                                       timeout=10, **kwargs):
        """Generates grasp model IK for given setup.

        Args:
            graspsetname (str): Name of the grasp set to use
            targeturi (str): uri of target scene, e.g. '4902201402644.mujin.dae'
            toolname (str): Name of the manipulator. Defaults to currently selected tool
            robotname (str, optional): Name of the robot
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'GenerateGraspModelFromIkParams',
            'graspsetname': graspsetname,
            'targeturi': targeturi,
            'toolname': toolname,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotname=robotname, toolname=toolname, timeout=timeout)

    def CheckGraspModelIk(self, graspsetname, targeturi, toolname, ikparamnames=None, timeout=10, **kwargs):
        """Checks if grasp model is generated for given setup.

        Args:
            graspsetname (str): Name of the grasp set to use
            targeturi: str. uri of target scene like 'mujin:4902201402644.mujin.dae'
            toolname (str): Name of the manipulator. Defaults to currently selected tool
            ikparamnames (list, optional):
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'CheckGraspModelIk',
            'graspsetname': graspsetname,
            'targeturi': targeturi,
            'toolname': toolname,
            'ikparamnames': ikparamnames,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetCurrentLayoutDataFromPLC(self, containername, containerLayoutSize, destObstacleName, ioVariableName, timeout=10, **kwargs):
        """Sets current layout from PLC.

        Args:
            containername (str):
            containerLayoutSize:
            destObstacleName (str):
            ioVariableName (str):
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'SetCurrentLayoutDataFromPLC',
            'containername': containername,
            'containerLayoutSize': containerLayoutSize,
            'ioVariableName': ioVariableName,
            'destObstacleName': destObstacleName,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=False)

    def ClearVisualization(self, timeout=10, fireandforget=False, **kwargs):
        """Clears visualization.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {'command': 'ClearVisualization'}
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetPlanStatistics(self, timeout=1, fireandforget=False, **kwargs):
        """Gets plan and execute statistics of the last pick and place

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 1)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {'command': 'GetPlanStatistics'}
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetCurrentLayoutDataSendOnObjectUpdateData(self, doUpdate, containername=None, containerLayoutSize=None, ioVariableName=None, fireandforget=True, **kwargs):
        """Sets currentLayoutDataSendOnObjectUpdateData structure

        Args:
            doUpdate: If True then currentLayoutData will be send on every ObjectUpdate, else currentLayoutDataSendOnObjectUpdate structure is reset
            containername (str, optional):
            containerLayoutSize:
            ioVariableName:
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: True)
        """
        taskparameters = {
            'command': 'SetCurrentLayoutDataSendOnObjectUpdateData',
            'doUpdate': doUpdate,
        }
        if containername is not None:
            taskparameters['containername'] = containername
        if containerLayoutSize is not None:
            taskparameters['containerLayoutSize'] = containerLayoutSize
        if ioVariableName is not None:
            taskparameters['ioVariableName'] = ioVariableName
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, fireandforget=fireandforget)

    def ManuallyPlacePackItem(self, packFormationComputationResult=None, inputPartIndex=None, placeLocationNames=None, placedTargetPrefix=None, dynamicGoalsGeneratorParameters=None, orderNumber=None, numLeftToPick=None, timeout=10, fireandforget=False):
        """Places an item according to the pack formation assuming the item is placed manually and updates robotbridge state

        Args:
            packFormationComputationResult:
            inputPartIndex:
            placeLocationNames:
            placedTargetPrefix:
            dynamicGoalsGeneratorParameters:
            orderNumber:
            numLeftToPick:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ManuallyPlacePackItem',
            'packFormationComputationResult': packFormationComputationResult,
            'inputPartIndex': inputPartIndex,
            'placeLocationNames': placeLocationNames,
        }
        if orderNumber is not None:
            taskparameters['orderNumber'] = orderNumber
        if numLeftToPick is not None:
            taskparameters['numLeftToPick'] = numLeftToPick
        if numLeftToPick is not None:
            taskparameters['numLeftToPick'] = numLeftToPick
        if placedTargetPrefix:
            taskparameters['placedTargetPrefix'] = placedTargetPrefix
        if dynamicGoalsGeneratorParameters is not None:
            taskparameters['dynamicGoalsGeneratorParameters'] = dynamicGoalsGeneratorParameters
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ToggleSourceContainerObservationMatrixVisualization(self, timeout=10, fireandforget=False):
        """Toggles visualization of source container observation matrix in the viewer.
        
        """
        taskparameters = {
            'command': 'ToggleSourceContainerObservationMatrixVisualization'
        }
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ValidatePackFormation(
        self,
        packFormation=None,
        validationSettings=None,
        **kwargs
    ):
        """Validates the pack formation.

        Args:
            packFormation (packFormationComputationResultType): A result of a pack formation computation.
            validationSettings (dict): Parameters for the validation.
        """
        taskparameters = {
            'command': 'ValidatePackFormation',
            'packFormation': packFormation,
            'validationSettings': validationSettings,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters)
