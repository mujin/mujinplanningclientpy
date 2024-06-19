# -*- coding: utf-8 -*-
# Copyright (C) 2023 MUJIN Inc.

from . import planningclient

import logging
log = logging.getLogger(__name__)

class MobileRobotPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the multi-agent mobile robot path planner"""

    tasktype = 'mobilerobotplanning'

    def __init__(self, **kwargs):
        kwargs['tasktype'] = self.tasktype # override task type
        super(MobileRobotPlanningClient, self).__init__(**kwargs)

    def ManageMobileRobotTasks(self, tasks, timeout=None, fireandforget=None, checkpreempt=True, blockwait=True, **kwargs):
        """
        This function will initialize the planning thread if it's not started.

        Arguments:
            tasks (list[dict]): list of MobileRobotTask changes.
        """
        taskparameters = {
            'command': 'ManageMobileRobotTasks',
            'tasks': tasks,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, checkpreempt=checkpreempt, blockwait=blockwait)
