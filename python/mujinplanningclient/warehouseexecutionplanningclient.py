# -*- coding: utf-8 -*-
# Copyright (C) 2024 MUJIN Inc.

from . import planningclient

import logging
log = logging.getLogger(__name__)

class WarehouseExecutionPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the warehouse execution planner"""

    tasktype = 'warehouseexecutionplanning'

    def __init__(self, **kwargs):
        kwargs['tasktype'] = self.tasktype # override task type
        super(WarehouseExecutionPlanningClient, self).__init__(**kwargs)

    def StartWarehouseExecutionPlanningThread(self, timeout=None, fireandforget=None, checkpreempt=True, blockwait=True, **kwargs):
        """
        This function will start the warehouse execution planning thread.
        """
        taskparameters = {
            'command': 'StartWarehouseExecutionPlanningThread',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, checkpreempt=checkpreempt, blockwait=blockwait)

    def StopWarehouseExecutionPlanningThread(self, timeout=None, fireandforget=None, checkpreempt=True, blockwait=True, **kwargs):
        """
        This function will stop the warehouse execution planning thread.

        """
        taskparameters = {
            'command': 'StopWarehouseExecutionPlanningThread',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, checkpreempt=checkpreempt, blockwait=blockwait)
