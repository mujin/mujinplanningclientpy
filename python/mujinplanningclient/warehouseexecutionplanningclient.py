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
