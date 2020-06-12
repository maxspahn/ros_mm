#!/usr/bin/env python

import os
import sys

sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "../../forcesLib/mobileManipulator/"
    )
)

import time
import numpy as np
import mm_MPC_py

def solve_MPC_mm(params):
    return mm_MPC_py.mm_MPC_solve(params)
