#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Thin launcher for the modular RobStride gain tuner.

This preserves the original script name while delegating to the
``robstride_gain_tuner`` package:

    python3 utils/robstride_gain_tuner_liveplot_mac_safe.py
    python3 -m utils.robstride_gain_tuner
"""

from robstride_gain_tuner.main import main

if __name__ == "__main__":
    main()
