
#!/usr/bin/env python

# Copyright (C) 2014, PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of PAL Robotics S.L. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import rospy
import rospkg

from std_msgs.msg import String

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QWidget, QFormLayout

# TODO:
# - Better UI suppor for continuous joints (see DoubleEditor TODO)
# - Can we load controller joints faster?, it's currently pretty slow
# - If URDF is reloaded, allow to reset the whole plugin?
# - Allow to configure:
#   - URDF location
#   - Command publishing and state update frequency
#   - Controller manager and jtc monitor frequency
#   - Min trajectory duration
# - Fail gracefully when the URDF or some other requisite is not set
# - Could users confuse the enable/disable button with controller start/stop
#   (in a controller manager sense)?
# - Better decoupling between model and view
# - Tab order is wrong. For the record, this did not work:
#   QWidget.setTabOrder(self._widget.controller_group,
#                       self._widget.joint_group)
#   QWidget.setTabOrder(self._widget.joint_group,
#                       self._widget.speed_scaling_group)

# NOTE:
# Controller enable/disable icons are in the public domain, and available here:
# freestockphotos.biz/photos.php?c=all&o=popular&s=0&lic=all&a=all&set=powocab_u2


class SeeScopeControl(Plugin):
    """
    Graphical frontend for a C{JointTrajectoryController}.

    There are two modes for interacting with a controller:
        1. B{Monitor mode} Joint displays are updated with the state reported
          by the controller. This is a read-only mode and does I{not} send
          control commands. Every time a new controller is selected, it starts
          in monitor mode for safety reasons.

        2. B{Control mode} Joint displays update the control command that is
        sent to the controller. Commands are sent periodically evan if the
        displays are not being updated by the user.

    To control the aggressiveness of the motions, the maximum speed of the
    sent commands can be scaled down using the C{Speed scaling control}

    This plugin is able to detect and keep track of all active controller
    managers, as well as the JointTrajectoryControllers that are I{running}
    in each controller manager.
    For a controller to be compatible with this plugin, it must comply with
    the following requisites:
        - The controller type contains the C{JointTrajectoryController}
        substring, e.g., C{position_controllers/JointTrajectoryController}
        - The controller exposes the C{command} and C{state} topics in its
        ROS interface.

    Additionally, there must be a URDF loaded with a valid joint limit
    specification, namely position (when applicable) and velocity limits.

    A reference implementation of the C{JointTrajectoryController} is
    available in the C{joint_trajectory_controller} ROS package.
    """

    def __init__(self, context):
        super(SeeScopeControl, self).__init__(context)
        self.setObjectName('SeeScopeControl')

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_see_scope_control'),
                               'resource',
                               'see_scope_control.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SeeScopeControlUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
            # Add widget to the user interface
        context.add_widget(self._widget)

        cmd_topic = '/see_scope/projector/command'
        self._cmd_pub = rospy.Publisher(cmd_topic,
                                        String,
                                        queue_size=1)

        w = self._widget
        w.enable_button.toggled.connect(self._on_ssc_enabled)

    def _enable_fringes(self):
        self._cmd_pub.publish(String("fringes"))

    def _disable_fringes(self):
        self._cmd_pub.publish(String("white"))
        
    def _on_ssc_enabled(self, val):
        if val:
            self._enable_fringes();
        else:
            self._disable_fringes();

            
