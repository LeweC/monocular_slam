# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import threading
from std_msgs.msg import String
import rclpy

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


def get_key(settings):
    """ToDo DocString"""
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    """ToDo DocString"""
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    """ToDo DocString"""
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    """ToDo DocString"""
    settings = save_terminal_settings()

    rclpy.init()

    server_direction_node = rclpy.create_node("server_direction_node")

    pub = server_direction_node.create_publisher(String, "cmd_vel", 10)

    spinner = threading.Thread(target=rclpy.spin, args=(server_direction_node,))
    spinner.start()
    msg = String()
    try:
        while True:
            key = get_key(settings)
            msg.data = "Null"
            if key == "\x03":
                break
            elif key == "A":
                msg.data = "Up"
            elif key == "B":
                msg.data = "Down"
            elif key == "C":
                msg.data = "Right"
            elif key == "D":
                msg.data = "Left"

            if msg.data != "Null":
                pub.publish(msg)

    finally:
        pub.publish(msg)
        rclpy.shutdown()
        spinner.join()

        restore_terminal_settings(settings)


if __name__ == "__main__":
    main()
