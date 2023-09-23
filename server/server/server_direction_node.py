# This Script is heavily inspired by: https://github.com/ros2/teleop_twist_keyboard/tree/dashing/
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
    """Get a key press from the user.

    This function reads a single key press from the user. On Windows, it uses msvcrt.getwch() to get the key,
    while on Linux, it sets the terminal to raw mode, reads one character from stdin, and then restores the terminal settings.

    Args:
        settings (termios): The original terminal settings.

    Returns:
        str: The key that was pressed.
    """
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
    """Save the current terminal settings.

    This function saves the current terminal settings to be restored later.

    Returns:
        termios or None: The terminal settings on Linux or None on Windows.
    """
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)

def restore_terminal_settings(old_settings):
    """Restore the terminal settings.

    This function restores the terminal settings to their original state.

    Args:
        old_settings (termios): The original terminal settings.
    """
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    """Main entry point for the server_direction_node.

    This function initializes the ROS 2 environment, creates a node, a publisher for cmd_vel messages,
    and sets up a loop to read keyboard input and publish corresponding direction commands. It also handles
    terminal settings to read single key presses.

    """
    settings = save_terminal_settings()

    rclpy.init()

    server_direction_node = rclpy.create_node("server_direction_node")

    pub = server_direction_node.create_publisher(String, "cmd_vel", 10)

    spinner = threading.Thread(target=rclpy.spin, args=(server_direction_node,))
    spinner.start()
    msg = String()
    try:
        while True:
            msg.data = "Null"
            key = get_key(settings)
            print(type(key))

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
            else:
                msg.data = "Null"
            pub.publish(msg)
    finally:
        pub.publish(msg)
        rclpy.shutdown()
        spinner.join()

        restore_terminal_settings(settings)

if __name__ == "__main__":
    main()
