#!/usr/bin/env python3
"""
ROS Controller Configuration Menu (Modularized)
- Manual controller selection with MAC address identification
- Separate DRIVE and ARM control systems (no conflicts)
- Modular button/axis mappings for easy customization
- ROS2 integration via joy_node subprocess
- Colorful & clean UI
"""

import curses
from curses import wrapper
from ui import ControllerUI

def main(stdscr):
    # Instantiate and run the UI
    # The UI manager handles ROS init, Controller scanning, and Joy processes
    ui = ControllerUI()
    ui.run(stdscr)

if __name__ == "__main__":
    try:
        wrapper(main)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Ensure terminal is restored on crash
        try:
            curses.endwin()
        except:
            pass
        print(f"An error occurred: {e}")
