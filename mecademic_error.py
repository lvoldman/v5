__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "1.0.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


mecademicErrorCodes:dict = {
    1000: "Command buffer is full.",
    1001: "Empty command or command unrecognized. - Command: '...'",
    1002: "Syntax error, symbol missing. - Command: '...'",
    1003: "Argument error. - Command: '...'",
    1005: "The robot is not activated.",
    1006: "The robot is not homed.",
    1007: "Joint over limit (... is not in range [...,...] for joint ...). - Command: '...'.",
    1010: "Linear move is blocked because a joint would rotate by more than 180deg. - Command: '...'",
    1011: "The robot is in error.",
    1012: "Linear move not possible due to a singularity along the path - Command: '...'.",
    1013: "Activation failed.",
    1014: "Homing failed.",
    1016: "Destination pose out of reach for any configuration. - Command: '...'",
    1016: "Destination pose out of reach for selected conf(...,...,... turn ...). - Command: '...'",
    1016: "The requested linear move is not possible due to a pose out of reach along the path. - Command: '...'",
    1022: "Robot was not saving the program.",
    1023: "Ignoring command for offline mode. - Command: '...'",
    1024: "Mastering needed. - Command: '...'",
    1025: "Impossible to reset the error. Please, power-cycle the robot.",
    1026: "Deactivation needed to execute the command. - Command: '...'",
    1027: "Simulation mode can only be enabled/ disabled while the robot is deactivated.",
    1029: "Offline program full. Maximum program size is 13,000 commands. Saving stopped.",
    1030: "Already saving.",
    1031: "Program saving aborted after receiving illegal command. - Command: '...'",
    1033: "Start conf mismatch",
    1038: "No gripper connected.",
    1040: "Command failed.",
    1041: "No Vbox",
    1042: "Ext tool sim must deactivated",
    1043: "The specified IO bank is not present on this robot",
    1044: "There is no vacuum module present on this robot.",
    3001: "Another user is already connected, closing connection.",
    3002: "A firmware upgrade is in progress (connection refused).",
    3003: "Command has reached the maximum length.",
    3005: "Error of motion.",
    3006: "Error of communication with drives",
    3009: "Robot initialization failed due to an internal error. Restart the robot.",
    3014: "Problem with saved program, save a new program.",
    3017: "No offline program saved.",
    3020: "Offline program ... is invalid",
    3025: "Gripper error.",
    3026: "Robot's maintenance check has discovered a problem. Mecademic cannot guarantee correct movements. Please contact Mecademic.",
    3027: "Internal error occurred.",
    3029: "Excessive torque error occurred",
    3031: "A previously received text API command was incorrect.",
    3033: "Robot configuration is missing/ corrupted. *** Please contact support ***",
    3037: "Pneumatic module error",
    3039: "External tool firmware must be updated.",
    3041: "Robot error due to imminent collision.",
    3042: "Detected failure in previous firmware update. Please re-install the firmware again.",
    3043: "Excessive communication errors with external tool.",
    3044: "Abnormal communication error with external port.",
    3046: "Power-supply detected a non-resettable power error. Please check power connection then power-cycle the robot",
    3047: "Robot failed to mount drive. Please try to power-cycle the robot. If the problem persists contact Mecademic support.",
    3049: "Robot error at work zone limit"
}

def mecademicErrorMsg(msgCode:int) -> str:
    return mecademicErrorCodes[msgCode]