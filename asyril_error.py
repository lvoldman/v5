__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["Leonid Voldman"]
__license__ = "SLA"
__version__ = "2.0.1"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Prototype"


asyrilErrorCodes:dict = {
    401: "The received command is unknown",

    402: "The given argument is not valid for this command",

    403: "System is not in production state. Ensure that a recipe has been started in production.",

    404: "The given parameter does not exist",

    405: "A get_part command is already active, most likely using a different connection",

    406: "The requested transition to a different system state is not allowed",

    407: "The recipe identifier is not found in the system. Ensure you used the correct recipe identifier and that the recipe is valid for production.",

    408: "The recipe is not ready to be used in production. Edit the recipe and complete the teaching wizard.",

    409: "The system is not in a valid state",

    410: "No valid license found. Ensure that a valid license has been installed.",

    411: "All internal concurrent connections are exhausted. Reduce the number of concurrent connections to the TCP/IP protocol.",

    412: "The purge option is not enabled in the Asycube settings. Navigate to configuration > Asycube and enable the purge option.",

    413: "System is not in purge state. Ensure that the purge has been started.",

    414: "A purge command is already active, most likely using a different connection.",

    415: "Invalid duration for purge command. Ensure that the argument is between 1 and 30000 (inclusive).",

    416: "Not enough points registered to perform hand-eye calibration. You must register 4 vision points and 4 robot points before calibrating.",

    417: "Requested point is not set. You must register the calibration point at least once before calling get_calibration_point.",

    419: "Invalid command for the recipe type. Commands related to a different recipe type are not supported on the current recipe. For instance, try to set a value for the multi part quantity variable on a single part recipe.",

    420: "The advanced purge is disabled in the recipe. Open the recipe and enable the feature in the purge configuration step.",

    421: "Missing reference image for the advanced purge. Ensure that the reference image in the recipe is configured in the purge configuration step.",

    422: "The purge vibrations aren’t fully configured. Ensure the purge vibrations in the recipe are fully configured.",

    423: "The Asyfill is not calibrated. Ensure that the Asyfill is calibrated. See Calibration procedure ",

    501: "Timeout occurred while trying to find valid parts. Check that there is a correct number of parts on the plate, the plate is not empty nor overfilled.",

    502: "An Asycube alarm was raised while trying to find valid parts, causing the command to timeout. ",

    503: "Timeout occurred while the system was waiting on can_take_image to become true",

    510: "get_part was interrupted by a stop/abort, most likely received on a different connection",

    511: "The system is not able to connect to the Asycube. Verify the cabling, the power supply and the IP settings.",

    512: "A communication error occurred while sending/receiving a message to/from the Asycube. Verify the cabling and the power supply.",

    513: "The command sent to the Asycube returned an error (ErXXXXX). Logs should contain more information. Check also the Asycube error codes.",

    514: "Error while turning the back light on or off. Verify the cabling, power supply and connection to the Asycube.",

    515: "Error while turning the front light on or off. Logs might contain more information.",

    516: "The camera is not connected. Logs should contain more information. Verify the cabling. The led should be green.",

    517: "The purge flap could not be opened/closed within the allowed time. Check that the purge flap can be operated properly, that no part is stuck. Check the sensor on the side.",

    518: "The purge was interrupted by a stop, most likely received on a different connection",

    519: "No calibration available. You must calculate the hand-eye calibration before saving or using it.",

    520: "No pick point match. No match found in last image analysis.",

    521: "All parts could not be purged within the given timeframe. Try running the command again. If no parts are left, try taking a new reference image from the recipe.",

    522: "A communication error occurred while sending/receiving a message to/from the Asyfill. Verify the cabling and the power supply to the Asyfill",

    595: "Internal error related to the Asyfill. Logs should contain more information about the problem.",

    596: "Internal error related to the production. Logs should contain more information about the problem.",

    597: "Internal error related to the vision. Logs should contain more information about the problem.",

    598: "Internal error related to the feeder. Logs should contain more information about the problem.",

    599: "Internal error related to system. Logs should contain more information about the problem."

}