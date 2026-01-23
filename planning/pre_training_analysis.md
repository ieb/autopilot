# Data Preparation

Training log input format should be candump to ensure all PGN data is available to data_loader.py. In addition pre training analysis needs to be performed to label the time periods of the logs. This information should be stored in metadata files which will record the mode of operation of the boat for periods of time in the logs. The dataloader will use this information to determine the mode of operation for any point in time.

## Pre Training Analysis

The log data needs to be analysed to discover how the boat was being sailed, mortored or at anchor, so that features like target_heading and mode can be determined for 
periods of time. Some of patterns are detailed below.

First the mode of boat operation.

If the sog is not present or zero with the lat lon wandering arround a fixed point then the boat is most probably at anchor and the section of the log file is nor relevant. Other features of a boat at anchor are it sailing on the anchor back and forth.

When the boat is motoring, in most cases you will find engine related PGNs in the logs, like rapid engine update with RPM. This is not definiatieve since sometimes the engine is run to charge batteries. If the tws is less than about 6kn, then the boat will almost certainly be motroing under engine. If the awa is below about 20 degrees then the boat is motoring into wind and cannot be sailing. If the roll is near zero or centered on zero then the boat most likely does not have sails up. The maximum stw the boat can do under engine is arround 7kn so anything above this is under sail.

A constat roll value on port or starboard indicates under sail, as does an angle in the polar where the boat can sail efficiently. 

Next target_heading.

There are 3 types of target modes that need to be supported. 

When motoring, the helm will be generally stearing in a straight line on a target heading for periods of between 5m and 30m. The helm may do collision avoidance to avoid other boats and obsticals, and may not be able to old the boat on the target heading all the time, but they will be steering to a point on the land or a compas bearing. The boat is never motored to a constant wind angle ever twa or awa.

When sailing the helm may be sailing to a target_heading, target_awa or target_twa.
Look for a straight line as with motoring or a constant awa irrespective of speed steering to a target_awa, or a constant twa as heading and awa varies steering to a target_twa.  Generally when the boat is sailing at a twa of more than 130, the mode is target_twa. Less than that the mode will have been target_awa.

There may be propprietary Raymarine Autopilot PGNs in the logs which may indicate that the autopilot is engaged and the mode it is following. The helm