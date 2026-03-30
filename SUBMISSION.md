# Lab 2 Submission

## Student
Sai Jayanth Seethamsetty

## Repository
This repository contains my Lab 2 work for the F1TENTH automatic emergency braking assignment.

## Files Completed
- `safety_node/scripts/safety_node.py`
- `safety_node/package.xml`
- `safety_node/setup.py`
- `safety_node/setup.cfg`

## Implementation Summary
I implemented the Python safety node to:
- subscribe to `/scan`
- subscribe to `/ego_racecar/odom`
- compute instantaneous TTC
- publish a brake command to `/drive` when TTC falls below threshold

## Testing Performed
- built the package locally
- built the package inside the simulator container
- confirmed simulator topics `/scan`, `/ego_racecar/odom`, and `/drive`
- launched the simulator bridge successfully
- launched teleop and recorded final testing evidence

## Video
Screencast link: https://drive.google.com/file/d/1Nl1QUkd1A7NZHsw4RP3RjCw14-aKtDKM/view?usp=sharing

## Limitation Encountered
Final runtime validation in this VM environment was affected by simulator/container deserialization and stability issues during the last stage of testing.
