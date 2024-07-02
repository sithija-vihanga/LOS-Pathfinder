<div style="text-align: center;">





## **<u>RECOVERY</u>Y <u>PATHFINDER PACKAGE CONFIGURATIONS</u>**

https://github.com/sithija-vihanga/LOS-Pathfinder/assets/116638289/415384c3-12f4-4249-9785-9aafbcb68d1c

![h5dbsdff](https://github.com/sithija-vihanga/LOS-Pathfinder/assets/116638289/dd133e2f-5cb8-4704-8506-03531cdbbb1d)


</div>

**Pulse Monitor:**

Main module of the pathfinder package which initiates all the required modules.

Contains 1 second timer to check the connectivity of the robot.

This module can be used to change the disconnectivity checking approach as given below.

![Pulse Monitor Image](https://github.com/sithija-vihanga/LOS-Pathfinder/assets/116638289/ff7f3dbe-0d9b-4bdf-8163-8d7f313300c8)

![Pulse Monitor Image](https://github.com/sithija-vihanga/LOS-Pathfinder/assets/116638289/8565cbcf-ad58-4fdb-9926-1d68c1cd7bd6)

![Pulse Monitor Image](https://github.com/sithija-vihanga/LOS-Pathfinder/assets/116638289/30524ee2-44ee-4f76-bf1d-9d9be2f86af7)

Contains logic of main state machine for recovery. RECOVERYSTATUS.CHECKPULSE RECOVERYSTATUS.RECOVERY RECOVERYSTATUS.NAVIGATE

**Pulse Checker:**

Publishes a header msg containing timestamp and robots ID.

Module subscribes to all header msgs coming from all the connected robots.

If robot does not receive any header msgs, detect it as robot disconnected from the network.

5-second timeout is defined before going for the recovery stage.

![Pulse Checker Image](https://github.com/sithija-vihanga/LOS-Pathfinder/assets/116638289/2032370f-7eef-4873-afa3-aa083c83ab6d)

**NOTE:** **Due to the decentralized approach of sending header msgs (pulses) affected by the local time of each robot, a difference between first and current readings are considered.**

**RSSI Monitor:**

Used to monitor received signal strength by onboard computer’s network interface card (Lattepanda board). This needs to be improved to work with RAJANT router’s network interface.

Threshold for disconnectivity is defined as -70dbm.

**Recovery Navigator:**

Uses the coordinates given by recovery path calculations to send a goal through action client while receiving progress.

Uses `navigateToPose` but actual orientation is neglected.

**NOTE:** **After goal succeeded, robot will check the connectivity status and proceed with the state machine. This helps to reduce back and forth movements of robots at the edge of the map.**

**To further improve this we could use key locations path to best coverage point and check connectivity at each location.**

**(Key points/junctions are already calculated by LOSPathfinder module)**

