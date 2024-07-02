> **<u>RECOVER</u>Y <u>PATHFINDER PACKAGE
> CONFIGURATIONS</u>**<img src="./h5dbsdff.png"
> style="width:6.69305in;height:7.31319in" />

**Pulse** **Monitor:**

> Main module of the pathfinder package which initiates all the required
> modules.
>
> Contains 1 second timer to check the connectivity of the robot.
>
> This module can be used to change the disconnectivity checking
> approach as given below.

<img src="./jbv4ellm.png"
style="width:8.01875in;height:1.33542in" /><img src="./vfpmhjyn.png"
style="width:7.44306in;height:3.42986in" /><img src="./2wqbganq.png"
style="width:7.325in;height:1.57708in" />

> Contains logic

of main state machine for recovery. RECOVERYSTATUS.CHECKPULSE
RECOVERYSTATUS.RECOVERY RECOVERYSTATUS.NAVIGATE

**Pulse** **Checker:**

Publishes a header msg containing timestamp and robots ID.

Module subscribes to all header msgs coming from all the connected
robots.

If robot doesnot receive any header msgs, detect it as robot
disconnected from the network.

5 second time out is defined before going for recovery stage.

<img src="./yvknahnr.png"
style="width:7.33472in;height:1.57986in" />

**NOTE:** **Due** **to** **decentralized** **approach** **of**
**sending** **header** **msgs** **(pulses)** **affected** **by** **the**
**local** **time** **of** **each** **robot,** **a** **difference**
**between** **first** **and** **current** **readings** **are**
**considered.**

**RSSI** **Monitor:**

Used to monitor received signal strength by on board computer’s network
interface card.(Lattepanda board) This needs to be improved to work with
RAJANT router’s network interface.

Threshold for disconnectivity is defined as -70dbm

**Recovery** **Navigator:**

> Uses the the cooridinates given by recovery path calculations to send
> a goal through action client while receiving progress.
>
> Uses navigateToPose but actual orientation is neglected.

**NOTE:** **After** **goal** **suceeded,** **robot** **will** **check**
**the** **connectivity** **status** **and** **proceed** **with** **the**
**state** **machine.** **This** **helps** **to** **reduce** **back**
**and** **forth** **movements** **of** **robots** **at** **the**
**edge** **of** **the** **map.**

> **To** **further** **improve** **this** **we** **could** **us3**
> **key** **locations** **path** **to** **best** **coverage** **point**
> **and** **check** **connectivity** **location.**
>
> **(Key** **points/** **junctions** **are** **already** **calculated**
> **by** **LOSPAthfinder** **module)**

**along** **the** **at** **each**
