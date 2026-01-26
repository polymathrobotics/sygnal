# Lexington Interface Layout and Procedure

## Control Device Interface Table

| Control Device | Interface | System | Control Range |
| :--- | :---: | :--- | :--- |
| MCM Sub 0 | IF 0 | Brake | 0-1 |
| | IF 1 | Throttle | 0-1 |
| | IF 2 | Steering | -1-1 |
| | IF 3 | Boom CTRL | -1-1 |
| | IF 4 | | |
| | IF 5 | | |
| | IF6 | | |
| | IF7 | | |
| | R0 | Implement Lockout Enable | 0/1 |
| MCM Sub 1 | IF 0 | - | |
| | IF 1 | - | |
| | IF 2 | FNR #1 | -1/0/1 |
| | IF 3 | Bucket CTRL | -1-1 |
| | IF 4 | FNR #2 | -1/0/1 |
| | IF 5 | | |
| | IF6 | | |
| | IF7 | | |
| | R1 | Implement Lockout Engage | 0/1 |
| IO1 | IF0 | Gear Shifter | -1/0/1 |
| | IF1 | | |
| | IF2 | | |
| | IF3 | | |
| MVEC | R1 | FR Wiper Wash | 0/1 |
| | R2 | FR Wiper LS | 0/1 |
| | R3 | Accessory Power | 0/1 |
| | R4 | Horn | 0/1 |
| | R5 | Hazards | 0/1 |
| | R6 | Parking Brake Enable | 0/1 |
| | R7 | Engine Start | 0/1 |
| | R8 | Parking Brake Engage | 0/1 |

## Procedures

* **Vehicle power up** – Turn on R3 to turn on Accessory power.
* **Vehicle engine start** – Turn on R7 for a few seconds then turn it off again.
* **Vehicle operations** – Turn on R4 to defeat the armrest.
* **Implement Lockout** – Turn on R0 on the MCM to enable switching. Then use R1 on the MCM for toggling the state.
* **Parking Brake** – Turn on R6 to enable switching. Then use R8 for toggling the state.
* **FNR Control** – FNR #1 and FNR #2 must be commanded to the same value for control. This may change once some experimentation is done.
