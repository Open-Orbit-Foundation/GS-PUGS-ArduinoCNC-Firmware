# Dual-Band SatCom Ground Station Firmware
## File Structure
`./MASTER.../` is a GitLab cloned repo with the SatNOGS tutorial firmware
- https://wiki.satnogs.org/SatNOGS_Arduino_Uno/CNC_Shield_Based_Rotator_Controller
- https://gitlab.com/Quartapound/satnogs-rotator-firmware

`./SOURCE.../` is the ACTIVE firmware being used on the Ground Station, adapted with PlatformIO tools for build and embeded package management

## Important Links
ROTCTLD Docs
- https://hamlib.sourceforge.net/html/rotctld.1.html


## Remote Machine Setup

1) Open the ROTCTLD service remotely
```
rotctld -m 202 -r /dev/ttyACM0 -s 9600 -T 127.0.0.1 -t 4533 -C  timeout=500 retry=0 -vvvvvvvv > pause
```

2) Forward remote TCP port 4533 to local machine
```
ssh -L 4544:localhost:4533 agsat-admin@10.1.225.44
```
*Remote machine IP may not be static*

with a bash shell on the local machine, install the hamlib-utils package
```
apt install libhamlib-utils
```

then command the rotator with 
```
rotctl -m 2
``` 

where machine #2 is the NET remote  


OPEN ORBIT GROUND SYSTEMS
