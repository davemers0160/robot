# DECAWAVE DWM1001 Configuration
This files shows how to configure the Decawave DWM1001 to be a Tag , Anchor Initiator or an Anchor

The basic requirements are a computer and putty (or other terminal program)

1. Connect the DWM1001 to your computer and open up putty

2. Configure the connection to the correct port and set the following
   Baudrate: 115200
   Data Bits: 8
   Stop Bits: 1   
   Parity: None

3. Open the connection

4. Hit "Enter" twice to enter UART shell mode and you should see something like this:

```
 DWM1001 TWR Real Time Location System

 Copyright :  2016-2019 LEAPS and Decawave
 License   :  Please visit https://decawave.com/dwm1001_license
 Compiled  :  Jun  7 2019 18:00:03

 Help      :  ? or help

dwm>
```

5. Enter the following command to set the panid:

```
nis 0x1234
```

## TAGS                                      

To set up the DWM1001 as a "Tag" enter the following:

```
nmt
```

The UART shell mode must be re-entered as described previously, by sending “enter” twice.  Enter the following command to verify the change:

```
si
```

You should see something like the following, with the mode set to "tn":

```
[000351.430 INF] sys: fw2 fw_ver=x01030001 cfg_ver=x00010700
[000351.430 INF] uwb0: panid=x1234 addr=xDECAB3C1C3B25506
[000351.440 INF] mode: tn (act,twr,np,le)
[000351.440 INF] uwbmac: disconnected
[000351.440 INF] uwbmac: bh disconnected
[000351.450 INF] cfg: sync=0 fwup=0 ble=1 leds=1 le=1 lp=0 stat_det=1 (sens=1) mode=0 upd_rate_norm=1 upd_rate_stat=100 label=DW5506
[000351.460 INF] enc: off
[000351.460 INF] ble: addr=DD:92:94:CB:02:BB
```

This completes the setup for the DWM1001 as a "Tag".  You can close down putty and disconnect the DWM1001.

## ANCHOR INITIATOR                               

To set up the DWM1001 as an "Anchor Initiator" enter the following (only one initiator is required per network):

```
nmi
```

The UART shell mode must be re-entered as described previously, by sending “enter” twice.  Enter the following command to verify the change:

```
si
```

You should see something like the following, with the mode set to "ani":

```
[000021.220 INF] sys: fw2 fw_ver=x01030001 cfg_ver=x00010700
[000021.220 INF] uwb0: panid=x1234 addr=xDECA4FF36A538493
[000021.230 INF] mode: ani (act,real)
[000021.230 INF] uwbmac: connected
[000021.230 INF] uwbmac: bh disconnected
[000021.240 INF] cfg: sync=0 fwup=0 ble=1 leds=1 init=1 upd_rate_stat=120 label=DW8493
[000021.250 INF] enc: off
[000021.250 INF] ble: addr=E4:E4:D3:DC:CE:0B
```

This completes the setup for the DWM1001 as a "Anchor Initiator".  You can close down putty and disconnect the DWM1001.

## ANCHOR                                     

To set up the DWM1001 as an "Anchor" enter the following:

```
nma
```

The UART shell mode must be re-entered as described previously, by sending “enter” twice.  Enter the following command to verify the change:

```
si
```

You should see something like the following, with the mode set to "an":

```
[000010.240 INF] sys: fw2 fw_ver=x01030001 cfg_ver=x00010700
[000010.240 INF] uwb0: panid=x1234 addr=xDECA1207F8B150AE
[000010.250 INF] mode: an (act,-)
[000010.250 INF] uwbmac: disconnected
[000010.250 INF] uwbmac: bh disconnected
[000010.260 INF] cfg: sync=0 fwup=0 ble=1 leds=1 init=0 upd_rate_stat=120 label=DW50AE
[000010.260 INF] enc: off
[000010.270 INF] ble: addr=D7:5F:07:EF:F3:82
```

This completes the setup for the DWM1001 as a "Anchor".  You can close down putty and disconnect the DWM1001.
