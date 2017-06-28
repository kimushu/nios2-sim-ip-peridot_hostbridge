# nios2-sim-ip-peridot_hostbridge

`peridot_hostbridge` IP simulator for nios2-sim

## Features supported

* SWI CSRs (Class ID, Unique ID, Message and SWI)
* Serial port emulation
  * Avalon-MM transaction
  * EEPROM emulation (with ver.2 header)
  * AS mode

## Features NOT supported

* PS mode support
* CPU reset
* SPI flash access

## Options

|Option|Description|
|--|--|
|`--swi-port <path>`|**Serial port name**<br>For Windows: COMxx<br>For mac/Linux: /dev/xxxx|
|`--swi-bid <str>`|**Board ID**<br>One of `J72A`, `J72N`, `J72B` or `J72X`|
|`--swi-sid <str>`|**Serial ID**<br>18-char serial ID (hyphens accepted):<br>`XXXXXX-XXXXXX-XXXXXX`|
|`--swi-uid <hex>`|**64-bit unique ID**<br>(hexadecimal)|
|`--swi-image <image>`|**Configuration image**<br>One of `boot` or `user`|
