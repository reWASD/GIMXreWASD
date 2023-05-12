# GIMX adapter firmware for reWASD

Protocol definitions for communication between reWASD and GIMX adapter with reWASD firmware.
To emulate virtual USB device for other PC or console reWASD uses the same hardware as GIMX project (see https://blog.gimx.fr),
but it flashes its own firmware tailored specifically to reWASD.
After this flashing the adapter will be able to work with reWASD only and not be compatible with GIMX project anymore - you will not need to install GIMX at all, because reWASD will communicate with the adapter directly.
Of course, if you do not need reWASD anymore, the firmware can be changed again with GIMX launcher.
The adapter is based on the pupular 'Pro Micro' board with atmega32u4 chip and USB-to-TTL converter.
You can build your own adapter using these instructions: https://gimx.fr/wiki/index.php?title=DIY_USB_adapter.
Alternatively, you can buy ready adapter from GIMX shop: https://blog.gimx.fr/product/gimx-adapter.
There can be different types of USB-to-TTL converters on the board, CP2102 is among the most popular.
The only requirement for reWASD as that it must support at least 500000 baud rate.
reWASD team wants to express special thanks to Mathieu Laurendeau for his hard work on GIMX project and developing this adapter!
Now reWASD is an alternative way to use this adapter and give more options to users.

The code is based on the original serialusb project by Mathieu Laurendeau (see https://github.com/matlo/serialusb), but was rewritten from scratch to optimize it for reWASD. 
With this firmware adapter works like USB proxy and reWASD can load any descriptors to it and emulate almost any gamepad. 
CRC checking was added to make it more robust and many other enhancements have been made, especially ISR offloading and asynchronous processing of CONTROL requests. 
It is allowed to modify this code and adapt to other architectures, provided Disc Soft FZE LLC is informed about changes and compatibility with existing firmware is maintained at the protocol level.

## Requirements

* Atmel Studio 7 with LUFA Library Extension
* GIMX Adapter
* reWASD 5.6 or higher


