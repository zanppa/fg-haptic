Fg-Haptic
=========

Force feedback (haptic) support for Flight Gear flight simulator
Version 0.1.1
Copyright Lauri Peltonen, 2011

This program is released under GPLv2 or later.


Description
-----------

This force feedback support consists of 4 components:
- fg-haptic or fg-haptic.exe
- force-feedback.xml
- force-feedback.nas
- ff-protocol.xml

fg-haptic or fg-haptic.exe is the main program, which
supports force feedback devices through SDL 1.3 library.

force-feedback.xml is a configuration utility which resides
inside Flight Gear's data directory.

force-feedback.nas is a wrapper between different flight
dynamics models etc. and the force feedback effects.

ff-protocol.xml is a generic IO protocol which fg-haptic
uses to communicate with flight Gear.

This program has been tested with following devices:

- Saitek Cyborg Evo Force (Linux)
  * Does not support updating effects while running
    which makes pilot/surface forces feel bad
  * All effects supported
  * Needs special kernel patch (for kernel 3.0.0)

- Gametech "Twin USB Joystick" gamepad with rumble effect (Linux)
  * Supports only stick shaker, if gain & strength are around 1.0
  * Does not have axis (so 1?) but claims to have 2

- Logitech Driving Force GT wheel (Linux)
  * Supports only constant forces
  * Really has 1 axis only, but calims to have 2

- Logitech Wingman Force 3D (Linux)
  * Supports only constant forces

- Microsoft Sidewinder Force Feedback 2 (Linux)
  * Supports numerous modes including constant force and stick shaker


Building
--------

Required libraries:
- libsdl2-dev
- libsdl2-net-dev

fg-haptic should build by running:

    make fg-haptic


For windows, you might need to set EXE=.exe also, i.e.:

    make fg-haptic EXE=.exe



Installation
------------

fg-haptic(.exe) can reside anywhere on your system.

force-feedback.xml must be copied into gui/dialogs/ directory
inside Flight Gear's data directory.

force-feedback.nas must be copied into Nasal/ directory
inside Flight Gear's data directory.

ff-protocol.xml must be copied into Protocol/ directory
inside Flight Gear's data directory.



Running
-------

Launch flightgear with at least the following options:

    fgfs --telnet=5401 --generic=socket,out,20,localhost,5402,udp,ff-protocol

Then run ```fg-haptic(.exe)```.

You can also launch them in different order (fg-haptic first). Also you can 
replace localhost with any other address, fg-haptic tries to determine the
FlightGear's address from the first received packet.

If FlightGear's menu is not visible, press F10 to display it.
In the Help menu you can find Force feedback options.
Tune them to your liking and fly!


To test force feedback effects, run

```fg-haptic --test```    or  ```fg-haptic -t```

which tests all effects on all connected joysticks.
