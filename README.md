Fg-Haptic
=========

Force feedback (haptic) support for Flight Gear flight simulator

Version 1.0.0

Copyright (C) Lauri Peltonen, 2011, 2020, 2025

This program is released under GPLv2 or later.


Description
-----------

This force feedback support consists of 4 components:
- fg-haptic or fg-haptic.exe
- gui/dialogs/force-feedback.xml and force-feedback-aircraft.xml
- Nasal/force_feedback.nas
- Protocol/ff-protocol.xml

fg-haptic or fg-haptic.exe is the main program, which
supports force feedback devices through SDL 2 library.

force-feedback.xml and force-feedback-aircraft.xml are configuration 
dialogs to modify force feedback device settings and aircraft effects setup.

force_feedback.nas calculates all the effect forces based on the 
flight dynamiocs models etc.

ff-protocol.xml is a generic IO protocol which fg-haptic
uses to communicate with Flight Gear.

This program has been tested with following devices:

- Saitek Cyborg Evo Force (Linux)
  * All effects supported
  * Needs special [kernel patch](https://github.com/zanppa/hid-pidff/)

- Saitek Cyborg Evo Force (Windows 10, 64 bit)
  * Joystick driver does not work properly. Even Saitek's own force tester in configuration crashes.

- Gametech "Twin USB Joystick" gamepad with rumble effect (Linux)
  * Supports only stick shaker, if gain & strength are around 1.0
  * Does not have axis (so 1?) but claims to have 2

- Logitech Driving Force GT wheel (Linux)
  * Supports only constant forces
  * Really has 1 axis only, but claims to have 2

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



Prerequirements
---------------

SDL2 and SDL2-net runtimes must be available for fg-haptic to work.

Linux: Install `libsdl2` and `libsdl2-net`.

Windows: Install [SDL2 runtime](https://github.com/libsdl-org/SDL/releases) (search for 2.x.y version) or [SDL2 compat](https://github.com/libsdl-org/sdl2-compat/releases) with SDL3.dll and [SDL2_net runtime](https://github.com/libsdl-org/SDL_net/releases) dlls to the same directory as fg-haptic.exe.



Installation
------------

fg-haptic is now packaged as an add-on. This means that installation should be rather 
straightforward without needing to copy (many) files around.

Clone the repository or donwload it as a zip file an unzip it to some location. If necessary (Linux), 
build the program.

In FlightGear's launcher, select Add-ons and add the location where you put the files as an add-on. 

Optionally, add following command line option to FlightGear:

    --addon=/path/to/fg-haptic

Note! Since FG addons do not yet support protocol file a manual copy must still be done:

Protocol/ff-protocol.xml must be copied into **Protocol/** directory
inside Flight Gear's data directory.



Manual installation
-------------------

fg-haptic(.exe) can reside anywhere on your system.

gui/dialogs/force-feedback.xml and force-feedback-aircraft.xml must be copied into **gui/dialogs/** directory
inside Flight Gear's data directory.

Nasal/force_feedback.nas must be copied into **Nasal/** directory
inside Flight Gear's data directory.

Protocol/ff-protocol.xml must be copied into **Protocol/** directory
inside Flight Gear's data directory.

Note that after manual installation, the ```--addon``` command line argument is not needed.


Running
-------

Launch flightgear with the addon enabled:

    fgfs --addon=/path/to/fg-haptic

This now automatically starts the telnet and generic protocols for communication. 
For manual installation, it should be enough to just normally launch Flight Gear without 
any additional arguments.

Then run ```fg-haptic(.exe)```.

You can also launch them in different order (fg-haptic first). Also you can 
replace localhost with any other address, fg-haptic tries to determine the
FlightGear's address from the first received packet.

fg-haptic expects telnet to be at port 5401 and receives generic IO on port
5402. These are currently hard coded. Note that compared to previous versions,
fg-haptic now uses UDP so fg-haptic can be launched/restarted while Flight Gear
is running.

Previously one needed to enable the protocols separately using following command line 
which is here for reference:

    fgfs --telnet=5401 --generic=socket,out,20,localhost,5402,udp,ff-protocol


If FlightGear's menu is not visible, press F10 to display it.
In the Help menu you can find Force feedback options.
Tune them to your liking and fly!

To test force feedback effects, run

```fg-haptic --test```    or  ```fg-haptic -t```

which tests all effects on all connected joysticks. There is also a simple test
implemented in the dialog inside Flight Gear.


Effects
-------

There are currently following effects implemented:
- Pilot forces. Any force (acceleration) on the pilot can be transferred to the
force feedback device.'
- Stick forces. Basic effect is speed dependent stiffening of the joystick. Other
effects include stick shaker when the aircraft is about to stall and stick pusher
if aircraft AoA is too high.
- Ground rumble. A periodic "click" when speeding up on a runway.
- Force feedback trim, which changes the center location of the joystick depending
on the trim value.
