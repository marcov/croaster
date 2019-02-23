## croaster

**croaster** is a fork of the TC4 aArtisan (PID) firmware for the Arduino Uno,
ak.a. [aArtisan_PID](https://github.com/greencardigan/TC4-shield/tree/master/applications/Artisan/).

It can used together with [Artisan](https://github.com/artisan-roaster-scope/artisan)
to log, control and automate coffee roasting.

The initial goal of this fork is to refactor and clean up the original aArtisan
code.

## Initial setup
You need:
-  [PlatformIO Core](https://platformio.org/install/cli)


## Building and flashing the firmware
From project root:
- Build the firmware with `make`
- Upload the firmware with `make upload`.

Refer to PlatformIO documentation for more information on customizing the build
environment (platformio.ini file).

### More Information

aArtian_PID was forked at commit hash [3c2cd2198db1f949b316c987c3238c8a08a7b6cd](https://github.com/greencardigan/TC4-shield/commit/3c2cd2198db1f949b316c987c3238c8a08a7b6cd).
