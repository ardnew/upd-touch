### Deprecation notice (current repository: [STUSB4500](https://github.com/ardnew/STUSB4500))

This project has been replaced with the Arduino-based library [STUSB4500](https://github.com/ardnew/STUSB4500) and is no longer maintained. It is kept for reference and for compatibility with STM32 HAL-based projects. However, any bug fixes and changes will not be merged into this repository.

This project was a demonstration for developing the device driver in pure C. Once working, the device driver was refactored into a standalone library compatible with *both* Arduino and STM32 (via a bunch of `#ifdef` macros) and was committed as [libstusb4500](https://github.com/ardnew/libstusb4500). However, providing support for both targets created unwarranted complexity (i.e. bugs). So for better general purpose support and compatibility, a conventional Arduino C++ port ([STUSB4500](https://github.com/ardnew/STUSB4500)) was selected as the final, sole target platform. Sorry STM32, still much ❤️ regardless.

----

# upd-touch
STM32 USB PD sink controller with touchscreen 
