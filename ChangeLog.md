DeltaKinematics ChangeLog
================

v0.1.0
======

Release date: 2017-07-01

Major changes and added features:

* Add support for STM32L452 target ([#608](https://github.com/texane/stlink/pull/608))
* Initial support to compile with Microsoft Visual Studio 2017 ([#602](https://github.com/texane/stlink/pull/602))
* Added support for flashing second bank on STM32F10x_XL ([#592](https://github.com/texane/stlink/pull/592))
* Add support for STM32L011 target ([#572](https://github.com/texane/stlink/pull/572)) 
* Allow building of debian package with CPack (@xor-gate)

Updates and fixes:

* Fix compilation with GCC 7 ([#590](https://github.com/texane/stlink/pull/590))
* Skip GTK detection if we're cross-compiling ([#588](https://github.com/texane/stlink/pull/588))
* Fix possible memory leak ([#570](https://github.com/texane/stlink/pull/570))
* Fix building with mingw64 ([#569](https://github.com/texane/stlink/pull/569), [#610](https://github.com/texane/stlink/pull/610))
* Update libusb to 1.0.21 for Windows ([#562](https://github.com/texane/stlink/pull/562))
* Fixing low-voltage flashing on STM32F7 parts. ([#567](https://github.com/texane/stlink/pull/567))
* Update libusb to 1.0.21 for Windows ([#562](https://github.com/texane/stlink/pull/562))
