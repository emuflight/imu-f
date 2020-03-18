# IMU-F

IMU-F flight controller project

## Development

## Build

 * clone the project
 * install [toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
 * ensure `gnu-rm` executables come first in `PATH`
 * call `make.py`

## Build on OSX

```bash
brew install direnv openocd python
curl -OL "https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-mac.tar.bz2"
tar xzf gcc-arm-none-eabi-9-2019-q4-major-mac.tar.bz2
direnv allow
./make.py -T=F3
```
