# Clone
Clone with `git clone --recurse-submodules`
# Build
libopencm3 needs to be built first, this needs to be done only once:
```
$ cd libopencm3
$ make
$ cd ..
```
Regular workflow:
```
$ cd src
$ make
$ make flash
```
