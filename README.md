# JR Integration

Integration of JR algorithm with i3dr camera systems. 
This is a simple program for trying out the process seperate from any other systems.

## Install

Install PhobosIntegration with deb file

```bash
sudo apt install Phobos-1.0.43-x86_64.deb
or
sudo dpkg -i Phobos-1.0.43-x86_64.deb
```

Installs to default location

```bash
/usr/local/Phobos
```

## Build

Create build folder

```bash
mkdir build
cd build
```

Make program

```bash
cmake .. && make -j4
```

VSCode build tasks for quick building are provided in

```bash
.vscode/tasks.json
```

**prerun**: Used to setup the environment

**build_debug**: Build program in debug mode

**build_release**: Build program in release mode

**clean**: Clean build environment

## Run

Can be run with live camera or image file.

Change directory to build folder

```bash
cd build
```

Run test_JR from the command line with arguemnts

```bash
./test_JR [path/to/parameter_config.cfg] [path/to/left_calibration.yaml] [path/to/right_calibration.yaml]
[(OPTIONAL) path/to/left_image.png] [(OPTIONAL) path/to/right_image.png]
```

If image files are not specified will run in 'live' mode

## Debug

VSCode debug launchers are provided in

```bash
.vscode/launch.json
```

If debugging, the program should have been build in debug mode

```bash
cmake -DCMAKE_BUILD_TYPE=Debug .. && make -j4
```

Or using the build task: **build_debug**
