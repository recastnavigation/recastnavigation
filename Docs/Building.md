# Building


## Building RecastDemo

RecastDemo uses [premake5](http://premake.github.io/) to build platform specific projects. Download it and make sure it's available on your path, or specify the path to it.

### Linux

- Install SDL2 and its dependencies according to your distro's guidelines.
- Navigate to the `RecastDemo` folder and run `premake5 gmake2`
- Navigate to `RecastDemo/Build/gmake2` and run `make`
- Navigate to `RecastDemo/Bin` and run `./RecastDemo`

### macOS

- Grab the latest SDL2 development library dmg from [here](https://github.com/libsdl-org/SDL) and place `SDL2.framework` in `RecastDemo/Bin`
- Navigate to the `RecastDemo` folder and run `premake5 xcode4`
- Open `Build/xcode4/recastnavigation.xcworkspace`
- Set the RecastDemo project as the target and build.

### Windows

- Grab the latest SDL2 development library release from [here](https://github.com/libsdl-org/SDL) and unzip it `RecastDemo\Contrib`.  Rename the SDL folder such that the path `RecastDemo\Contrib\SDL\lib\x86` is valid.
- Navigate to the `RecastDemo` folder and run `premake5 vs2022`
- Open `Build/vs2022/recastnavigation.sln`.
- Set `RecastDemo` as the startup project, build, and run.

## Running Unit tests

- Follow the instructions to build RecastDemo above.  Premake should generate another build target called "Tests".
- Build the "Tests" project.  This will generate an executable named "Tests" in `RecastDemo/Bin/`
- Run the "Tests" executable.  It will execute all the unit tests, indicate those that failed, and display a count of those that succeeded.
