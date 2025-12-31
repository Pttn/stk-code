# SuperTuxKart 1.x Speedrunner Edition

This is a Mod of SuperTuxKart, adding some features and improvements to improve Speedrunning experience. Join us on [Speedrun.com](https://www.speedrun.com/stk) and [Discord](https://discord.gg/7VU5DNq) for discussion or support.

We assume that you know how to build STK from sources. Revisit the [original instructions](https://github.com/supertuxkart/stk-code/blob/master/INSTALL.md) if needed.

Note that the user folders (config, replays,...) are next to the binary and the assets, in order to conveniently have everything at the same place and prevent interference with possible other STK installations.

Binary for Windows (2025-12-09): https://kdrive.infomaniak.com/app/share/409092/b6621e27-2329-4993-89b8-7879c4ae9c1c

If the system complains about a missing `MSVCP140.dll` or `VCRUNTIME140.dll`, try installing the [Microsoft Visual C++ 2015 - 2022 Redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe).

## Quick Build Instructions

### Linux

Get Dependencies if needed, then

```bash
git clone https://github.com/Pttn/stk-code.git -b 1.SE
cd stk-code/Build
cmake ..
make -j 16 # Adjust Number of Threads
```

Run with

```bash
cd bin
./supertuxkart
```

### Windows 11

* Install [Visual Studio 2026 Community](https://visualstudio.microsoft.com/downloads/) and [CMake](https://cmake.org/download/)
* Download the [Source Code and Assets](https://github.com/Pttn/stk-code/archive/refs/heads/1.SE.zip) and Unzip
* Download the [Dependancies](https://github.com/supertuxkart/dependencies/releases/tag/preview) (normally, [dependencies-win-x86_64.zip](https://github.com/supertuxkart/dependencies/releases/download/preview/dependencies-win-x86_64.zip)), Unzip and put the `dependencies-win-...` Folder in the `stk-code-1.SE` from the previous step.
* Open the `Build` Folder inside `stk-code-1.SE` and then Right Click and `Open in Terminal` (or open the Terminal anywhere and Cd to that Folder)
* Run `cmake ..`
* Run `MSBuild.exe SuperTuxKart.sln`
	* If `MSBuild.exe` is not found then try instead something like `&"C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\amd64\MSBuild.exe" SuperTuxKart.sln`. You may need to look yourself for the `MSBuild.exe` file and adjust the path.
	* Append ` /t:Build /p:Configuration=Release` for a Release Build.
* Go to the `bin` folder: a `Debug` (or `Release`) one was created inside. Move the `assets` one inside that Folder. Now, you can run `supertuxkart.exe`.

## TAS

The Speedrunner Edition also includes TAS Tools for IL TASes. You can also already get started with the following instructions.

You need to use Command Line:

```bash
./supertuxkart --tas --no-start-screen --track=volcano_island_1.4 --numkarts=1 --kart=pidgin --mode=1 --tas-inputs=inputs --ghost=volcano_island_1.4_202618_1_99_7501.replay
```

This will start a Volcan Island 1.4 IL with Pidgin and replay the Inputs in the `inputs.stktas` file next to the Binary. Additionally (optional), this will race against the Ghost from File `volcano_island_1.4_202618_1_99_7501.replay`.

### Inputs

The TAS Tools simulates Keyboard Inputs, which each line corresponding to one Tick. The following letters have the following interpretation, and their presence or not meaens whether the key was pressed or not.

* `a` : **a**ccelerate
* `d` : slow **d**own
* `l`/`r` : steer **l**eft/**r**ight
* `s` : **s**kid
* `n` : **n**itro
* `f` : **f**ire
* `b` : look **b**ack
* `t` : **T**hunderbird

### Commands

Press `P` to Pause, `O` to Tick Advance, `Ctrl + Z` to set a Checkpoint (works similarly to Save States) and `Z` to return to the Checkpoint.
