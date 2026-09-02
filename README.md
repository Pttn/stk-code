# SuperTuxKart 1.x Speedrunner Edition

This is a Mod of SuperTuxKart, adding some features and improvements to improve Speedrunning experience. Join us on [Speedrun.com](https://www.speedrun.com/stk) and [Discord](https://discord.gg/7VU5DNq) for discussion or support.

We assume that you know how to build STK from sources. Revisit the [original instructions](https://github.com/supertuxkart/stk-code/blob/master/INSTALL.md) if needed.

Note that the user folders (config, replays,...) are next to the binary and the assets, in order to conveniently have everything at the same place and prevent interference with possible other STK installations.

[Binary for Windows, 2026-09-03](https://moeverse.xyz/stk/files/STK1.SE_Win64_260903.zip). If the system complains about a missing `MSVCP140.dll` or `VCRUNTIME140.dll`, try installing the [Microsoft Visual C++ 2015 - 2022 Redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe).

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

* Get the correct variant of the [Llvm Compiler](https://github.com/mstorsjo/llvm-mingw/releases/latest) (normally, `msvcrt-x86_64`), extract the contained Folder somewhere, as let's say `C:\llvm-mingw`.
* Get [Ninja](https://github.com/ninja-build/ninja/releases/latest) (normally, [ninja-win.zip](https://github.com/ninja-build/ninja/releases/latest/download/ninja-win.zip)), extract the `ninja.exe` inside to `C:\llvm-mingw`.
* Download the [Source Code and Assets](https://github.com/Pttn/stk-code/archive/refs/heads/1.SE.zip) and extract it somewhere.
* Download the [Dependencies](https://github.com/supertuxkart/dependencies/releases/tag/preview) (normally, [dependencies-win-x86_64.zip](https://github.com/supertuxkart/dependencies/releases/download/preview/dependencies-win-x86_64.zip)), extract and put the `dependencies-win-...` Folder in the `stk-code-1.SE` from the previous step.
* Get [CMake](https://cmake.org/download/) and install it.
* Launch Cmake (Gui) and do the following in the Interface:
	* Set the `stk-code-1.SE` location for "Where is the source code".
	* Set the location of the `Build` Folder inside `stk-code-1.SE` for "Where to build the binaries".
	* With the "Add Entry" Button, add the following:
		* `LLVM_ARCH`: `STRING` of Value `x86_64`
		* `LLVM_PREFIX`: `STRING` of Value `C:/llvm-mingw`
		* `CMAKE_MAKE_PROGRAM`: `STRING` of Value `C:/llvm-mingw/ninja.exe`
		* `USE_WIIUSE`: `BOOL` of Value False (Unchecked)
	* Press the "Configure" Button.
		* Confirm the creation of a Directory.
		* Choose Ninja for "Specify the generator for this project" and go Next.
		* Set the Toolchain File `stk-code-1.SE\cmake\Toolchain-llvm-mingw.cmake` and Finish.
	* Press the "Generate" Button.
* Open the `Build` Folder inside `stk-code-1.SE` and then Right Click and `Open in Terminal` (or open the Terminal anywhere and Cd to that Folder)
	* Run `C:\llvm-mingw\ninja.exe`
* Copy the Dll Files from `C:\llvm-mingw\x86_64-w64-mingw32\bin` to `stk-code-1.SE\Build\bin` in which `supertuxkart.exe` should have been produced.

Now, you can run `supertuxkart.exe`.

## TAS

The Speedrunner Edition also includes TAS Tools for IL TASes. You can get started with the following instructions.

You need to use Command Line:

```bash
./supertuxkart --tas --no-start-screen --track=volcano_island_1.4 --numkarts=1 --kart=pidgin --mode=1 --tas-inputs=inputs --ghost=volcano_island_1.4_202618_1_99_7501.replay
```

This will start a Volcan Island 1.4 IL with Pidgin and replay the Inputs in the `inputs.stktas` file next to the Binary. Additionally (optional), this will race against the Ghost from File `volcano_island_1.4_202618_1_99_7501.replay`.

### Inputs

The TAS Tools simulates Keyboard Inputs, which each line corresponding to one Tick. The following letters have the following interpretation, and their presence or not means whether the key was pressed or not.

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
