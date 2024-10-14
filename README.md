# SuperTuxKart 1.4 TAS Mod

This is a Mod of SuperTuxKart, adding some tools to it in order to perform TASes.

I assume that you know how to build STK from sources. Revisit the original repository if needed - https://github.com/supertuxkart/stk-code/

Take a look at [TAS Resources](https://moeverse.xyz/stk/tas/) to learn how to TAS SuperTuxKart.

## Building from source

The Mod was only tested for my own system, I cannot guarantee that it works on systems other than Linux and am not going to test. Feel free to make Pull Requests if you can fix something that broke for some other system.

### Retrieve Remaining Assets

There is a partial asset folder in `Build/bin/assets` as the Code was written so that important files are all put together at a single place. You can download the 7 missing folders of the original 1.4 Assets [here](https://moeverse.xyz/stk/tas/stk-assets1.4TAS.zip), put all of them (`karts`, `library`, ...) in `Build/bin/assets` before building.

### Build

Get the source code with for example Git Clone

```bash
git clone https://github.com/Pttn/stk-code.git -b 1.4TAS
```

and install the dependencies listed in the original STK instructions. Then,
 
```bash
# Go into the stk-code/Build directory with cd (or open a terminal in the correct folder)
cd stk-code/Build
 
# Run cmake to generate the Makefile
cmake ..
 
# Compile (to use more threads for compilation, do for example "make -j 8" to use 8 threads
make
```

## Executing this Mod

In order to do so, the assets folder needs to be next to the binary, which should automatically be the case in Linux.

```bash
cd bin
./supertuxkart
```

Note that the user folders (config, replays,...) are next to the binary and the assets, in order to have everything at the same place and avoid having to look for these folders every time.

You can now start TASing. I am looking forward to seeing your TASes!
