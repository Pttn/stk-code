# SuperTuxKart 1.x Speedrunner Edition

This is a Mod of SuperTuxKart, adding some features and improvements to improve Speedrunning experience. Join us on [Speedrun.com](https://www.speedrun.com/stk) and [Discord](https://discord.gg/7VU5DNq) for discussion or support.

We assume that you know how to build STK from sources. Revisit the [original instructions](https://github.com/supertuxkart/stk-code/blob/master/INSTALL.md) if needed.

Note that the user folders (config, replays,...) are next to the binary and the assets, in order to conveniently have everything at the same place and prevent interference with possible other STK installations.

## Quick Build Instructions

### Linux

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

Coming Soon...
