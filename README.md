# SuperTuxKart 1.x Speedrunner Edition

This is a Mod of SuperTuxKart, adding some features and improvements to improve Speedrunning experience.

I assume that you know how to build STK from sources. Revisit the [original repository](https://github.com/supertuxkart/stk-code/) if needed.

Note that the user folders (config, replays,...) are next to the binary and the assets, in order to have everything at the same place, avoid having to look for these folders every time and prevent interference with possible other Stk installations.

Have fun Speedrunning :D ! Join our discussion channels on [Speedrun.com](https://www.speedrun.com/stk) and [Discord](https://discord.gg/7VU5DNq)!

## Building from source

The Mod was only tested for my own system, I cannot guarantee that it works on systems other than Debian (I will also try Windows 11) and am not going to test. Feel free to make Pull Requests if you can fix something that might be broken for some other system.

### Debian

Get the source code with for example Git Clone

```bash
git clone https://github.com/Pttn/stk-code.git -b 1.SE
```

and install the dependencies listed in the original STK instructions. Then,

```bash
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
