# Build

User instructions: [introduction.md, Requirements](../stmbldoc/content/docs/introduction.md).

## Toolchain

`arm-none-eabi-gcc` from `PATH`. On Debian or Ubuntu:

```sh
sudo apt-get install -y --no-install-recommends gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi make python3
```

`--no-install-recommends` skips `libstdc++-arm-none-eabi-newlib`, several hundred MB that the firmware does not need.

Another install location: `make TOOLCHAIN=/path/to/bin/arm-none-eabi- ...`. Pass it on the command line; don't create or edit `toolchain-user.mak` in an agent session (it is the user's local override and is ignored by git).

## Targets

| Command | Builds |
|---|---|
| `make -j16 DOCS=0` | F4 app, F4 bootloader, F3 bootloader, F3 app (embedded in the F4 app) |
| `make -j16` | the same plus the wiki pages and graphs (needs graphviz: `apt install graphviz` and `pip install graphviz`) |
| `make docs` | only the generated wiki pages |
| `make f3` | only the F3 app |
| `make binall` | `.dfu` files for DfuSe / manual dfu-util |
| `make format` | clang-format over the C sources |
| `make clean` | removes build output |

Flashing targets are in [flashing.md](flashing.md).

## Check after every build

- The size lines at the end. The F3 (`hvf3.elf`, STM32F303CB) is close to full in flash and in its 8 KB CCM RAM (rt code runs from CCM). A link error like `region 'CCMRAM' overflowed` or `region 'FLASH' overflowed` is a size problem, not a code bug: look at what grew.
- No new warnings in files you touched.

## GCC version check (upstream)

Upstream builds with **GCC 12.2** (the Debian bookworm package). Code size changes with the compiler, so a change that fits with a newer GCC can overflow the F3 with 12.2, and the other way round. The `Firmware build` GitHub Actions workflow (`.github/workflows/firmware.yml`) builds every push and pull request with it. To check locally before pushing:

```sh
docker run --rm -v "$PWD":/src -w /src debian:bookworm bash -c '
  apt-get update -qq &&
  apt-get install -y -qq --no-install-recommends gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi make python3 git >/dev/null &&
  git config --global --add safe.directory /src &&
  arm-none-eabi-gcc --version | head -1 &&
  make -j16 DOCS=0 && make clean >/dev/null'
```

The firmware is also known to build with GCC 13.2 and 14.2. Report the GCC version with any size figure.
