# Atech hardware project

This is a complete, self-contained PlatformIO project for your Atech board.
Everything needed to compile and flash is included, just keep building from here.

## Layout

- `src/main.cpp` — your generated firmware. Edit this to add features.
- `platformio.ini` — build configuration (board, libraries, flags).
- `lib/` — the Atech module + driver libraries. PlatformIO's Library
  Dependency Finder resolves these automatically; no manual setup needed.

## Build & flash

Install [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html), then from this folder run:

```sh
pio run            # compile
pio run -t upload  # compile and flash the connected board
```

## Atech SDK

You can also choose to install the Atech SDK to easily continue coding using your favourite coding agent. 

```
pip install atech
```

See [here](https://pypi.org/project/atech/) for more details. 
