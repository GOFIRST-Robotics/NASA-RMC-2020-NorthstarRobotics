Configuration
===
This project uses STM32CubeMX to generate peripheral configuration code. 
You can get this for free [here.](https://www.st.com/en/development-tools/stm32cubemx.html)
It's not strictly necessary to use this, but makes it a lot easier.

Building
===
Bare CMake is possible, but using CLion is *highly* recommended.
This is available for free for students through JetBrains. 
https://www.jetbrains.com/student/

Contributing Guidelines
===
C guidelines:
- Use explicit signed/unsigned and sized types (U32, S16, etc) from `types.h`
- Mark everything const unless necessary (write as <type> const [*] for safety)

No C++ is allowed.
Make sure to run clang-format before committing (run-format.sh).
Keep configurable parameters in header files, and keep IDs that interact in `rt_conf.h`.

If you update a system CAN interface, make sure you update README.md. 

Because this project uses STM32CubeMX, take care not to edit the auto-generated code.
If you need to change parameters or add items, use the generation program.
