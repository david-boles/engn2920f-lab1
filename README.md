# ENGN 2920F Lab 1
The software interacts with the operator through semihosting, it shows up in the OpenOCD console. CSV data is sent to the onboard UART to USB converter, I was able to dump it into a file with `cat /dev/ttyACM0 > data.csv`.

# Build/ Debug
1. Install rust toolchain: https://rustup.rs/ (follow post-install directions to source `~/.cargo/env`)
2. Configure toolchain for Cortex-M4F and M7F with hardware floating point (ARMv7E-M architecture): `rustup target add thumbv7em-none-eabihf`
3. Install the xPack OpenOCD distribution by following the instructions here: `https://xpack.github.io/openocd/install/`

You can now open the folder in VSCode and start a debugging session. To run commands manually, see the current configuration in `.vscode/launch.json` and `.vscode/tasks.json`. Note that `openocd.cfg` and `openocd.gdb` *should* work but may not be up to date.