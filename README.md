## STM32F1 Bare-Metal Programming in D

This project provides a basic bare-metal framework for working with STM32F1 microcontrollers using the D programming language. It is fully functional and includes the following features:

- **Basic C Library**: A minimal C library is included to support essential operations;
- **No Assembly Required**: The project is written entirely in D, with no assembly code needed;
- **Configurable `reset_handler` and `init` Function**: The reset handler and initialization function can be customized to suit your needs;
- **Rewritten ST's Peripheral Register Library**: The peripheral register library has been rewritten in D for better integration and ease of use.

This little project allows you to write efficient and maintainable bare-metal code for STM32F1 microcontrollers using the powerful features of the D programming language.

## Getting Started

### Prerequisites

To get started, you will need the LDC compiler. You can install it using the following methods:

- **On Arch**:
    ```sh
    sudo pacman -Sy ldc
    ```

- **On Ubuntu**:
    ```sh
    sudo apt-get install ldc
    ```

- **On macOS**:
    ```sh
    brew install ldc
    ```

- **On Windows**:
    Download the installer from the [LDC releases page](https://github.com/ldc-developers/ldc/releases) and follow the installation instructions.

Additionally, you will need the `arm-none-eabi-gcc` toolchain for building and flashing the project. You can install it using the following methods:

- **On Arch**:
    ```sh
    sudo pacman -Sy arm-none-eabi-gcc
    ```

- **On Ubuntu**:
    ```sh
    sudo apt-get install gcc-arm-none-eabi
    ```

- **On macOS**:
    ```sh
    brew install arm-none-eabi-gcc
    ```

- **On Windows**:
    Download the installer from the [ARM GNU Toolchain page](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) and follow the installation instructions.

### Setting Up the Development Environment

For debugging, you can use CLion or VSCode with PlatformIO. Follow these steps to install PlatformIO:

- **In CLion**:
    1. Open CLion and go to `File` > `Settings` > `Plugins`.
    2. Search for "PlatformIO" and install the plugin.
    3. Restart CLion.

- **In VSCode**:
    1. Open VSCode and go to the Extensions view by clicking on the Extensions icon in the Activity Bar on the side of the window.
    2. Search for "PlatformIO IDE" and install the extension.
    3. Restart VSCode.

### Building and Flashing the Project

Once everything is set up, you can build and flash the project using the following commands in the terminal:

- To build the project:
    ```sh
    make
    ```

- To flash the project to the device:
    ```sh
    make flash
    ```

By default, CMSIS-DAP is used for flashing, but you can configure other methods in the `platformio.ini` file.

### Debugging

To debug the project, perform the following steps:

1. Run the build and flash commands:
     ```sh
     make && make flash
     ```

2. Open the PlatformIO debugger:
    - In CLion, go to `Run` > `Debug` (this option has not been tested yet).
    - In VSCode, go to `Run and Debug` > `PIO Debug (skip Pre-Debug)`.

This will start the debugging session, allowing you to step through your code and inspect variables.

### Acknowledgements

This project uses parts of the code from the D standard library, STM32F1 peripheral libraries, and code from the user [shima-529/stm32OnDlang](https://github.com/shima-529/stm32OnDlang). I thank them for their work.
