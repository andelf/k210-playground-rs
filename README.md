# K210 Boards Playground in Rust

K210 boards examples in Rust.

- Maixduino
- Maix Bit

K210 boards almost share the same functionality.

## Prerequisite

Download RISC-V 64 GCC toolchain from <https://www.sifive.com/software>.

```sh
rustup target add riscv64gc-unknown-none-elf

pip3 install kflash
```

## How to Run

```sh
./build.sh

./flash.sh
```
