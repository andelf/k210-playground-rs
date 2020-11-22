# Maixduino Playground in Rust

K210 board examples in Rust.

## Prerequisite

Download RISC-V 64 toolchain from <https://www.sifive.com/software>.

```sh
rustup target add riscv64gc-unknown-none-elf

pip3 install kflash
```

## How to

```sh
./build.sh

./flash.sh
```
