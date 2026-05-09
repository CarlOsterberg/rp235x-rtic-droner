# Hello world on rp pico 235x

This program writes "Hello RTIC UART" over UART on Gpio0/pin1.
The UART settings are: baud rate 115200, 8 bits, no parity and 1 stop bit.

### Installing dependencies

```sh
rustup target install thumbv8m.main-none-eabihf
cargo install flip-link
cargo install probe-rs-tools # If debugging with SWD
```

## Flashing and running the code

There are two ways, either you can use the picotool or probe-rs. The pciotool
requires less tools but you need to put the device into mass storage mode somehow.
probe-rs is easier to use but you need a debugger supporting swd and to solder the debug pins.
For details see the [cargo config](.cargo/config.toml).

## License

The contents of this repository are dual-licensed under the _[MIT](LICENSE-MIT) OR [Apache-2.0](LICENSE-APACHE-2.0)_ License. That means you can chose either the MIT licence or the
Apache-2.0 licence when you re-use this code. See [`LICENSE-MIT`](LICENSE-MIT) or [`LICENSE-APACHE-2.0`](LICENSE-APACHE-2.0) for more
information on each specific licence.
