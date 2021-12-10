cargo build --release
cargo objcopy --bin sim800_ups_monitor --target thumbv7m-none-eabi --release -- -O binary sim800_ups_monitor.bin
st-flash erase
st-flash write sim800_ups_monitor.bin 0x8000000
