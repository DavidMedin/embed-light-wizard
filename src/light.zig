const arduino = @cImport(
    @cInclude("arduino.h")
);
pub const Color = struct {
    r: u8,
    g: u8,
    b: u8
};

// Waits for `ns` number of nanoseconds to pass.
// Note: this will only wait the correct amount of time on the
// Arduino Uno R4 WIFI! This works by executing a large number of
// no-op (nop) instructions determined by the clock speed!
pub inline fn delay(comptime ns : usize) void {
    // The Uno R4 WIFI has a Ra4m1 (arm cortex-m4) cpu, which has
    // a 48 MHz frequency clock.
    // But delay is a measure of period, not frequency.
    // 48 MHz = 20.83333 ns
    // So our resolution is 20.8333 nanoseconds.
    @setEvalBranchQuota(10000);
    inline for (@as(comptime_int, @divFloor(@as(comptime_float, ns),  20.833333333333) )) |_| {
        asm volatile("nop");
    }
}

pub inline fn show_color(comptime color : Color) void {
    // R
    send_byte(color.r);
    // G
    send_byte(color.g);
    // B
    send_byte(color.b);
}

pub inline fn send_byte(comptime byte : u8) void {
    inline for(0..8) |i| { // 8 bits in a byte
        const mark = 0b10000000;
        const func = comptime blk: {
            const bit = byte & (mark >> i);
            if ( bit > 0 ) {
                break :blk send_one;
            }else {
                break :blk send_zero;
            }
        };
        func();
    }
}

// it takes time to set a pin high or low. it is about `write_offset` nanoseconds.
const write_offset = 300;
pub inline fn set_high() void {
    //https://forum.arduino.cc/t/digitalwritefast-with-uno-r4/1145206/13
    const posr : usize = 0x4004008a; // Address prob only works for Arduino Uno R4 WIFI
    const posr_ptr : *u16 = @ptrFromInt(posr);
    posr_ptr.* = 1024;
}

pub inline fn set_low() void {
    const porr : usize align(16) = 0x40040088;
    const porr_ptr : *u16 = @ptrFromInt(porr);
    porr_ptr.* = 1024;
}

pub inline fn send_zero() void {
    // Will send a zero over pin 12.
    // put high
    set_high();
    // wait 400 ns (.4 us)
    delay(400 - write_offset);
    // put low
    set_low();
    // wait 850 ns (.85 us)
    delay(850 - write_offset);
}

pub inline fn send_one() void {
    // Will send a one over pin 12.
    // put high
    set_high();

    // wait 800 ns (.8 us)
    delay(800);
    // put low
    set_low();
    // wait 450 ns (.45 us)
    delay(450);
}

pub inline fn send_reset() void {
    arduino.wait_us(300);
}
