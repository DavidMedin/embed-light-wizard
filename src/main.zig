// File: main.zig
// Author: David Medin
// Date: 07/22/2024
// An Arduino program written in Zig for the Arduino Uno R4 Wifi.
// Operates WS2812B LED lights.

// WS2812B specs : https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
// Guide on WS2812B : https://www.arrow.com/en/research-and-events/articles/protocol-for-the-ws2812b-programmable-led
// https://www.suntechleds.com/info/how-does-the-ws2812b-led-light-work-55934182.html
// A library to use for the Zig+GPIO : https://github.com/Elara6331/zig-gpio

const std = @import("std");
const light = @import("light.zig");
const arduino = @cImport(
    @cInclude("arduino.h")
);

// Ported from Arduino's Common.h
const PinMode = enum(i32){
  INPUT            = 0x0,
  OUTPUT           = 0x1,
  INPUT_PULLUP     = 0x2,
  INPUT_PULLDOWN   = 0x3,
  OUTPUT_OPENDRAIN = 0x4,
};

var GLOBAL_FBA : ?std.heap.FixedBufferAllocator = null;
var GLOBAL_ALLOCATOR: ?std.mem.Allocator = null;
const buffer_size = 1024;
var buffer : [buffer_size]u8 = undefined;
fn print(comptime format: []const u8, args: anytype) void {
    const fmtd_string: [:0]u8 = std.fmt.allocPrintZ(GLOBAL_ALLOCATOR.?, format, args) catch unreachable;
    _ = nosuspend arduino.println(@ptrCast(fmtd_string));
    GLOBAL_ALLOCATOR.?.free(fmtd_string);
}


const data_pin = 12;
export fn setup() void {
    // setup printing
    GLOBAL_FBA = std.heap.FixedBufferAllocator.init(&buffer);
    GLOBAL_ALLOCATOR = GLOBAL_FBA.?.allocator();

    // setup serial and pins
    arduino.serial_begin(115200);
    arduino.pin_mode(data_pin,@intFromEnum( PinMode.OUTPUT ));

    // do one-off work
    // do_light_things();
}


// fn hue_to_rgb(p : f32, q: f32, t : f32) f32 {
//     if(t < 0){
//         t += 1;
//     }
//     if (t > 1){
//         t -= 1;
//     }
//     if (t < 1.0/6.0){
//         return p + (q - p) * 6 * t;
//     }

//     if (t < 1.0/2.0){
//         return q;
//     }

//     if (t < 2.0/3.0) {
//         return p + (q - p) * (2.0/3.0 - t) * 6;
//     }

//     return p;

// }
// comptime fn hsl_to_rgb(h : f32, s : f32, l : f32) light.Color {
//     var r : f32 = 0.0;
//     var g : f32 = 0.0;
//     var b : f32 = 0.0;

//     if (s == 0.0) {
//         r = l;
//         g = l;
//         b = l;
//     }else {
//         const q = if (l < 0.5) l * ( 1 +  s) else l + s - l * s;
//         const p = 2 * l - q;
//         r = hue_to_rgb(p,q,h + 1.0/3.0);
//         g = hue_to_rgb(p,q,h);
//         b = hue_to_rgb(p,q,h - 1.0/3.0);
//     }
//     return .{.r = std.math.round(r * 255), .g = std.math.round(g * 255), .b = std.math.round(b * 255)};
// }

// var hue : f32 = 0.0;
// const saturation : f32 = 1.0;
// const lightness :f32 = 1.0;
export fn loop() void {
    const led_num = 400;

    // const color = hsl_to_rgb(hue, saturation, lightness);
    const color : light.Color = .{.r=87,.g=20,.b=0};
    for (0..led_num) |_| { // all leds
       light.show_color(color);
    }

    light.send_reset();
    // hue += 0.1;
    // hue %= 1;
}

// pub fn do_light_things() void {
//     const led_num = 400;
//     // const color : light.Color = .{.r=0,.g=200,.b=40};
//     const color : light.Color = .{.r=87,.g=20,.b=0};
//     for (0..led_num) |_| { // all leds
//        light.show_color(color);
//     }

//     light.send_reset();
// }
