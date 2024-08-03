const std = @import("std");
// Oh boy. This needs to compile a zig file into a object file, that can
// run on a Renesas ra4m1 processor. Yikes.
// Here is the command that Arduino uses to compile a .ino.cpp file into a .ino.cpp.o file.

// /home/david/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-g++
// -c -> Do not run linker.
// -w  -> Inhibit all warnings
// -Os -> Optimize for size. All -O2 optimizations except for ones that increase size.
// -g3 -> Include debug symbols, level 3.
// -fno-use-cxa-atexit -> don't call __cxa_atexit when the program ends (it destroys global resources and dll things)
// -fno-rtti -> Don't generate dynamic information about types for the C++ runtime (dynamic_cast doesn't work then)
// -fno-exceptions -> Exceptions don't happen
// -MMD -> Instead of outputting preprocessing, make a Make rule file that describe dependancies. (not system files.)
// -nostdlib -> no hidden includes; no standard library (for C)
// -DF_CPU=48000000 -> CPU clock speed
// -DNO_USB -> define no usb?
// -DBACKTRACE_SUPPORT
// -DARDUINO_UNOR4_WIFI
// -MMD
// -std=gnu++17 -> Use GNU C++17
// -mcpu=cortex-m4 -> Specifies the type of cpu
// -mfloat-abi=hard -> Allows FPU-specific floating-point calling convensions.
// -mfpu=fpv4-sp-d16 -> specifies the FPU hardware is fpv4-sp-d16
// -fsigned-char -> char is signed.
// -ffunction-sections -> all functions in its own data sections if the target supports.
// -fdata-sections  -> all data items have their own sections. Can make the binary smaller.
// -fmessage-length=0 -> no line-wrapping on errors.
// -fno-builtin -> Do not optimize the shit out of some funcitons, unless it starts with __builtin_
// -DARDUINO=10607
// "-DPROJECT_NAME=\"/tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/SoftwareSerialExample.ino\""
// -DARDUINO_UNOWIFIR4
// -DARDUINO_ARCH_RENESAS_UNO
// -DARDUINO_ARCH_RENESAS
// -DARDUINO_FSP
// -D_XOPEN_SOURCE=700
// -mthumb -> use Thumb (16 bit mode)
// @/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/defines.txt -> read more options
// |
// v
// -D_RA_CORE=CM4
// -D_RENESAS_RA_

// -DCFG_TUSB_MCU=OPT_MCU_RAXXX
//                                  A bunch of includes
// -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino/tinyusb
// -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino/api/deprecated
// -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino/api/deprecated-avr-comp
// -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino
// -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4
// -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/libraries/SoftwareSerial/src
// -iprefix/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0 -> used to set up the prefix of includes.txt v
// @/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes.txt
// |
// v
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/fsp/inc
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/fsp/inc/api
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/fsp/inc/instances
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/arm/CMSIS_5/CMSIS/Core/Include
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra_gen
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra_cfg/fsp_cfg/bsp
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra_cfg/fsp_cfg
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/fsp/src/r_usb_basic/src/driver/inc
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/private/inc
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/public/inc
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/primitive
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/common
// -iwithprefixbefore/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce

// /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/sketch/SoftwareSerialExample.ino.cpp -> input file
// -o /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/sketch/SoftwareSerialExample.ino.cpp.o -> output file

// Paths that are referenced a lot.
const renesas: std.Build.LazyPath = .{ .cwd_relative = "/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/" };
const libc: std.Build.LazyPath = .{ .cwd_relative = "/home/david/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/" };

pub fn toolchain(b: *std.Build, step: *std.Build.Step.Compile) !void {

    // Include these directories into this object file (exe)
    // -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino/tinyusb
    // -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino/api/deprecated
    // -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino/api/deprecated-avr-comp
    // -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/cores/arduino
    // -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4
    // -I/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/libraries/SoftwareSerial/src
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/fsp/inc
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/fsp/inc/api
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/fsp/inc/instances
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/arm/CMSIS_5/CMSIS/Core/Include
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra_gen
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra_cfg/fsp_cfg/bsp
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra_cfg/fsp_cfg
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/fsp/src/r_usb_basic/src/driver/inc
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/private/inc
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/public/inc
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/primitive
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/common
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/includes/ra/fsp/src/r_sce
    step.addIncludePath(renesas.path(b, "cores/arduino/tinyusb"));
    step.addIncludePath(renesas.path(b, "cores/arduino/api/deprecated"));
    step.addIncludePath(renesas.path(b, "cores/arduino/api/deprecated-avr-comp"));
    step.addIncludePath(renesas.path(b, "cores/arduino"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4"));
    step.addIncludePath(renesas.path(b, "libraries/SoftwareSerial/src"));

    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/fsp/inc"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/fsp/inc/api"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/fsp/inc/instances"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/arm/CMSIS_5/CMSIS/Core/Include"));
    step.addSystemIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra_gen"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra_cfg/fsp_cfg/bsp"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra_cfg/fsp_cfg"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/fsp/src/r_usb_basic/src/driver/inc"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/private/inc"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/public/inc"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/crypto_procedures/src/sce5/plainkey/primitive"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/fsp/src/r_sce/common"));
    step.addIncludePath(renesas.path(b, "variants/UNOWIFIR4/includes/ra/fsp/src/r_sce"));

    step.addIncludePath(b.path("src/c-binding/"));
    // -DF_CPU=48000000 -> CPU clock speed
    // -DNO_USB -> define no usb?
    // -DBACKTRACE_SUPPORT
    // -DARDUINO_UNOR4_WIFI
    // -DARDUINO=10607
    // "-DPROJECT_NAME=\"/tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/SoftwareSerialExample.ino\""
    // -DARDUINO_UNOWIFIR4
    // -DARDUINO_ARCH_RENESAS_UNO
    // -DARDUINO_ARCH_RENESAS
    // -DARDUINO_FSP
    // -D_XOPEN_SOURCE=700
    // -D_RA_CORE=CM4
    // -D_RENESAS_RA_
    // -DCFG_TUSB_MCU=OPT_MCU_RAXXX
    try step.root_module.c_macros.appendSlice(b.allocator, &.{
        "-DF_CPU=48000000",
        "-DNO_USB",
        "-DBACKTRACE_SUPPORT",
        "-DARDUINO_UNOR4_WIFI",
        "-DARDUINO=10607",
        "-DPROJECT_NAME=/home/david/projects/pi-zig-light-wizard/wut",
        "-DARDUINO_UNOWIFIR4",
        "-DARDUINO_ARCH_RENESAS_UNO",
        "-DARDUINO_ARCH_RENESAS",
        "-DARDUINO_FSP",
        "-D_XOPEN_SOURCE=700",
        "-D_RA_CORE=CM4",
        "-D_RENESAS_RA_",
        "-DCFG_TUSB_MCU=OPT_MCU_RAXXX",
    });

    // libc needs to be included as well
    step.addSystemIncludePath(libc.path(b, "arm-none-eabi/include/"));
    step.addSystemIncludePath(libc.path(b, "lib/gcc/arm-none-eabi/7.2.1/include/"));
}

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{
        .default_target = .{
            .cpu_arch = .thumb, // 16 bit arm
            .os_tag = .freestanding,
            .abi = .eabihf,
            .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
            .cpu_features_add = std.Target.arm.featureSet(&.{.vfp4d16sp}),
            .cpu_features_sub = std.Target.arm.featureSet(&.{ // Don't have these features
                .soft_float, // so, use hard float
            }),
        },
    });

    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .Debug });

    const zig_obj = b.addObject(.{ .name = "zig-code", .target = target, .optimize = optimize, .root_source_file = b.path("src/main.zig") });
    try toolchain(b, zig_obj);

    const arduino_binding_obj = b.addObject(.{ .name = "arduino", .target = target, .optimize = optimize });
    // -fshort-enums wasn't from Arduino's toolchain. It shuts up a `ld` warning about variable-sized enums.
    arduino_binding_obj.addCSourceFile(.{ .file = b.path("src/c-binding/arduino.cpp"), .flags = &.{ "-w", "-fno-use-cxa-atexit", "-fno-rtti", "-fno-exceptions", "-std=gnu++17", "-fshort-enums" } });
    try toolchain(b, arduino_binding_obj);
    arduino_binding_obj.addIncludePath(libc.path(b, "arm-none-eabi/include/c++/7.2.1/"));
    arduino_binding_obj.addSystemIncludePath(libc.path(b, "arm-none-eabi/include/c++/7.2.1/arm-none-eabi/thumb/"));

    const fastled_path = b.path("deps/FastLED-3.7.0/src/");
    arduino_binding_obj.addIncludePath(fastled_path);
    // const fastled_obj = b.addObject(.{ .name = "fastled", .target = target, .optimize = optimize });
    // // -fshort-enums wasn't from Arduino's toolchain. It shuts up a `ld` warning about variable-sized enums.
    // fastled_obj.addCSourceFiles(.{
    //     .root = fastled_path,
    //     .files =&.{
    //         // "platforms/esp/32/clockless_rmt_esp32.cpp",
    //         "noise.cpp",
    //         "colorutils.cpp",
    //         "lib8tion.cpp",
    //         "wiring.cpp",
    //         "power_mgt.cpp",
    //         "platforms.cpp",
    //         "bitswap.cpp",
    //         "hsv2rgb.cpp",
    //         "FastLED.cpp",
    //         "five_bit_hd_gamma.cpp",
    //         "colorpalettes.cpp"
    //     },
    //     .flags = &.{ "-w", "-fno-use-cxa-atexit", "-fno-rtti", "-fno-exceptions", "-std=gnu++17", "-fshort-enums" }
    // });
    // try toolchain(b, fastled_obj);
    // fastled_obj.addIncludePath(fastled_path);
    // fastled_obj.addIncludePath(libc.path(b, "arm-none-eabi/include/c++/7.2.1/"));
    // fastled_obj.addSystemIncludePath(libc.path(b, "arm-none-eabi/include/c++/7.2.1/arm-none-eabi/thumb/"));

    // Linking
    // This is the command that the Arduino IDE uses to link things together. vvv

    // /home/david/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-g++
    // -Wl,--gc-sections
    // --specs=nosys.specs
    // -w
    // -mcpu=cortex-m4
    // -mfloat-abi=hard
    // -mfpu=fpv4-sp-d16
    // -o /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/SoftwareSerialExample.ino.elf
    // -L/tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75
    // -L/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4
    // -T/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/fsp.ld

    // Objects that need to be linked vvvvvvvvvvv
    // /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/sketch/SoftwareSerialExample.ino.cpp.o
    // /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/libraries/SoftwareSerial/SoftwareSerial.cpp.o
    // /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/core/tmp_gen_c_files/common_data.c.o
    // /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/core/tmp_gen_c_files/main.c.o
    // /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/core/tmp_gen_c_files/pin_data.c.o
    // /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/core/variant.cpp.o

    // -Wl,--whole-archive
    // -Wl,--start-group

    // Libraries to link to vvvvvvvvvvvvvv
    // /home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/libs/libfsp.a
    // /tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/core/core.a

    // -Wl,--no-whole-archive
    // --specs=nano.specs
    // -lstdc++
    // -lsupc++
    // -lm
    // -lc
    // -lgcc
    // -lnosys -Wl,--end-group -Wl,-Map,/tmp/arduino/sketches/DFC96378F6DD7B8AA75E80BD294AEC75/SoftwareSerialExample.ino.map
    //
    // ==============================


    // Link everything!
    const link_step = b.addSystemCommand(&.{"/home/david/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-g++"});
    link_step.addArgs(&.{
        "-Wl,--gc-sections",
        "--specs=nosys.specs",
        "-w",
        "-mcpu=cortex-m4",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
        "-T./link.ld"
    });
    link_step.addArtifactArg(zig_obj);
    link_step.addArtifactArg(arduino_binding_obj);
    link_step.addArgs(&.{
        "-Wl,--whole-archive",
        "-Wl,--start-group",
        "/home/david/.arduino15/packages/arduino/hardware/renesas_uno/1.2.0/variants/UNOWIFIR4/libs/libfsp.a",
        b.path("core.a").getPath(b),
        b.path("fastled.a").getPath(b),
        "-Wl,--no-whole-archive",
        "--specs=nano.specs",
        "-lstdc++",
        "-lsupc++",
        "-lm",
        "-lc",
        "-lgcc",
        "-lnosys",
        "-Wl,--end-group",
        "-o"
    });
    const wizard_exe_name = "pi-light-wizard.elf";
    const output_elf_path = link_step.addOutputFileArg(wizard_exe_name);

    // Turn the ELF into a bin.
    const bin_step = b.addSystemCommand(&.{"/home/david/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-objcopy"});
    bin_step.addArgs(&.{
        "-O",
        "binary",
        "-j",
        ".text",
        "-j",
        ".data",
    });
    bin_step.addFileArg(output_elf_path);
    const wizard_bin_name = "pi-light-wizard.bin";
    const output_bin_path = bin_step.addOutputFileArg(wizard_bin_name);
    bin_step.step.dependOn(&link_step.step);

    // Save the file. Not really neccessary, but do it.
    const install_file_step = b.addInstallFile(output_bin_path, "./" ++ wizard_bin_name);
    install_file_step.step.dependOn(&bin_step.step);

    b.getInstallStep().dependOn(&install_file_step.step);

    // Program the Arduino!
    const program_step = b.addSystemCommand(&.{"arduino-cli"});
    program_step.addArgs(&.{
        "upload",
        "--fqbn",
        "arduino:renesas_uno:unor4wifi",
        "-p",
        "/dev/ttyACM0",
        "--input-file"
    });
    program_step.addFileArg(output_bin_path);

    // Terminal node in the graph! Allows the command 'zig build program'.
    const program_run_step = b.step("program", "Program the Arduino Uno R4 WIFI");
    program_run_step.dependOn(&program_step.step);
}
