use std::env;
use std::path::PathBuf;

fn main() {
    let mut b = freertos_cargo_build::Builder::new();
    let working = &PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());

    // Path to FreeRTOS kernel or set ENV "FREERTOS_SRC" instead
    b.freertos(format!("{}/FreeRTOS-Kernel", working.display()));
    b.freertos_config("frtos_cfg");       // Location of `FreeRTOSConfig.h` 
    b.freertos_port(String::from("GCC/ARM_CM4F")); // Port dir relative to 'FreeRTOS-Kernel/portable' 
    b.heap::<String>(String::from("heap_4.c"));              // Set the heap_?.c allocator to use from 
                                    // 'FreeRTOS-Kernel/portable/MemMang' (Default: heap_4.c)       

    // b.get_cc().file("More.c");   // Optional additional C-Code to be compiled

    b.compile().unwrap_or_else(|e| { panic!(e.to_string()) });

    // Re-run the build script when config is changed.
    println!("cargo:rerun-if-changed=frtos_cfg/FreeRTOSConfig.h");

    // Re-run the build script when memory.x is changed.
    println!("cargo:rerun-if-changed=memory.x");
}
