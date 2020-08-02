/// Handlers for events such as allocation failure, panic, and processor exceptions.

use cortex_m::{
    asm,
    interrupt
};

use cortex_m_rt::{
    exception,
    ExceptionFrame,
};

use itm_logger::{
    log,
    error,
    Level,
};

use core::panic::PanicInfo;

use freertos_rust::*;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    interrupt::disable();

    error!("{}", info);

    loop {
        // Get the debugger's attention.
        asm::bkpt();
    }
}

#[exception]
fn DefaultHandler(irqn: i16) {
    
    let type_name = match irqn {
        -1 => "Reset",
        -2 => "NMI",
        -3 => "HardFault",
        -4 => "MemManage",
        -5 => "BusFault",
        -6 => "UsageFault",
        -10..=-7 => "Reserved",
        -11 => "SVCall",
        -12 => "Debug Monitor",
        -13 => "Reserved",
        -14 => "PendSV",
        -15 => "SysTick",
        _ => if irqn < 0 {
            "External Interrupt"
        } else {
            "Custom Exception"
        }
    };

    panic!("Default Exception Handler: ({}) - {}", irqn, type_name);
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn MemoryManagement() -> ! {
    panic!("Memory management exception.");
}

#[exception]
fn BusFault() -> !{
    panic!("Bus fault.");
}

#[exception]
fn UsageFault() -> ! {
    panic!("Usage fault.");
}

// We need a function to handle our allocation errors.
#[alloc_error_handler]
fn allocation_error_handler(_: core::alloc::Layout) -> ! {
    // We just pass our error as a panic.
    panic!("Allocation failure.");
}

#[no_mangle]
extern "C" fn vApplicationIdleHook() {
    // Will put the processor to sleep until the next interrupt happens.
    asm::wfi();
}

#[no_mangle]
extern "C" fn vApplicationStackOverflowHook(_px_task: FreeRtosTaskHandle, pc_task_name: FreeRtosCharPtr) {
    panic!("Stack overflow in task: {:#?}", pc_task_name);
}
