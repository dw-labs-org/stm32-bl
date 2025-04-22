#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embassy_boot_stm32::*;
use embassy_stm32::flash::{Flash, BANK1_REGION, WRITE_SIZE};
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, peripherals, usb};
use embassy_sync::blocking_mutex::Mutex;
use embassy_usb::{msos, Builder};
use embassy_usb_dfu::consts::DfuAttributes;
use embassy_usb_dfu::{usb_dfu, Control, ResetImmediate};

bind_interrupts!(struct Irqs {
    OTG_FS => embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_FS>;

});
// This is a randomly generated GUID to allow clients on Windows to find our device
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{EAA9A5DC-30BA-44BC-9232-606CDC875321}"];

#[entry]
fn main() -> ! {
    #[cfg(feature = "defmt")]
    {
        defmt::info!("Bootloader started");
    }

    let mut config = embassy_stm32::Config::default();
    use embassy_stm32::rcc::*;
    config.rcc.hsi = true;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSI, // 16 MHz
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL10,
        divp: None,
        divq: None,
        divr: Some(PllDiv::DIV1), // 160 MHz
    });

    // config.rcc.ahb_pre = AHBPrescaler::DIV4;
    config.rcc.ls.lsi = true;
    config.rcc.mux.dac1sel = mux::Dacsel::LSI;
    config.rcc.sys = Sysclk::PLL1_R;
    config.rcc.voltage_range = VoltageScale::RANGE1;
    config.rcc.hsi48 = Some(Hsi48Config {
        sync_from_usb: true,
    }); // needed for USB
    config.rcc.mux.iclksel = mux::Iclksel::HSI48; // USB uses ICLK
    let p = embassy_stm32::init(config);

    // Prevent a hard fault when accessing flash 'too early' after boot.
    #[cfg(feature = "defmt")]
    for _ in 0..1000000 {
        cortex_m::asm::nop();
    }
    #[cfg(feature = "defmt")]
    {
        defmt::trace!("Trace");
    }

    let layout = Flash::new_blocking(p.FLASH).into_blocking_regions();
    let flash1 = Mutex::new(RefCell::new(layout.bank1_region));
    let flash2 = Mutex::new(RefCell::new(layout.bank2_region));

    let config = BootLoaderConfig::from_linkerfile_blocking(&flash1, &flash2, &flash1);
    let active_offset = config.active.offset();
    let bl = BootLoader::prepare::<_, _, _, 2048>(config);
    if bl.state == State::DfuDetach || true {
        let mut ep_out_buffer = [0u8; 256];
        let driver = Driver::new_fs(
            p.USB_OTG_FS,
            Irqs,
            p.PA12,
            p.PA11,
            &mut ep_out_buffer,
            Default::default(),
        );
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("USB-DFU Bootloader example");
        config.serial_number = Some("1235678");

        let fw_config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash2, &flash1);
        let mut buffer = AlignedBuffer([0; WRITE_SIZE]);
        let updater = BlockingFirmwareUpdater::new(fw_config, &mut buffer.0[..]);

        let mut config_descriptor = [0; 256];
        let mut bos_descriptor = [0; 256];
        let mut msos_descriptor = [0; 256];
        let mut control_buf = [0; 4096];
        let mut state = Control::new(updater, DfuAttributes::CAN_DOWNLOAD);
        let mut builder = Builder::new(
            driver,
            config,
            &mut config_descriptor,
            &mut bos_descriptor,
            &mut msos_descriptor,
            &mut control_buf,
        );

        // We add MSOS headers so that the device automatically gets assigned the WinUSB driver on Windows.
        // Otherwise users need to do this manually using a tool like Zadig.
        //
        // It seems it is important for the DFU class that these headers be on the Device level.
        //
        builder.msos_descriptor(msos::windows_version::WIN8_1, 2);
        builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
        builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
            "DeviceInterfaceGUIDs",
            msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
        ));

        usb_dfu::<_, _, _, ResetImmediate, 4096>(&mut builder, &mut state);

        let mut dev = builder.build();
        #[cfg(feature = "defmt")]
        {
            defmt::info!("Running USB");
        }
        embassy_futures::block_on(dev.run());
    }

    unsafe { bl.load(BANK1_REGION.base + active_offset) }
}

#[no_mangle]
#[cfg_attr(target_os = "none", link_section = ".HardFault.user")]
unsafe extern "C" fn HardFault() {
    #[cfg(feature = "defmt")]
    {
        defmt::error!("HardFault");
        defmt::flush();
    }
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
    let irqn = core::ptr::read_volatile(SCB_ICSR) as u8 as i16 - 16;
    #[cfg(feature = "defmt")]
    {
        defmt::error!("DefaultHandler #{:?}", irqn);
        defmt::flush();
    }
    panic!("DefaultHandler #{:?}", irqn);
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    #[cfg(feature = "defmt")]
    {
        defmt::error!("Panic: {}", _info);
        defmt::flush();
        loop {}
    }
    cortex_m::asm::udf();
}
