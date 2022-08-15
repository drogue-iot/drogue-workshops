#![no_std]
#![no_main]
#![macro_use]
#![allow(incomplete_features)]
#![allow(unused_imports)]
#![allow(dead_code)]
#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]
#![feature(concat_idents)]

use drogue_device::drivers::sensors::hts221::Hts221;
use drogue_device::firmware::FirmwareManager;
use drogue_device::{
    bsp::{boards::stm32l4::iot01a::*, Board},
    traits::wifi::*,
    *,
};
use drogue_device::{
    drivers::dns::{DnsEntry, StaticDnsResolver},
    drogue,
    firmware::BlockingFlash,
};
use embassy::time::Timer;
use embassy::time::{with_timeout, Duration, Ticker};
use embassy::util::Forever;
use embassy::util::{select, Either};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_stm32::{flash::Flash, Peripherals};
use embedded_io::{Error, ErrorKind};
use embedded_nal_async::{AddrType, Dns, IpAddr, Ipv4Addr, SocketAddr, TcpConnect};
use embedded_tls::*;
use embedded_update::{
    service::{DrogueHttp, InMemory},
    DeviceStatus,
};
use futures::StreamExt;
use rand_core::{CryptoRng, RngCore};
use reqwless::*;
use serde::{Deserialize, Serialize};

#[cfg(feature = "panic-probe")]
use panic_probe as _;

use defmt_rtt as _;

#[cfg(feature = "panic-reset")]
use panic_reset as _;

const WIFI_SSID: &str = drogue::config!("wifi-ssid");
const WIFI_PSK: &str = drogue::config!("wifi-password");

const HOST: &str = drogue::config!("hostname");
const PORT: &str = drogue::config!("port");
const USERNAME: &str = drogue::config!("http-username");
const PASSWORD: &str = drogue::config!("http-password");

const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");
const FIRMWARE_REVISION: Option<&str> = option_env!("REVISION");

#[embassy::main(config = "Iot01a::config(true)")]
async fn main(spawner: embassy::executor::Spawner, p: Peripherals) {
    let mut board = Iot01a::new(p);
    static NETWORK: Forever<SharedEsWifi> = Forever::new();
    let network: &'static SharedEsWifi = NETWORK.put(SharedEsWifi::new(board.wifi));

    spawner
        .spawn(network_task(
            network,
            WIFI_SSID.trim_end(),
            WIFI_PSK.trim_end(),
        ))
        .unwrap();

    defmt::info!("Application running");

    let _ = board.led_blue.off();
    let _ = board.led_green.off();

    spawner
        .spawn(updater_task(network, board.flash, board.rng))
        .unwrap();

    #[cfg(feature = "blue")]
    {
        let mut led = board.led_blue;
        // Loop blinking our LED
        loop {
            let _ = led.on();
            Timer::after(Duration::from_millis(1000)).await;
            let _ = led.off();
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}

#[embassy::task]
async fn network_task(adapter: &'static SharedEsWifi, ssid: &'static str, psk: &'static str) {
    loop {
        let _ = adapter.run(ssid, psk).await;
    }
}

#[embassy::task]
async fn updater_task(network: &'static SharedEsWifi, flash: Flash<'static>, rng: Rng) {
    use drogue_device::firmware::BlockingFlash;
    use embassy::time::{Delay, Timer};
    let ip = DNS
        .get_host_by_name(HOST.trim_end(), AddrType::IPv4)
        .await
        .unwrap();

    let version = FIRMWARE_REVISION.unwrap_or(FIRMWARE_VERSION);
    defmt::info!("Running firmware version {}", version);
    let updater = embassy_boot_stm32::FirmwareUpdater::default();

    let service: DrogueHttp<'_, _, _, 5120> = DrogueHttp::new(
        network,
        rng,
        SocketAddr::new(ip, PORT.parse::<u16>().unwrap()),
        HOST.trim_end(),
        USERNAME.trim_end(),
        PASSWORD.trim_end(),
    );

    let mut device: FirmwareManager<BlockingFlash<Flash<'static>>, 2048, 4096> =
        FirmwareManager::new(BlockingFlash::new(flash), updater, version.as_bytes());
    let mut updater = embedded_update::FirmwareUpdater::new(
        service,
        embedded_update::UpdaterConfig {
            timeout_ms: 40_000,
            backoff_ms: 100,
        },
    );

    loop {
        defmt::info!("Starting updater task");
        match updater.run(&mut device, &mut Delay).await {
            Ok(s) => {
                defmt::info!("Updater finished with status: {:?}", s);
                match s {
                    DeviceStatus::Updated => {
                        defmt::debug!("Resetting device");
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                    DeviceStatus::Synced(delay) => {
                        if let Some(delay) = delay {
                            Timer::after(Duration::from_secs(delay as u64)).await;
                        } else {
                            Timer::after(Duration::from_secs(10)).await;
                        }
                    }
                }
            }
            Err(e) => {
                defmt::warn!("Error running updater: {:?}", defmt::Debug2Format(&e));
                Timer::after(Duration::from_secs(10)).await;
            }
        }
    }
}

static DNS: StaticDnsResolver<'static, 2> = StaticDnsResolver::new(&[
    DnsEntry::new("localhost", IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1))),
    DnsEntry::new(
        "http.sandbox.drogue.cloud",
        IpAddr::V4(Ipv4Addr::new(65, 108, 135, 161)),
    ),
]);
