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
    domain::temperature::Celsius,
    traits::sensors::temperature::TemperatureSensor,
    traits::wifi::*,
    *,
};
use drogue_device::{
    drivers::dns::{DnsEntry, StaticDnsResolver},
    drogue,
};
use embassy::time::{with_timeout, Duration, Ticker};
use embassy::util::Forever;
use embassy::util::{select, Either};
use embassy_stm32::{flash::Flash, Peripherals};
use embedded_io::{Error, ErrorKind};
use embedded_nal_async::{AddrType, Dns, IpAddr, Ipv4Addr, SocketAddr, TcpConnect};
use embedded_tls::*;
use embedded_update::{service::DrogueHttp, DeviceStatus};
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

const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");
const FIRMWARE_REVISION: Option<&str> = option_env!("REVISION");

#[embassy::main(config = "Iot01a::config(true)")]
async fn main(spawner: embassy::executor::Spawner, p: Peripherals) {
    let board = Iot01a::new(p);
    unsafe {
        RNG_INST.replace(board.rng);
    }

    static NETWORK: Forever<SharedEsWifi> = Forever::new();
    let network: &'static SharedEsWifi = NETWORK.put(SharedEsWifi::new(board.wifi));

    spawner
        .spawn(network_task(
            network,
            WIFI_SSID.trim_end(),
            WIFI_PSK.trim_end(),
        ))
        .unwrap();

    let mut app = App::new(
        HOST,
        PORT.parse::<u16>().unwrap(),
        USERNAME.trim_end(),
        PASSWORD.trim_end(),
        network,
        TlsRand,
    );
    let mut sensor = Hts221::new(board.i2c2);
    sensor.initialize().await.ok().unwrap();

    let mut button = board.user_button;
    let interval = Duration::from_secs(30);
    let mut ticker = Ticker::every(interval);
    defmt::info!(
        "Application running. Sensor data is sent every {} seconds or when button is pressed",
        interval.as_secs()
    );

    spawner.spawn(updater_task(network, board.flash)).unwrap();

    loop {
        let _ = select(button.wait_released(), ticker.next()).await;
        match sensor.temperature().await {
            Ok(data) => {
                let payload = TemperatureData {
                    geoloc: None, // TODO: Set to your latlong coordinates
                    temp: data.temperature.raw_value(),
                    hum: data.relative_humidity,
                };
                match with_timeout(Duration::from_secs(20), app.send(payload)).await {
                    Ok(r) => match r {
                        Ok(_) => {
                            defmt::info!("Measurement sent");
                        }
                        Err(_) => {
                            defmt::error!("Error sending measurement");
                        }
                    },
                    Err(_) => {
                        defmt::warn!("Timed out sending sensor data");
                    }
                }
            }
            Err(_) => {
                defmt::error!("Error reading sensor");
            }
        }
    }
}

#[embassy::task]
async fn network_task(adapter: &'static SharedEsWifi, ssid: &'static str, psk: &'static str) {
    loop {
        let _ = adapter.run(ssid, psk).await;
    }
}

static mut RNG_INST: Option<Rng> = None;

#[no_mangle]
fn _embassy_rand(buf: &mut [u8]) {
    use rand_core::RngCore;

    critical_section::with(|_| unsafe {
        defmt::unwrap!(RNG_INST.as_mut()).fill_bytes(buf);
    });
}

pub struct TlsRand;

impl rand_core::RngCore for TlsRand {
    fn next_u32(&mut self) -> u32 {
        critical_section::with(|_| unsafe { defmt::unwrap!(RNG_INST.as_mut()).next_u32() })
    }
    fn next_u64(&mut self) -> u64 {
        critical_section::with(|_| unsafe { defmt::unwrap!(RNG_INST.as_mut()).next_u64() })
    }
    fn fill_bytes(&mut self, buf: &mut [u8]) {
        critical_section::with(|_| unsafe {
            defmt::unwrap!(RNG_INST.as_mut()).fill_bytes(buf);
        });
    }
    fn try_fill_bytes(&mut self, buf: &mut [u8]) -> Result<(), rand_core::Error> {
        critical_section::with(|_| unsafe {
            defmt::unwrap!(RNG_INST.as_mut()).fill_bytes(buf);
        });
        Ok(())
    }
}
impl rand_core::CryptoRng for TlsRand {}

#[embassy::task]
async fn updater_task(network: &'static SharedEsWifi, flash: Flash<'static>) {
    use drogue_device::firmware::BlockingFlash;
    use embassy::time::{Delay, Timer};

    let version = FIRMWARE_REVISION.unwrap_or(FIRMWARE_VERSION);
    defmt::info!("Running firmware version {}", version);
    let updater = embassy_boot_stm32::FirmwareUpdater::default();

    let ip = DNS
        .get_host_by_name(HOST.trim_end(), AddrType::IPv4)
        .await
        .unwrap();

    let service: DrogueHttp<'_, _, _, 2048> = DrogueHttp::new(
        network,
        TlsRand,
        SocketAddr::new(ip, PORT.parse::<u16>().unwrap()),
        HOST.trim_end(),
        USERNAME.trim_end(),
        PASSWORD.trim_end(),
    );

    let mut device: FirmwareManager<BlockingFlash<Flash<'static>>, 4096, 2048> =
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

const HOST: &str = drogue::config!("hostname");
const PORT: &str = drogue::config!("port");
const USERNAME: &str = drogue::config!("http-username");
const PASSWORD: &str = drogue::config!("http-password");

static DNS: StaticDnsResolver<'static, 2> = StaticDnsResolver::new(&[
    DnsEntry::new("localhost", IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1))),
    DnsEntry::new(
        "http.sandbox.drogue.cloud",
        IpAddr::V4(Ipv4Addr::new(65, 108, 135, 161)),
    ),
]);

pub struct App<'a, T, RNG>
where
    T: TcpConnect,
    RNG: CryptoRng + RngCore,
{
    host: &'a str,
    username: &'a str,
    password: &'a str,
    port: u16,
    client: T,
    rng: RNG,
    tls: [u8; 16384],
}

impl<'a, T, RNG> App<'a, T, RNG>
where
    T: TcpConnect,
    RNG: CryptoRng + RngCore,
{
    fn new(
        host: &'a str,
        port: u16,
        username: &'a str,
        password: &'a str,
        client: T,
        rng: RNG,
    ) -> Self {
        Self {
            host,
            port,
            username,
            password,
            client,
            rng,
            tls: [0; 16384],
        }
    }
    async fn send(&mut self, data: TemperatureData) -> Result<(), ErrorKind> {
        defmt::debug!("Resolving {}:{}", self.host, self.port);
        let ip = DNS
            .get_host_by_name(self.host, AddrType::IPv4)
            .await
            .map_err(|_| ErrorKind::Other)?;

        defmt::debug!("Connecting to {:?}:{}", defmt::Debug2Format(&ip), self.port);
        let connection = self
            .client
            .connect(SocketAddr::new(ip, self.port))
            .await
            .map_err(|e| e.kind())?;

        let mut connection: TlsConnection<'_, _, Aes128GcmSha256> =
            TlsConnection::new(connection, &mut self.tls);

        defmt::debug!("Establishing TLS connection to {}", self.host);
        connection
            .open::<_, NoClock, 1>(TlsContext::new(
                &TlsConfig::new().with_server_name(self.host),
                &mut self.rng,
            ))
            .await
            .map_err(|_| ErrorKind::Other)?;

        defmt::info!("Connected to {}:{}", self.host, self.port);

        let mut client = client::HttpClient::new(&mut connection, self.host);

        let tx: heapless::String<128> =
            serde_json_core::ser::to_string(&data).map_err(|_| ErrorKind::Other)?;
        let mut rx_buf = [0; 1024];
        let response = client
            .request(
                request::Request::post()
                    // Pass on schema
                    .path("/v1/foo?data_schema=urn:drogue:iot:temperature")
                    .basic_auth(self.username, self.password)
                    .payload(tx.as_bytes())
                    .content_type(request::ContentType::ApplicationJson)
                    .build(),
                &mut rx_buf[..],
            )
            .await;

        match response {
            Ok(response) => {
                defmt::info!("Response status: {:?}", response.status);
                if response.status != request::Status::Accepted {
                    defmt::warn!("Response error: {:?}", response.status);
                    Err(ErrorKind::Other)
                } else {
                    Ok(())
                }
            }
            Err(e) => {
                defmt::warn!("Error performing HTTP request: {:?}", e);
                Err(ErrorKind::Other)
            }
        }
    }
}

// Data types
#[derive(Clone, Serialize, Deserialize, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GeoLocation {
    pub lon: f32,
    pub lat: f32,
}

#[derive(Clone, Serialize, Deserialize, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TemperatureData {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub geoloc: Option<GeoLocation>,
    pub temp: f32,
    pub hum: f32,
}
