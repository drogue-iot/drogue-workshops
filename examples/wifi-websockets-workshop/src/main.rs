#![no_std]
#![no_main]
#![macro_use]
#![allow(incomplete_features)]
#![allow(dead_code)]
#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]
#![feature(concat_idents)]

use drogue_device::{
    bsp::{boards::stm32l4::iot01a::*, Board},
    traits::sensors::temperature::TemperatureSensor,
    *,
};
use drogue_device::{
    drivers::dns::{DnsEntry, StaticDnsResolver},
    drivers::sensors::hts221::Hts221,
};
use embassy::util::Forever;
use embassy::{
    time::{with_timeout, Duration, Ticker},
    util::select,
};
use embassy_stm32::rcc::*;
use embassy_stm32::Peripherals;
use embedded_io::{Error, ErrorKind};
use embedded_nal_async::{AddrType, Dns, IpAddr, Ipv4Addr, SocketAddr, TcpConnect};
use embedded_tls::*;
use futures::StreamExt;
use rand_core::{CryptoRng, RngCore};
use reqwless::*;
use serde::{Deserialize, Serialize};

use defmt_rtt as _;
use panic_probe as _;

const WIFI_SSID: &str = drogue::config!("wifi-ssid");
const WIFI_PSK: &str = drogue::config!("wifi-password");
const HOST: &str = drogue::config!("hostname");
const PORT: &str = drogue::config!("port");
const USERNAME: &str = drogue::config!("http-username");
const PASSWORD: &str = drogue::config!("http-password");

pub fn config() -> embassy_stm32::Config {
    let enable_debug = true;
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = ClockSrc::PLL(
        // 16 Mhz (vs 32 Mhz)
        PLLSource::HSI16,
        PLLClkDiv::Div2,
        PLLSrcDiv::Div2,
        PLLMul::Mul12,
        Some(PLLClkDiv::Div2),
    );
    config.rcc.ahb_pre = AHBPrescaler::Div8;
    config.enable_debug_during_sleep = enable_debug;
    config
}

#[embassy::main(config = "config()")]
async fn main(spawner: embassy::executor::Spawner, p: Peripherals) {
    let board = Iot01a::new(p);

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
        board.rng,
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

static DNS: StaticDnsResolver<'static, 3> = StaticDnsResolver::new(&[
    DnsEntry::new("localhost", IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1))),
    DnsEntry::new(
        "http.sandbox.drogue.cloud",
        IpAddr::V4(Ipv4Addr::new(65, 108, 135, 161)),
    ),
    DnsEntry::new(
        "http-endpoint-drogue-dev.apps.wonderful.iot-playground.org",
        IpAddr::V4(Ipv4Addr::new(65, 108, 135, 161)),
    ),
]);
