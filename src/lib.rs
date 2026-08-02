#![no_std]

#[cfg(feature = "definitions")]
extern crate std;

#[allow(unused_imports)]
#[allow(dead_code)]
mod mavlink {
    use core::{concat, env, include};

    include!(concat!(env!("OUT_DIR"), "/mavlink/mod.rs"));
}

use core::time::Duration;

pub use mavlink::dialects::rapid;
pub use mavlink::dialects::Rapid;

#[cfg(feature = "definitions")]
pub mod definitions {
    use std::sync::OnceLock;

    use mavinspect::protocol::Protocol;

    /// Reads protocol definitions which were generated and embedded at compile-time.
    pub fn protocol() -> &'static Protocol {
        static P: OnceLock<Protocol> = OnceLock::new();
        P.get_or_init(|| {
            const BYTES: &[u8] =
                include_bytes!(concat!(env!("OUT_DIR"), "/rapid-protocol.postcard.deflate"));
            let postcard = miniz_oxide::inflate::decompress_to_vec(BYTES)
                .expect("the embedded rapid protocol did not inflate");
            postcard::from_bytes(&postcard).expect("the embedded rapid protocol is corrupt")
        })
    }
}

// `From<rapid::messages::X> for Rapid` for every inherited message variant. mavspec only emits
// these for messages defined natively in the rapid standard, so build.rs fills in the rest.
include!(concat!(env!("OUT_DIR"), "/rapid_from_impls.rs"));

// mavspec's `Dialect` derive emits `MessageSpec` and `IntoPayload` for the dialect enum, but not
// the empty `Message` blanket. Add it so callers can pass `&Rapid` directly to anything taking
// `&dyn Message` (e.g. `mavio::Endpoint::next_frame`) without dispatching on every variant.
impl mavspec::rust::spec::Message for Rapid {}

use mavlink::dialects::minimal::enums::MavState;

// TODO: Do we need/want our flight mode type to live here or do we want to move it to the
// firmware?

// Variants are ordered roughly by mission timeline so the value also conveys progression.
#[derive(Default, Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Hash)]
pub enum FlightMode {
    /// Rocket is idle, outputs are physically disconnected
    #[default]
    Idle = 0,
    /// [hybrid] Pressurant is transferred from the external tank into the rocket pressurant tank
    FillPressurant = 1,
    /// [hybrid] Oxidizer is transferred from the external tank into the rocket, vents may be pulsed
    FillOxidizer = 2,
    /// [hybrid] Vents both pressurant and oxidizer
    Vent = 3,
    /// [hybrid] Pressurization valve opens, ignition is expected to follow soon
    Pressurize = 4,
    /// [hybrid] Hold all valve states when entered, allows manual operation
    Hold = 5,
    /// [solid] Rocket is awaiting external ignition, detects launch via acceleration
    DetectLaunch = 6,
    /// [hybrid] Runs ignition sequence, but switch to Burn still happens via launch accel. detection
    Ignite = 7,
    /// Motor is active, thrust exceeds drag
    Burn = 8,
    /// Coasting to apogee
    Coast = 9,
    /// Entered on apogee, triggers drogue deployment
    DeployDrogue = 10,
    /// Entered below threshold altitude (above ground), triggers main parachute
    DeployMain = 11,
    /// Entered on touchdown
    Landed = 12,
}

impl TryFrom<u8> for FlightMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Idle),
            1 => Ok(Self::FillPressurant),
            2 => Ok(Self::FillOxidizer),
            3 => Ok(Self::Vent),
            4 => Ok(Self::Pressurize),
            5 => Ok(Self::Hold),
            6 => Ok(Self::DetectLaunch),
            7 => Ok(Self::Ignite),
            8 => Ok(Self::Burn),
            9 => Ok(Self::Coast),
            10 => Ok(Self::DeployDrogue),
            11 => Ok(Self::DeployMain),
            12 => Ok(Self::Landed),
            _ => Err(()),
        }
    }
}

// We derive MAV_STATE from the flight mode. We don't necessarily have to do this, this could be
// orthogonal to our flight mode.
impl Into<MavState> for FlightMode {
    fn into(self) -> MavState {
        match self {
            Self::Idle => MavState::Standby,
            Self::FillPressurant
            | Self::FillOxidizer
            | Self::Pressurize
            | Self::Hold
            | Self::Vent
            | Self::DetectLaunch
            | Self::Ignite
            | Self::Burn
            | Self::Coast => MavState::Active,
            Self::DeployDrogue | Self::DeployMain | Self::Landed => MavState::FlightTermination,
        }
    }
}

impl FlightMode {
    pub const ALL: [FlightMode; 13] = [
        Self::Idle,
        Self::FillPressurant,
        Self::FillOxidizer,
        Self::Vent,
        Self::Pressurize,
        Self::Hold,
        Self::DetectLaunch,
        Self::Ignite,
        Self::Burn,
        Self::Coast,
        Self::DeployDrogue,
        Self::DeployMain,
        Self::Landed,
    ];

    /// Name as a 35-length char array, as included in AVAILABLE_MODES MAVLink messages. Must
    /// include null termination character.
    pub fn mavlink_name(self) -> [u8; 35] {
        // no format in no_std, vim macro goes brrr
        let string = match self {
            Self::Idle => "Idle",
            Self::FillPressurant => "FillPressurant",
            Self::FillOxidizer => "FillOxidizer",
            Self::Vent => "Vent",
            Self::Pressurize => "Pressurize",
            Self::Hold => "Hold",
            Self::DetectLaunch => "DetectLaunch",
            Self::Ignite => "Ignite",
            Self::Burn => "Burn",
            Self::Coast => "Coast",
            Self::DeployDrogue => "DeployDrogue",
            Self::DeployMain => "DeployMain",
            Self::Landed => "Landed",
        };

        let mut buf = [0; 35];
        for (b, c) in buf.iter_mut().zip(string.chars()) {
            *b = c as u8;
        }
        buf
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum ValveCommand {
    Open,
    Partial(f32),
    PulseOpen(Duration),
    Close,
}
