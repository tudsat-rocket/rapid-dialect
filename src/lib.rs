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
    use mavinspect::Inspector;

    // Counterpart to `mavspec::definitions::protocol()` for the rapid dialect
    // tree, which mavspec's bundled definitions don't cover. Used by tooling
    // that needs to introspect rapid messages (e.g. nadir's DB layer).
    pub fn protocol() -> &'static Protocol {
        static P: OnceLock<Protocol> = OnceLock::new();
        P.get_or_init(|| {
            const STANDARD: &str =
                concat!(env!("CARGO_MANIFEST_DIR"), "/message_definitions/standard",);
            const EXTRA: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/message_definitions/extra",);
            Inspector::builder()
                .set_sources(&[STANDARD, EXTRA])
                .build()
                .unwrap()
                .parse()
                .unwrap()
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
    /// Rocket is idle software-side, but arming pins are pulled/switches are thrown
    HardwareArmed = 1,
    /// [hybrid] Oxidizer is being filled, ox vent valve may be opened
    Filling = 2,
    /// [hybrid] Vents both pressurant and oxidizer
    Venting = 3,
    /// [hybrid] Pressurization valve opens, ignition is expected to follow soon
    Pressurizing = 4,
    /// [hybrid] Hold all valve states when entered, allows manual operation
    Hold = 5,
    /// [solid] Rocket is awaiting external ignition, detects launch via acceleration
    Armed = 6,
    /// [hybrid] Runs ignition sequence, but switch to Burn still happens via launch accel. detection
    Ignition = 7,
    /// Motor is active, thrust exceeds drag
    Burn = 8,
    /// Coasting to apogee
    Coast = 9,
    /// Entered on apogee, triggers drogue deployment
    RecoveryDrogue = 10,
    /// Entered below threshold altitude (above ground), triggers main parachute
    RecoveryMain = 11,
    /// Entered on touchdown
    Landed = 12,
}

impl TryFrom<u8> for FlightMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Idle),
            1 => Ok(Self::HardwareArmed),
            2 => Ok(Self::Filling),
            3 => Ok(Self::Venting),
            4 => Ok(Self::Pressurizing),
            5 => Ok(Self::Hold),
            6 => Ok(Self::Armed),
            7 => Ok(Self::Ignition),
            8 => Ok(Self::Burn),
            9 => Ok(Self::Coast),
            10 => Ok(Self::RecoveryDrogue),
            11 => Ok(Self::RecoveryMain),
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
            // From MAVLink docs: System is calibrating and not flight-ready.
            Self::Idle => MavState::Calibrating,
            // From MAVLink docs: System is grounded and on standby. It can be launched any time.
            Self::HardwareArmed => MavState::Standby,
            // From MAVLink docs: System is active and might be already airborne. Motors are engaged.
            Self::Filling
            | Self::Pressurizing
            | Self::Hold
            | Self::Venting
            | Self::Armed
            | Self::Ignition
            | Self::Burn
            | Self::Coast => MavState::Active,
            // From MAVLink docs: System is terminating itself (failsafe or commanded).
            Self::RecoveryDrogue | Self::RecoveryMain | Self::Landed => MavState::FlightTermination,
        }
    }
}

impl FlightMode {
    pub const ALL: [FlightMode; 13] = [
        Self::Idle,
        Self::HardwareArmed,
        Self::Filling,
        Self::Venting,
        Self::Pressurizing,
        Self::Hold,
        Self::Armed,
        Self::Ignition,
        Self::Burn,
        Self::Coast,
        Self::RecoveryDrogue,
        Self::RecoveryMain,
        Self::Landed,
    ];

    /// Name as a 35-length char array, as included in AVAILABLE_MODES MAVLink messages. Must
    /// include null termination character.
    pub fn mavlink_name(self) -> [u8; 35] {
        let string = match self {
            Self::Idle => "IDLE",
            Self::HardwareArmed => "HWARMED",
            Self::Filling => "FILL",
            Self::Venting => "VENT",
            Self::Pressurizing => "PRESS",
            Self::Hold => "HOLD",
            Self::Armed => "ARMED",
            Self::Ignition => "IGNITION",
            Self::Burn => "BURN",
            Self::Coast => "COAST",
            Self::RecoveryDrogue => "DROGUE",
            Self::RecoveryMain => "MAIN",
            Self::Landed => "LANDED",
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
