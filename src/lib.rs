#![no_std]

#[allow(unused_imports)]
#[allow(dead_code)]
mod mavlink {
    use core::{concat, env, include};

    include!(concat!(env!("OUT_DIR"), "/mavlink/mod.rs"));
}

pub use mavlink::dialects::rapid;
pub use mavlink::dialects::Rapid;

// TODO: mavspec generates these intos/froms for messages defined in the same standard,
// but not for included ones. we should generate these automatically.
impl Into<Rapid> for rapid::messages::Heartbeat {
    fn into(self) -> Rapid {
        Rapid::Heartbeat(self)
    }
}

impl Into<Rapid> for rapid::messages::Attitude {
    fn into(self) -> Rapid {
        Rapid::Attitude(self)
    }
}

impl Into<Rapid> for rapid::messages::LocalPositionNed {
    fn into(self) -> Rapid {
        Rapid::LocalPositionNed(self)
    }
}

impl Into<Rapid> for rapid::messages::ScaledImu {
    fn into(self) -> Rapid {
        Rapid::ScaledImu(self)
    }
}

impl Into<Rapid> for rapid::messages::ScaledImu2 {
    fn into(self) -> Rapid {
        Rapid::ScaledImu2(self)
    }
}

impl Into<Rapid> for rapid::messages::ScaledImu3 {
    fn into(self) -> Rapid {
        Rapid::ScaledImu3(self)
    }
}

impl Into<Rapid> for rapid::messages::ScaledPressure {
    fn into(self) -> Rapid {
        Rapid::ScaledPressure(self)
    }
}

impl Into<Rapid> for rapid::messages::ScaledPressure2 {
    fn into(self) -> Rapid {
        Rapid::ScaledPressure2(self)
    }
}

impl Into<Rapid> for rapid::messages::ScaledPressure3 {
    fn into(self) -> Rapid {
        Rapid::ScaledPressure3(self)
    }
}

impl Into<Rapid> for rapid::messages::BatteryStatus {
    fn into(self) -> Rapid {
        Rapid::BatteryStatus(self)
    }
}

use mavlink::dialects::minimal::enums::MavState;

// TODO: Do we need/want our flight mode type to live here or do we want to move it to the
// firmware?

#[derive(Default, Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Hash)]
pub enum FlightMode {
    #[default]
    Idle = 0,
    HardwareArmed = 1,
    Armed = 2,
    ArmedLaunchImminent = 3,
    Burn = 4,
    Coast = 5,
    RecoveryDrogue = 6,
    RecoveryMain = 7,
    Landed = 8,
}

impl TryFrom<u8> for FlightMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Idle),
            1 => Ok(Self::HardwareArmed),
            2 => Ok(Self::Armed),
            3 => Ok(Self::ArmedLaunchImminent),
            4 => Ok(Self::Burn),
            5 => Ok(Self::Coast),
            6 => Ok(Self::RecoveryDrogue),
            7 => Ok(Self::RecoveryMain),
            8 => Ok(Self::Landed),
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
            Self::Armed | Self::ArmedLaunchImminent | Self::Burn | Self::Coast => MavState::Active,
            // From MAVLink docs: System is terminating itself (failsafe or commanded).
            Self::RecoveryDrogue | Self::RecoveryMain | Self::Landed => MavState::FlightTermination,
        }
    }
}

impl FlightMode {
    pub const ALL: [FlightMode; 9] = [
        Self::Idle,
        Self::HardwareArmed,
        Self::Armed,
        Self::ArmedLaunchImminent,
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
            Self::Armed => "ARMED",
            Self::ArmedLaunchImminent => "IMMINENT",
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
