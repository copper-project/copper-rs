//! Linux PHC reads and bounded ptp4l management queries.

use cu29::clock::sync::{ClockDomain, ClockObservation};
use cu29::clock_sync::{ClockReference, ClockReferenceBundle};
use cu29::prelude::*;
use cu29::resource::{BundleContext, ResourceBundle, ResourceManager};
use std::fs::File;
use std::os::fd::AsRawFd;
use std::os::linux::net::SocketAddrExt;
use std::os::unix::net::{SocketAddr, UnixDatagram};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

const DEFAULT_DS: u16 = 0x2000;
const TIME_PROPERTIES: u16 = 0x2003;
const PORT_DS: u16 = 0x2004;
const TIME_STATUS: u16 = 0xc000;
const PORT_HWCLOCK: u16 = 0xc009;
const QUERY_TIMEOUT: Duration = Duration::from_millis(100);
const PTP_TIMESCALE: u8 = 0x08;
const UTC_OFFSET_VALID: u8 = 0x04;
const EMPTY_DOMAIN: ClockDomain = ClockDomain {
    id: 0,
    identity: [0; 8],
    session: 0,
};

/// Read-only Linux hardware-clock reference. Opening I/O is deferred until start.
///
/// Experimental. Reference uncertainty is an operator-supplied bound relative
/// to the root, combined with ptp4l's measured offset and capture uncertainty.
pub struct LinuxPhc {
    device: String,
    management: String,
    domain_number: u8,
    port: u16,
    reference_error_ns: u64,
    max_reference_age_ns: u64,
    allow_local_master: bool,
    domain: ClockDomain,
    io: Option<LinuxIo>,
}

impl LinuxPhc {
    /// Configures a PHC and a ptp4l management socket. No clock is adjusted.
    pub fn open(device: &str, management: &str, reference_error_ns: u64) -> CuResult<Self> {
        if reference_error_ns == 0 {
            return Err(CuError::from("PTP reference_error_ns must be positive"));
        }
        Ok(Self {
            device: device.into(),
            management: management.into(),
            domain_number: 0,
            port: 1,
            reference_error_ns,
            max_reference_age_ns: 5_000_000_000,
            allow_local_master: false,
            domain: EMPTY_DOMAIN,
            io: None,
        })
    }

    fn configured(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let config = config.ok_or(CuError::from(
            "LinuxPtpBundle requires device, management and reference_error_ns",
        ))?;
        let device = config
            .get::<String>("device")?
            .ok_or(CuError::from("LinuxPtpBundle requires device"))?;
        let management = config
            .get::<String>("management")?
            .ok_or(CuError::from("LinuxPtpBundle requires management"))?;
        let error = config
            .get::<u64>("reference_error_ns")?
            .ok_or(CuError::from("LinuxPtpBundle requires reference_error_ns"))?;
        let mut reference = Self::open(&device, &management, error)?;
        reference.domain_number = config.get::<u8>("domain")?.unwrap_or(0);
        reference.port = config.get::<u16>("port")?.unwrap_or(1);
        reference.max_reference_age_ns = config
            .get::<u64>("max_reference_age_ns")?
            .unwrap_or(5_000_000_000);
        reference.allow_local_master = config.get::<bool>("allow_local_master")?.unwrap_or(false);
        if reference.port == 0 || reference.max_reference_age_ns == 0 {
            return Err(CuError::from(
                "PTP port and max_reference_age_ns must be positive",
            ));
        }
        Ok(reference)
    }
}

/// Resource provider exporting `reference` for `runtime.clock.parent`.
pub struct LinuxPtpBundle;
cu29::bundle_resources!(LinuxPtpBundle: Reference = "reference");
impl ClockReferenceBundle for LinuxPtpBundle {
    type Reference = LinuxPhc;
}
impl ResourceBundle for LinuxPtpBundle {
    fn build(
        bundle: BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        manager.add_owned(
            bundle.key(LinuxPtpBundleId::Reference),
            LinuxPhc::configured(config)?,
        )
    }
}

impl ClockReference for LinuxPhc {
    fn create_clock(&self) -> CuResult<RobotClock> {
        Ok(RobotClock::new())
    }
    fn start(&mut self) -> CuResult<()> {
        let path = std::fs::canonicalize(&self.device)
            .map_err(|e| CuError::new_with_cause("Failed to resolve PHC device", e))?;
        let index = path
            .file_name()
            .and_then(|name| name.to_str())
            .and_then(|name| name.strip_prefix("ptp"))
            .and_then(|number| number.parse::<i32>().ok())
            .ok_or(CuError::from("PHC device must resolve to /dev/ptpN"))?;
        let phc = File::open(&path)
            .map_err(|e| CuError::new_with_cause("Failed to open PHC read-only", e))?;
        let management =
            ManagementClient::connect(Path::new(&self.management), self.domain_number, self.port)?;
        self.io = Some(LinuxIo {
            phc,
            management,
            index,
        });
        Ok(())
    }
    fn domain(&self) -> ClockDomain {
        self.domain
    }
    fn poll(&mut self, clock: &RobotClock) -> CuResult<Option<ClockObservation>> {
        let io = self
            .io
            .as_mut()
            .ok_or(CuError::from("PTP reference must be started"))?;
        let policy = CapturePolicy {
            reference_error_ns: self.reference_error_ns,
            max_age_ns: self.max_reference_age_ns,
            allow_local_master: self.allow_local_master,
        };
        let sample = capture(io, clock, policy)?;
        if let Some(sample) = sample {
            self.domain = sample.domain;
        }
        Ok(sample)
    }
    fn stop(&mut self) -> CuResult<()> {
        self.io = None;
        Ok(())
    }
}

trait ReferenceIo {
    fn health(&mut self) -> CuResult<Health>;
    fn parent_now(&mut self) -> CuResult<u64>;
}

#[derive(Clone, Copy)]
struct Health {
    domain: ClockDomain,
    local_identity: [u8; 8],
    port_state: u8,
    gm_present: bool,
    ptp_timescale: bool,
    utc_offset_valid: bool,
    utc_offset: i16,
    offset_ns: i64,
    ingress_ns: i64,
}

#[derive(Clone, Copy)]
struct CapturePolicy {
    reference_error_ns: u64,
    max_age_ns: u64,
    allow_local_master: bool,
}

fn capture(
    io: &mut impl ReferenceIo,
    clock: &RobotClock,
    policy: CapturePolicy,
) -> CuResult<Option<ClockObservation>> {
    let health = io.health()?;
    let local_master = policy.allow_local_master
        && health.port_state == 6
        && health.domain.identity == health.local_identity;
    if !local_master && !(health.port_state == 9 && health.gm_present) {
        return Ok(None);
    }
    if !health.ptp_timescale && !health.utc_offset_valid {
        return Ok(None);
    }
    let before = clock.raw_now();
    let parent = io.parent_now()?;
    let after = clock.raw_now();
    if after < before {
        return Err(CuError::from("Raw clock moved backward during PHC capture"));
    }
    if !local_master {
        let ingress = u64::try_from(health.ingress_ns)
            .map_err(|_| CuError::from("Invalid ptp4l ingress timestamp"))?;
        if ingress > parent || parent - ingress > policy.max_age_ns {
            return Ok(None);
        }
    }
    let tai = if health.ptp_timescale {
        parent
    } else {
        let tai = i128::from(parent) + i128::from(health.utc_offset) * 1_000_000_000;
        u64::try_from(tai).map_err(|_| CuError::from("PTP UTC-to-TAI conversion overflow"))?
    };
    if tai >= CuTime::MAX.0 {
        return Err(CuError::from("PHC time exceeds Copper timestamp range"));
    }
    let uncertainty = policy
        .reference_error_ns
        .checked_add(health.offset_ns.unsigned_abs())
        .and_then(|error| error.checked_add((after - before).as_nanos().div_ceil(2)))
        .ok_or(CuError::from("PTP capture uncertainty overflow"))?;
    Ok(Some(ClockObservation {
        raw_local: before + (after - before) / 2u64,
        parent_ns: tai,
        uncertainty: CuDuration(uncertainty),
        domain: health.domain,
    }))
}

struct LinuxIo {
    phc: File,
    management: ManagementClient,
    index: i32,
}
impl ReferenceIo for LinuxIo {
    fn health(&mut self) -> CuResult<Health> {
        let health = self.management.health()?;
        let hardware = self.management.query(PORT_HWCLOCK)?;
        let index = i32::from_be_bytes(field(&hardware, 10)?);
        if index != self.index {
            return Err(CuError::from(
                "Selected PHC differs from ptp4l port's hardware clock",
            ));
        }
        Ok(health)
    }
    fn parent_now(&mut self) -> CuResult<u64> {
        let id: libc::clockid_t = (!self.phc.as_raw_fd() << 3) | 3;
        let mut time = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        // SAFETY: the PHC fd remains open and time points to a valid timespec.
        if unsafe { libc::clock_gettime(id, &mut time) } != 0 {
            return Err(CuError::new_with_cause(
                "Failed to read PHC",
                std::io::Error::last_os_error(),
            ));
        }
        if time.tv_sec < 0 || !(0..1_000_000_000).contains(&time.tv_nsec) {
            return Err(CuError::from("Invalid PHC timespec"));
        }
        let nanos = i128::from(time.tv_sec) * 1_000_000_000 + i128::from(time.tv_nsec);
        u64::try_from(nanos).map_err(|_| CuError::from("PHC timestamp overflow"))
    }
}

fn field<const N: usize>(data: &[u8], offset: usize) -> CuResult<[u8; N]> {
    data.get(offset..offset + N)
        .and_then(|field| field.try_into().ok())
        .ok_or(CuError::from("Truncated PTP management data"))
}

struct ManagementClient {
    socket: UnixDatagram,
    domain: u8,
    port: u16,
    sequence: u16,
}
impl ManagementClient {
    fn connect(path: &Path, domain: u8, port: u16) -> CuResult<Self> {
        static CLIENT_ID: AtomicU64 = AtomicU64::new(0);
        let name = format!(
            "cu-ptp-{}-{}",
            std::process::id(),
            CLIENT_ID.fetch_add(1, Ordering::Relaxed)
        );
        let address = SocketAddr::from_abstract_name(name.as_bytes())
            .map_err(|e| CuError::new_with_cause("Invalid PTP client address", e))?;
        let socket = UnixDatagram::bind_addr(&address)
            .map_err(|e| CuError::new_with_cause("Failed to bind PTP management client", e))?;
        socket.connect(path).map_err(|e| {
            CuError::new_with_cause("Failed to connect to ptp4l management socket", e)
        })?;
        socket
            .set_read_timeout(Some(QUERY_TIMEOUT))
            .map_err(|e| CuError::new_with_cause("Failed to set PTP query timeout", e))?;
        socket
            .set_write_timeout(Some(QUERY_TIMEOUT))
            .map_err(|e| CuError::new_with_cause("Failed to set PTP send timeout", e))?;
        Ok(Self {
            socket,
            domain,
            port,
            sequence: 0,
        })
    }

    fn request(&mut self, id: u16) -> [u8; 54] {
        self.sequence = self.sequence.wrapping_add(1);
        let mut request = [0u8; 54];
        request[0] = 0x0d;
        request[1] = 0x02;
        request[2..4].copy_from_slice(&54u16.to_be_bytes());
        request[4] = self.domain;
        request[20..28].copy_from_slice(&u64::from(std::process::id()).to_be_bytes());
        request[28..30].copy_from_slice(&1u16.to_be_bytes());
        request[30..32].copy_from_slice(&self.sequence.to_be_bytes());
        request[32] = 4;
        request[33] = 0x7f;
        request[34..42].fill(0xff);
        request[42..44].copy_from_slice(&self.port.to_be_bytes());
        request[44] = 1;
        request[45] = 1;
        request[48..50].copy_from_slice(&1u16.to_be_bytes());
        request[50..52].copy_from_slice(&2u16.to_be_bytes());
        request[52..54].copy_from_slice(&id.to_be_bytes());
        request
    }

    fn response<'a>(&self, packet: &'a [u8], id: u16) -> CuResult<Option<&'a [u8]>> {
        if packet.len() < 54 {
            return Err(CuError::from("Truncated PTP management response"));
        }
        if packet[0] & 0xf != 0xd
            || packet[1] & 0xf != 2
            || packet[4] != self.domain
            || u16::from_be_bytes(field(packet, 30)?) != self.sequence
        {
            return Ok(None);
        }
        let length = usize::from(u16::from_be_bytes(field(packet, 2)?));
        let tlv_length = usize::from(u16::from_be_bytes(field(packet, 50)?));
        if length != packet.len()
            || tlv_length < 2
            || 52 + tlv_length != length
            || packet[46] & 0xf != 2
        {
            return Err(CuError::from(
                "Invalid PTP management response length/action",
            ));
        }
        if u16::from_be_bytes(field(packet, 48)?) == 2 {
            return Err(CuError::from("ptp4l rejected management query"));
        }
        if u16::from_be_bytes(field(packet, 48)?) != 1
            || u16::from_be_bytes(field(packet, 52)?) != id
        {
            return Ok(None);
        }
        Ok(Some(&packet[54..]))
    }

    fn query(&mut self, id: u16) -> CuResult<Vec<u8>> {
        let request = self.request(id);
        self.socket
            .send(&request)
            .map_err(|e| CuError::new_with_cause("PTP management send failed", e))?;
        let deadline = Instant::now() + QUERY_TIMEOUT;
        let mut packet = [0u8; 512];
        for _ in 0..16 {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            self.socket
                .set_read_timeout(Some(remaining))
                .map_err(|e| CuError::new_with_cause("PTP timeout setup failed", e))?;
            let used = self
                .socket
                .recv(&mut packet)
                .map_err(|e| CuError::new_with_cause("PTP management receive failed", e))?;
            if let Some(data) = self.response(&packet[..used], id)? {
                return Ok(data.to_vec());
            }
        }
        Err(CuError::from("PTP management response deadline exceeded"))
    }

    fn health(&mut self) -> CuResult<Health> {
        let defaults = self.query(DEFAULT_DS)?;
        let properties = self.query(TIME_PROPERTIES)?;
        let port = self.query(PORT_DS)?;
        let status = self.query(TIME_STATUS)?;
        if field::<1>(&defaults, 18)?[0] != self.domain {
            return Err(CuError::from("ptp4l domain differs from selected domain"));
        }
        let flags = field::<1>(&properties, 2)?[0];
        Ok(Health {
            domain: ClockDomain {
                id: self.domain,
                identity: field(&status, 42)?,
                session: u64::from(u16::from_be_bytes(field(&status, 24)?)),
            },
            local_identity: field(&defaults, 10)?,
            port_state: field::<1>(&port, 10)?[0],
            gm_present: i32::from_be_bytes(field(&status, 38)?) != 0,
            ptp_timescale: flags & PTP_TIMESCALE != 0,
            utc_offset_valid: flags & UTC_OFFSET_VALID != 0,
            utc_offset: i16::from_be_bytes(field(&properties, 0)?),
            offset_ns: i64::from_be_bytes(field(&status, 0)?),
            ingress_ns: i64::from_be_bytes(field(&status, 8)?),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const DOMAIN: ClockDomain = ClockDomain {
        id: 0,
        identity: *b"testroot",
        session: 1,
    };
    struct MockIo {
        health: Health,
        time: u64,
        mock: RobotClockMock,
        delay: u64,
        reads: usize,
    }
    impl ReferenceIo for MockIo {
        fn health(&mut self) -> CuResult<Health> {
            Ok(self.health)
        }
        fn parent_now(&mut self) -> CuResult<u64> {
            self.reads += 1;
            self.mock.increment(CuDuration(self.delay));
            Ok(self.time)
        }
    }
    fn fixture() -> (RobotClock, MockIo, CapturePolicy) {
        let (clock, mock) = RobotClock::mock();
        (
            clock,
            MockIo {
                health: Health {
                    domain: DOMAIN,
                    local_identity: *b"follower",
                    port_state: 9,
                    gm_present: true,
                    ptp_timescale: true,
                    utc_offset_valid: true,
                    utc_offset: 37,
                    offset_ns: -200,
                    ingress_ns: 1_000_000_000,
                },
                time: 2_000_000_000,
                mock,
                delay: 101,
                reads: 0,
            },
            CapturePolicy {
                reference_error_ns: 100,
                max_age_ns: 5_000_000_000,
                allow_local_master: false,
            },
        )
    }
    #[test]
    fn capture_brackets_raw_counter_and_includes_upstream_error() {
        let (clock, mut io, policy) = fixture();
        let sample = capture(&mut io, &clock, policy).unwrap().unwrap();
        assert_eq!(sample.raw_local, CuInstant::from_nanos(50));
        assert_eq!(sample.uncertainty, CuDuration(351));
        assert_eq!(sample.parent_ns, io.time);
    }
    #[test]
    fn ticking_phc_does_not_hide_upstream_loss() {
        let (clock, mut io, policy) = fixture();
        io.health.gm_present = false;
        assert!(capture(&mut io, &clock, policy).unwrap().is_none());
        assert_eq!(io.reads, 0);
        io.health.gm_present = true;
        io.health.port_state = 7;
        assert!(capture(&mut io, &clock, policy).unwrap().is_none());
        io.health.port_state = 9;
        io.time = 10_000_000_000;
        assert!(capture(&mut io, &clock, policy).unwrap().is_none());
    }
    #[test]
    fn utc_requires_valid_offset_and_normalizes_to_tai() {
        let (clock, mut io, policy) = fixture();
        io.health.ptp_timescale = false;
        io.health.utc_offset_valid = false;
        assert!(capture(&mut io, &clock, policy).unwrap().is_none());
        io.health.utc_offset_valid = true;
        assert_eq!(
            capture(&mut io, &clock, policy).unwrap().unwrap().parent_ns,
            39_000_000_000
        );
    }
    #[test]
    fn local_master_requires_explicit_selection_and_root_identity() {
        let (clock, mut io, mut policy) = fixture();
        io.health.port_state = 6;
        io.health.gm_present = false;
        assert!(capture(&mut io, &clock, policy).unwrap().is_none());
        policy.allow_local_master = true;
        assert!(capture(&mut io, &clock, policy).unwrap().is_none());
        io.health.local_identity = DOMAIN.identity;
        assert!(capture(&mut io, &clock, policy).unwrap().is_some());
    }
    #[test]
    fn native_software_capture_queries_real_datagrams_without_ptp_hardware() {
        let (client, server) = UnixDatagram::pair().unwrap();
        let mut io = SystemIo {
            management: ManagementClient {
                socket: client,
                domain: 0,
                port: 1,
                sequence: 0,
            },
            utc_offset_seconds: 37,
        };
        let peer = std::thread::spawn(move || {
            for id in [DEFAULT_DS, TIME_PROPERTIES, PORT_DS, TIME_STATUS, 0xc004] {
                let mut request = [0; 512];
                let n = server.recv(&mut request).unwrap();
                assert_eq!(u16::from_be_bytes(field(&request, 52).unwrap()), id);
                let size = match id {
                    DEFAULT_DS => 20,
                    TIME_PROPERTIES => 4,
                    PORT_DS => 26,
                    TIME_STATUS => 50,
                    0xc004 => 14,
                    _ => unreachable!(),
                };
                let mut data = vec![0; size];
                match id {
                    DEFAULT_DS => data[10..18].copy_from_slice(&DOMAIN.identity),
                    PORT_DS => data[10] = 6,
                    TIME_STATUS => {
                        data[42..50].copy_from_slice(&DOMAIN.identity);
                        data[24..26].copy_from_slice(&1u16.to_be_bytes());
                    }
                    _ => {}
                }
                let mut response = request[..n].to_vec();
                response[46] = 2;
                response.extend(data);
                let len = response.len() as u16;
                response[2..4].copy_from_slice(&len.to_be_bytes());
                response[50..52].copy_from_slice(&(len - 52).to_be_bytes());
                server.send(&response).unwrap();
            }
        });
        let clock = RobotClock::new();
        let before = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
            + 37_000_000_000;
        let sample = capture(
            &mut io,
            &clock,
            CapturePolicy {
                reference_error_ns: 5_000_000,
                max_age_ns: 5_000_000_000,
                allow_local_master: true,
            },
        )
        .unwrap()
        .unwrap();
        assert_eq!(sample.domain, DOMAIN);
        assert!(sample.parent_ns >= before);
        assert!(sample.uncertainty.0 >= 5_000_000);
        peer.join().unwrap();
    }

    #[test]
    fn binary_management_roundtrip_checks_sequence_lengths_and_action() {
        let (client, server) = UnixDatagram::pair().unwrap();
        client.set_read_timeout(Some(QUERY_TIMEOUT)).unwrap();
        let mut manager = ManagementClient {
            socket: client,
            domain: 0,
            port: 1,
            sequence: 0,
        };
        let server = std::thread::spawn(move || {
            let mut request = [0; 512];
            let n = server.recv(&mut request).unwrap();
            assert_eq!(n, 54);
            assert_eq!(request[46], 0);
            assert_eq!(
                u16::from_be_bytes(field(&request, 52).unwrap()),
                TIME_STATUS
            );
            let mut response = request[..n].to_vec();
            response[46] = 2;
            response.extend_from_slice(&[1; 50]);
            let len = response.len() as u16;
            response[2..4].copy_from_slice(&len.to_be_bytes());
            response[50..52].copy_from_slice(&(len - 52).to_be_bytes());
            server.send(&response).unwrap();
        });
        assert_eq!(manager.query(TIME_STATUS).unwrap(), vec![1; 50]);
        server.join().unwrap();
        let mut response = manager.request(TIME_STATUS);
        response[46] = 2;
        assert!(manager.response(&response[..53], TIME_STATUS).is_err());
        response[30] ^= 1;
        assert!(manager.response(&response, TIME_STATUS).unwrap().is_none());
        response[30] ^= 1;
        response[2] = 0xff;
        assert!(manager.response(&response, TIME_STATUS).is_err());
    }
}

/// Software-timestamped ptp4l reference using Linux CLOCK_REALTIME as UTC.
///
/// Experimental. Supply the known TAI−UTC offset for the experiment. The
/// management service must run with software timestamping on the selected port.
pub struct LinuxSystemPtp {
    reference: LinuxPhc,
    utc_offset_seconds: i16,
    io: Option<SystemIo>,
}

impl LinuxSystemPtp {
    /// Configures a system-clock reference with an explicit, known leap offset.
    pub fn new(
        management: &str,
        reference_error_ns: u64,
        utc_offset_seconds: i16,
    ) -> CuResult<Self> {
        Ok(Self {
            reference: LinuxPhc::open("", management, reference_error_ns)?,
            utc_offset_seconds,
            io: None,
        })
    }
}

struct SystemIo {
    management: ManagementClient,
    utc_offset_seconds: i16,
}
impl ReferenceIo for SystemIo {
    fn health(&mut self) -> CuResult<Health> {
        let mut health = self.management.health()?;
        let properties = self.management.query(0xc004)?; // PORT_PROPERTIES_NP
        if field::<1>(&properties, 11)?[0] != 0 {
            // TS_SOFTWARE
            return Err(CuError::from(
                "System PTP reference requires ptp4l software timestamping",
            ));
        }
        health.ptp_timescale = false;
        health.utc_offset_valid = true;
        health.utc_offset = self.utc_offset_seconds;
        Ok(health)
    }
    fn parent_now(&mut self) -> CuResult<u64> {
        let mut time = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        // SAFETY: CLOCK_REALTIME is valid and time is a writable timespec.
        if unsafe { libc::clock_gettime(libc::CLOCK_REALTIME, &mut time) } != 0 {
            return Err(CuError::new_with_cause(
                "Failed to read system PTP clock",
                std::io::Error::last_os_error(),
            ));
        }
        let ns = i128::from(time.tv_sec) * 1_000_000_000 + i128::from(time.tv_nsec);
        u64::try_from(ns).map_err(|_| CuError::from("System PTP timestamp overflow"))
    }
}

impl ClockReference for LinuxSystemPtp {
    fn create_clock(&self) -> CuResult<RobotClock> {
        Ok(RobotClock::new())
    }
    fn start(&mut self) -> CuResult<()> {
        self.io = Some(SystemIo {
            management: ManagementClient::connect(
                Path::new(&self.reference.management),
                self.reference.domain_number,
                self.reference.port,
            )?,
            utc_offset_seconds: self.utc_offset_seconds,
        });
        Ok(())
    }
    fn domain(&self) -> ClockDomain {
        self.reference.domain
    }
    fn poll(&mut self, clock: &RobotClock) -> CuResult<Option<ClockObservation>> {
        let io = self
            .io
            .as_mut()
            .ok_or(CuError::from("System PTP reference must be started"))?;
        let sample = capture(
            io,
            clock,
            CapturePolicy {
                reference_error_ns: self.reference.reference_error_ns,
                max_age_ns: self.reference.max_reference_age_ns,
                allow_local_master: self.reference.allow_local_master,
            },
        )?;
        if let Some(sample) = sample {
            self.reference.domain = sample.domain;
        }
        Ok(sample)
    }
    fn stop(&mut self) -> CuResult<()> {
        self.io = None;
        Ok(())
    }
}

/// Software-timestamped PTP resource for local Linux experiments.
pub struct LinuxSystemPtpBundle;
cu29::bundle_resources!(LinuxSystemPtpBundle: Reference = "reference");
impl ClockReferenceBundle for LinuxSystemPtpBundle {
    type Reference = LinuxSystemPtp;
}
impl ResourceBundle for LinuxSystemPtpBundle {
    fn build(
        bundle: BundleContext<Self>,
        config: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        let config = config.ok_or(CuError::from(
            "LinuxSystemPtpBundle requires management, reference_error_ns and utc_offset_seconds",
        ))?;
        let management = config
            .get::<String>("management")?
            .ok_or(CuError::from("System PTP requires management"))?;
        let error = config
            .get::<u64>("reference_error_ns")?
            .ok_or(CuError::from("System PTP requires reference_error_ns"))?;
        let offset = config
            .get::<i16>("utc_offset_seconds")?
            .ok_or(CuError::from(
                "System PTP requires a known utc_offset_seconds",
            ))?;
        let mut parent = LinuxSystemPtp::new(&management, error, offset)?;
        parent.reference.domain_number = config.get::<u8>("domain")?.unwrap_or(0);
        parent.reference.port = config.get::<u16>("port")?.unwrap_or(1);
        parent.reference.max_reference_age_ns = config
            .get::<u64>("max_reference_age_ns")?
            .unwrap_or(5_000_000_000);
        parent.reference.allow_local_master =
            config.get::<bool>("allow_local_master")?.unwrap_or(false);
        if parent.reference.port == 0 || parent.reference.max_reference_age_ns == 0 {
            return Err(CuError::from(
                "System PTP port and reference age must be positive",
            ));
        }
        manager.add_owned(bundle.key(LinuxSystemPtpBundleId::Reference), parent)
    }
}
