//! Opt-in local diagnostics. Only one profiled Init may run per process.
//! Nested spans are inclusive; parallel item spans sum worker wall time.
use std::{collections::BTreeMap, sync::{Mutex, OnceLock}, time::Instant};
use serde::Serialize;

#[derive(Default, Serialize)]
struct Stat { calls: u64, total_ms: f64, max_ms: f64 }
#[derive(Default, Serialize)]
struct Data { spans: BTreeMap<&'static str, Stat>, counters: BTreeMap<&'static str, u64> }
static DATA: OnceLock<Mutex<Data>> = OnceLock::new();
fn data() -> &'static Mutex<Data> { DATA.get_or_init(|| Mutex::new(Data::default())) }

pub struct Session { start: Instant, path: Option<std::ffi::OsString> }
impl Session {
    pub fn begin() -> Self {
        *data().lock().unwrap() = Data::default();
        Self { start: Instant::now(), path: std::env::var_os("SWARM_INIT_PROFILE_PATH") }
    }
}
impl Drop for Session {
    fn drop(&mut self) {
        let total_ms = self.start.elapsed().as_secs_f64() * 1000.0;
        let Some(path) = &self.path else { return };
        let data = data().lock().unwrap();
        let report = serde_json::json!({"total_ms": total_ms, "data": &*data,
            "boundary": "native Init entry to exit, excluding profile file write; phase.* are disjoint, detail.* inclusive; parallel import item sums are not wall time"});
        match serde_json::to_vec_pretty(&report).map_err(std::io::Error::other)
            .and_then(|bytes| std::fs::write(path, bytes)) {
            Ok(()) => eprintln!("Init profile written: {} ({total_ms:.3} ms)", path.to_string_lossy()),
            Err(e) => eprintln!("Init profile write failed: {e}"),
        }
    }
}

pub struct Scope { name: &'static str, start: Option<Instant> }
impl Scope {
    pub fn new(name: &'static str) -> Self { Self { name, start: Some(Instant::now()) } }
    pub fn finish(&mut self) {
        if let Some(start) = self.start.take() {
            let ms = start.elapsed().as_secs_f64() * 1000.0;
            let mut data = data().lock().unwrap();
            let stat = data.spans.entry(self.name).or_default();
            stat.calls += 1;
            stat.total_ms += ms;
            stat.max_ms = stat.max_ms.max(ms);
        }
    }
    pub fn next(&mut self, name: &'static str) { self.finish(); *self = Self::new(name); }
}
impl Drop for Scope { fn drop(&mut self) { self.finish(); } }
pub fn count(name: &'static str, n: usize) {
    *data().lock().unwrap().counters.entry(name).or_default() += n as u64;
}
