use jagua_rs::Instant;
use std::time::Duration;

/// Generic trait for any struct that can determine if the optimization process should terminate.
pub trait Terminator: Sync {
    /// Checks if the termination condition is met
    fn kill(&self) -> bool;

    /// Returns true only for a persistent external cancellation request.
    ///
    /// A phase-local timeout also makes [`Self::kill`] true, but must not be
    /// mistaken for a request to cancel the remaining optimizer stages.
    fn stop_requested(&self) -> bool {
        // Preserve the historical meaning for third-party Terminator
        // implementations. Timeout-aware implementations can override this
        // method to distinguish their local deadline from persistent stop.
        self.kill()
    }

    /// Sets a new timeout duration
    fn new_timeout(&mut self, timeout: Duration);

    /// Returns the instant when a timeout was set, if any
    fn timeout_at(&self) -> Option<Instant>;
}

#[derive(Debug, Clone)]
pub struct BasicTerminator {
    pub timeout: Option<Instant>,
}

impl Default for BasicTerminator {
    fn default() -> Self {
        Self::new()
    }
}

impl BasicTerminator {
    pub fn new() -> Self {
        Self { timeout: None }
    }
}

impl Terminator for BasicTerminator {
    fn kill(&self) -> bool {
        self.timeout.is_some_and(|timeout| Instant::now() > timeout)
    }

    fn stop_requested(&self) -> bool {
        false
    }

    fn new_timeout(&mut self, timeout: Duration) {
        self.timeout = Some(Instant::now() + timeout);
    }

    fn timeout_at(&self) -> Option<Instant> {
        self.timeout
    }
}

/// Adds the current Init request to a local deadline; captures it for worker use.
pub struct InitTerminator<T> {
    inner: T,
    token: Option<crate::config::runtime::InitCancelToken>,
}
impl<T> InitTerminator<T> {
    pub fn new(inner: T) -> Self {
        Self { inner, token: crate::config::runtime::init_cancel_token() }
    }
}
impl<T: Terminator> Terminator for InitTerminator<T> {
    fn kill(&self) -> bool { self.stop_requested() || self.inner.kill() }
    fn stop_requested(&self) -> bool {
        self.token.is_some_and(|t| t.is_cancelled()) || self.inner.stop_requested()
    }
    fn new_timeout(&mut self, timeout: Duration) { self.inner.new_timeout(timeout); }
    fn timeout_at(&self) -> Option<Instant> { self.inner.timeout_at() }
}
