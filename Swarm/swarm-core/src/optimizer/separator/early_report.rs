//! Owner-thread delivery of completed worker results within one synchronous round.
pub(super) fn run_round<T: Send, R: Send, E: Send>(
    pool: &rayon::ThreadPool,
    workers: &mut [T],
    work: impl Fn(&mut T) -> (R, Option<E>) + Sync,
    mut deliver: impl FnMut(E),
) -> Vec<R> {
    // An unusual reentrant call from this very pool must never block its last
    // worker on receive. Its sequential fallback still retains per-worker RNG.
    if pool.current_thread_index().is_some() {
        return workers
            .iter_mut()
            .map(|worker| {
                let (result, event) = work(worker);
                if let Some(event) = event {
                    deliver(event);
                }
                result
            })
            .collect();
    }
    pool.in_place_scope(|scope| {
        // At most one message per worker; this queue cannot outlive the round.
        // Dropping the last sender also ends reception after a worker panic.
        let (sender, receiver) = std::sync::mpsc::channel();
        let mut results = Vec::with_capacity(workers.len());
        for (index, worker) in workers.iter_mut().enumerate() {
            let sender = sender.clone();
            let work = &work;
            scope.spawn(move |_| {
                let result = work(worker);
                let _ = sender.send((index, result));
            });
        }
        drop(sender);
        for (index, (result, event)) in receiver {
            if let Some(event) = event {
                deliver(event);
            }
            results.push((index, result));
        }
        results.sort_unstable_by_key(|(index, _)| *index);
        results.into_iter().map(|(_, result)| result).collect()
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        cell::Cell,
        sync::{Mutex, mpsc},
        time::Duration,
    };

    #[test]
    fn publishes_on_owner_before_slow_peer_finishes() {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(2)
            .build()
            .unwrap();
        let owner = std::thread::current().id();
        let (release, released) = mpsc::channel();
        let released = Mutex::new(released);
        let (started, ready) = mpsc::channel();
        let ready = Mutex::new(ready);
        let slow_saw_callback = Cell::new(false);
        let reports = run_round(
            &pool,
            &mut [0, 1],
            |index| {
                if *index == 1 {
                    started.send(()).unwrap();
                    let received = released
                        .lock()
                        .unwrap()
                        .recv_timeout(Duration::from_secs(2))
                        .is_ok();
                    (received, None)
                } else {
                    ready.lock().unwrap().recv().unwrap();
                    (true, Some(()))
                }
            },
            |_| {
                assert_eq!(owner, std::thread::current().id());
                slow_saw_callback.set(true); // !Send state deliberately stays on owner.
                let _ = release.send(());
            },
        );
        assert!(slow_saw_callback.get());
        assert!(
            reports[1],
            "slow worker ended before owner delivered the completed peer"
        );
    }

    #[test]
    fn one_worker_delivers_and_returns() {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap();
        let seen = Cell::new(0);
        assert_eq!(
            run_round(
                &pool,
                &mut [7],
                |value| (*value, Some(*value)),
                |event| seen.set(event)
            ),
            vec![7]
        );
        assert_eq!(seen.get(), 7);
    }

    #[test]
    fn worker_panic_closes_queue_and_propagates() {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            run_round(
                &pool,
                &mut [0],
                |_| -> ((), Option<()>) { panic!("{}", crate::dmsg!("worker failure")) },
                |_| {},
            );
        }));
        assert!(result.is_err());
    }
}
