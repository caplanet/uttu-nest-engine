using System;
using System.Diagnostics;
using System.Threading.Tasks;

namespace CAPlanetSwarmSample.WPF.Services;

public enum InitialPreviewResult { Failed, Obsolete, Stopped, Ready }

/// <summary>
/// Repeats Init cancellation until native completion, including requests made
/// before the native call enters. Retains an already completed initial preview.
/// Disposal/new-run invalidation still suppresses publication entirely.
/// </summary>
public static class InitialPreviewFlow
{
    public static async Task<InitialPreviewResult> RunAsync(
        Task<bool> initialization,
        Func<Task> publishPreview,
        Func<bool> isObsolete,
        Func<bool> isStopRequested,
        Func<TimeSpan, Task> reportWaiting,
        Action? cancelInitialization = null)
    {
        var elapsed = Stopwatch.StartNew();
        var nextReport = TimeSpan.FromSeconds(1);
        try
        {
            while (!initialization.IsCompleted)
            {
                if (isStopRequested() || isObsolete()) cancelInitialization?.Invoke();
                await Task.WhenAny(initialization, Task.Delay(100)).ConfigureAwait(false);
                if (!initialization.IsCompleted && !isObsolete() && elapsed.Elapsed >= nextReport)
                {
                    await reportWaiting(elapsed.Elapsed).ConfigureAwait(false);
                    nextReport = elapsed.Elapsed + TimeSpan.FromSeconds(1);
                }
            }
        }
        finally
        {
            // A dispatcher/progress failure must not dispose the optimizer
            // while its handle-producing native call is still running.
            await initialization.ConfigureAwait(false);
        }

        // Always join Init before the caller disposes its native optimizer.
        if (!await initialization.ConfigureAwait(false)) return InitialPreviewResult.Failed;
        if (isObsolete()) return InitialPreviewResult.Obsolete;

        await publishPreview().ConfigureAwait(false);
        if (isObsolete()) return InitialPreviewResult.Obsolete;
        return isStopRequested() ? InitialPreviewResult.Stopped : InitialPreviewResult.Ready;
    }
}
