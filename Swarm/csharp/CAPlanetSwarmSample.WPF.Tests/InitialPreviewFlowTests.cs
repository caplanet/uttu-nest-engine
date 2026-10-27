using CAPlanetSwarmSample.WPF.Services;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public class InitialPreviewFlowTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task StopOrDisposalRepeatedlyCancelsInitWithoutPublishingPartialResult(bool obsolete)
    {
        var init = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        int requests = 0, published = 0;
        var result = await InitialPreviewFlow.RunAsync(init.Task,
            () => { published++; return Task.CompletedTask; }, () => obsolete,
            () => !obsolete, _ => Task.CompletedTask,
            () => { if (++requests == 3) init.SetResult(false); }).WaitAsync(TimeSpan.FromSeconds(5));
        Assert.Equal(InitialPreviewResult.Failed, result);
        Assert.Equal(3, requests); // Includes requests made before the native call entered.
        Assert.Equal(0, published);
    }

    [Fact]
    public async Task SuccessfulInitDoesNotReceiveCancellation()
    {
        int requests = 0;
        var result = await InitialPreviewFlow.RunAsync(Task.FromResult(true),
            () => Task.CompletedTask, () => false, () => false, _ => Task.CompletedTask,
            () => requests++);
        Assert.Equal(InitialPreviewResult.Ready, result);
        Assert.Equal(0, requests);
    }

    [Fact]
    public async Task DisposalDuringInitWaitsForNativeCompletionWithoutPublishing()
    {
        var init = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        bool obsolete = false;
        int published = 0;
        var run = InitialPreviewFlow.RunAsync(init.Task,
            () => { published++; return Task.CompletedTask; }, () => obsolete,
            () => true, _ => Task.CompletedTask);
        obsolete = true;
        Assert.False(run.IsCompleted);
        init.SetResult(true);
        Assert.Equal(InitialPreviewResult.Obsolete, await run);
        Assert.Equal(0, published);
    }

    [Fact]
    public async Task StopDuringInitStillAwaitsPreviewBeforeReturningStopped()
    {
        var init = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        var previewStarted = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var previewFinished = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        bool stop = false;
        var run = InitialPreviewFlow.RunAsync(init.Task, async () =>
        {
            previewStarted.SetResult();
            await previewFinished.Task;
        }, () => false, () => stop, _ => Task.CompletedTask);
        stop = true;
        Assert.False(previewStarted.Task.IsCompleted);
        init.SetResult(true);
        await previewStarted.Task.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.False(run.IsCompleted);
        previewFinished.SetResult();
        Assert.Equal(InitialPreviewResult.Stopped, await run);
    }

    [Theory]
    [InlineData(false, false, InitialPreviewResult.Failed)]
    [InlineData(true, true, InitialPreviewResult.Obsolete)]
    [InlineData(true, false, InitialPreviewResult.Ready)]
    public async Task OnlySuccessfulCurrentInitPublishes(bool initialized, bool obsolete, InitialPreviewResult expected)
    {
        int published = 0;
        var result = await InitialPreviewFlow.RunAsync(Task.FromResult(initialized),
            () => { published++; return Task.CompletedTask; }, () => obsolete,
            () => false, _ => Task.CompletedTask);
        Assert.Equal(expected, result);
        Assert.Equal(expected == InitialPreviewResult.Ready ? 1 : 0, published);
    }
}
