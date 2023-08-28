using System.Drawing;
using GameFramework;
using GameFramework.ImGui;
using GameFramework.Layers;
using GameFramework.Renderer.Batch;
using GameFramework.Scene;
using Veldrid;

namespace Common;

public abstract class VisualizationLayer3d : Layer, IDisposable
{
    protected readonly VisualizationApp App;
    protected readonly PerspectiveCameraController CameraController;
    private readonly ImGuiLayer _imGui;
    private readonly QuadBatch _mainBatch;

    private readonly List<QuadBatch> _batches = new();

    protected QuadBatch GetBatch()
    {
        CheckDisposed();
        var batch = App.Resources.BatchPool.Get();
        _batches.Add(batch);
        return batch;
    }

    public VisualizationLayer3d(VisualizationApp app, ImGuiLayer imGui)
    {
        App = app;
        _imGui = imGui;

        CameraController = PerspectiveCameraController.CreateDefault();
        CameraController.MoveSpeed = 5f;

        _mainBatch = GetBatch();

        UpdatePipelinesAndView();

        imGui.Submit += ImGuiOnSubmit;
    }

    protected abstract void ImGuiOnSubmit(ImGuiRenderer sender);

    private bool _rendering;
    private bool _userBatchesInitialized;

    protected void SetCameraTransform(QuadBatch batch)
    {
        batch.Effects = batch.Effects with { Transform = CameraController.Camera.CameraMatrix };
    }

    protected void RenderPass(QuadBatch batch, Action<QuadBatch> body)
    {
        CheckDisposed();

        SetCameraTransform(batch);
        
        batch.Clear();
        body(batch);
        batch.Submit();
    }

    protected void RenderPassMain(Action<QuadBatch> body)
    {
        CheckDisposed();

        if (!_rendering)
        {
            throw new InvalidOperationException();
        }

        RenderPass(_mainBatch, body);
    }

    protected sealed override void Render(FrameInfo frameInfo)
    {
        _rendering = true;
        RenderStack();
        _rendering = false;
    }

    protected abstract void RenderStack();

    protected override void OnAdded()
    {
        RegisterHandler<MouseEvent>(HandleMouse);

        UpdateBatchPipelines(App.Device.SwapchainFramebuffer.OutputDescription);
        _userBatchesInitialized = true;
    }

    protected virtual bool HandleMouse(MouseEvent e)
    {
        return true;
    }

    protected override void Resize(Size size)
    {
        base.Resize(size);

        UpdatePipelinesAndView();
    }

    private void UpdatePipelinesAndView()
    {
        var outputDesc = App.Device.SwapchainFramebuffer.OutputDescription;

        _mainBatch.UpdatePipelines(
            outputDescription: outputDesc,
            depthStencilState: DepthStencilStateDescription.DepthOnlyLessEqual,
            blendState: BlendStateDescription.SingleAlphaBlend
        );

        if (_userBatchesInitialized)
        {
            UpdateBatchPipelines(outputDesc);
        }

        CameraController.SetAspect(App.Window.Width, App.Window.Height);
    }
   
    protected virtual void UpdateBatchPipelines(OutputDescription outputDesc)
    {

    }

    protected override void Update(FrameInfo frameInfo)
    {
        CheckDisposed();

        base.Update(frameInfo);

        if (App.InCameraMode)
        {
            UpdateCamera(frameInfo);
        }
    }
  
    private void UpdateCamera(FrameInfo frameInfo)
    {
        if (_imGui.Captured)
        {
            return;
        }

        foreach (var key in App.Input.DownKeys)
        {
            CameraController.ProcessKey(key, frameInfo.DeltaTime);
        }

        CameraController.ProcessMouse(App.Input.MouseDelta, frameInfo.DeltaTime);
    }

    private void CheckDisposed()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
    }

    private bool _disposed;

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;

        foreach (var quadBatch in _batches)
        {
            App.Resources.BatchPool.Return(quadBatch);
        }
    }
}