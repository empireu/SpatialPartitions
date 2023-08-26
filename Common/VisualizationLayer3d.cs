using System.Drawing;
using GameFramework;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Layers;
using GameFramework.PostProcessing;
using GameFramework.Renderer.Batch;
using GameFramework.Scene;
using Veldrid;

namespace Common;

public abstract class VisualizationLayer3d : Layer, IDisposable
{
    protected readonly VisualizationApp App;
    private readonly ImGuiLayer _imGui;
    protected readonly PerspectiveCameraController CameraController;
    private readonly QuadBatch _batch;

    public VisualizationLayer3d(VisualizationApp app, ImGuiLayer imGui)
    {
        App = app;
        _imGui = imGui;

        CameraController = PerspectiveCameraController.CreateDefault();
        CameraController.MoveSpeed = 5f;

        _batch = app.Resources.BatchPool.Get();

        UpdatePipelinesAndView();

        imGui.Submit += ImGuiOnSubmit;
    }

    protected abstract void ImGuiOnSubmit(ImGuiRenderer sender);

    private bool _rendering;
    
    protected void RenderPass(Action<QuadBatch> body)
    {
        if (!_rendering)
        {
            throw new InvalidOperationException();
        }

        _batch.Effects = QuadBatchEffects.Transformed(CameraController.Camera.CameraMatrix);
        _batch.Clear();
        body(_batch);
        _batch.Submit();
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
    }

    protected virtual bool HandleMouse(MouseEvent e)
    {
        return true;
    }

    private void UpdatePipelinesAndView()
    {
        _batch.UpdatePipelines(
            outputDescription: App.Device.SwapchainFramebuffer.OutputDescription,
            depthStencilState: DepthStencilStateDescription.DepthOnlyLessEqual,
            blendState: BlendStateDescription.SingleAlphaBlend
        );

        CameraController.SetAspect(App.Window.Width, App.Window.Height);
    }

    protected override void Resize(Size size)
    {
        base.Resize(size);

        UpdatePipelinesAndView();
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

    protected override void Update(FrameInfo frameInfo)
    {
        base.Update(frameInfo);

        if (App.InCameraMode)
        {
            UpdateCamera(frameInfo);
        }
    }

    private bool _disposed;

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;

        App.Resources.BatchPool.Return(_batch);
    }
}