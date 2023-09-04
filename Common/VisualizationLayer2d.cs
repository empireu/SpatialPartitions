using System.Drawing;
using System.Numerics;
using GameFramework;
using GameFramework.Extensions;
using GameFramework.ImGui;
using GameFramework.Layers;
using GameFramework.PostProcessing;
using GameFramework.Renderer.Batch;
using GameFramework.Scene;
using Veldrid;

namespace Common;

public abstract class VisualizationLayer2d : Layer, IDisposable
{
    protected virtual float MinZoom => 1f;
    protected virtual float MaxZoom => 250f;
    protected virtual float CamDragSpeed => 5f;
    protected virtual float CamZoomSpeed => 50f;

    protected readonly VisualizationApp App;
    private readonly ImGuiLayer _imGui;


    private readonly List<QuadBatch> _batches = new();
    private readonly QuadBatch _mainBatch;
    private bool _dragCamera;
    private bool _disposed;
    private bool _rendering;
    private bool _userBatchesInitialized;

    public VisualizationLayer2d(VisualizationApp app, ImGuiLayer imGui)
    {
        App = app;

        _imGui = imGui;

        CameraController = new OrthographicCameraController2D(new OrthographicCamera(0, -1, 1), translationInterpolate: 25f, zoomInterpolate: 10f)
        {
            FutureZoom = 35
        };

        _mainBatch = GetBatch();

        UpdatePipelinesAndView();

        imGui.Submit += ImGuiOnSubmit;
    }

    public OrthographicCameraController2D CameraController { get; }

    protected QuadBatch GetBatch()
    {
        CheckDisposed();
        var batch = App.Resources.BatchPool.Get();
        _batches.Add(batch);
        return batch;
    }

    protected Vector2 Mouse => CameraController.Camera.MouseToWorld2D(App.Input.MousePosition, App.Window.Width, App.Window.Height);
    protected Vector2ds MouseI => Mouse.Map(mouse => new Vector2ds((int)MathF.Round(mouse.X), (int)MathF.Round(mouse.Y)));

    protected abstract void ImGuiOnSubmit(ImGuiRenderer sender);

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
        RegisterHandler<MouseEvent>(OnMouseEvent);

        UpdateBatchPipelines(App.Device.SwapchainFramebuffer.OutputDescription);
        _userBatchesInitialized = true;
    }

    protected bool OnMouseEvent(MouseEvent @event)
    {
        if (@event is { MouseButton: MouseButton.Right, Down: true })
        {
            _dragCamera = true;
        }
        else if (@event is { MouseButton: MouseButton.Right, Down: false })
        {
            _dragCamera = false;
        }
        else
        {
            HandleMouse(@event);
        }

        return true;
    }

    protected virtual void HandleMouse(MouseEvent e)
    {

    }

    private void UpdatePipelinesAndView()
    {
        var outputDesc = App.Device.SwapchainFramebuffer.OutputDescription;

        _mainBatch.UpdatePipelines(outputDescription: outputDesc);

        if (_userBatchesInitialized)
        {
            UpdateBatchPipelines(outputDesc);
        }

        CameraController.Camera.AspectRatio = App.Window.Width / (float)App.Window.Height;
    }

    protected virtual void UpdateBatchPipelines(OutputDescription outputDesc)
    {

    }

    protected override void Resize(Size size)
    {
        base.Resize(size);
        UpdatePipelinesAndView();
    }

    protected override void Update(FrameInfo frameInfo)
    {
        base.Update(frameInfo);
        UpdateCamera(frameInfo);
    }

    private void UpdateCamera(FrameInfo frameInfo)
    {
        if (!_imGui.Captured)
        {
            if (_dragCamera)
            {
                var delta = App.Input.MouseDelta / new Vector2(App.Window.Width, App.Window.Height) * new Vector2(-1, 1) * CameraController.Camera.Zoom * CamDragSpeed;
                CameraController.FuturePosition2 += delta;
            }

            CameraController.FutureZoom += App.Input.ScrollDelta * CamZoomSpeed * frameInfo.DeltaTime;
            CameraController.FutureZoom = Math.Clamp(CameraController.FutureZoom, MinZoom, MaxZoom);
        }

        CameraController.Update(frameInfo.DeltaTime);
    }

    private void CheckDisposed()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
    }

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

        _batches.Clear();
    }
}