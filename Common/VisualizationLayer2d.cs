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
    private const float MinZoom = 1.0f;
    private const float MaxZoom = 250f;
    private const float CamDragSpeed = 5f;
    private const float CamZoomSpeed = 50f;

    protected readonly GameApplication App;
    private readonly ImGuiLayer _imGui;

    private readonly OrthographicCameraController2D _cameraController;
    private readonly QuadBatch _batch;
    private readonly PostProcessor _postProcess;
    private bool _dragCamera;

    public VisualizationLayer2d(GameApplication app, ImGuiLayer imGui)
    {
        App = app;
        _imGui = imGui;

        _cameraController = new OrthographicCameraController2D(
            new OrthographicCamera(0, -1, 1),
            translationInterpolate: 25f,
            zoomInterpolate: 10f
        );

        _cameraController.FutureZoom = 35;

        _batch = app.Resources.BatchPool.Get();

        _postProcess = new PostProcessor(app)
        {
            BackgroundColor = RgbaFloat.Black
        };

        UpdatePipelines();

        imGui.Submit += ImGuiOnSubmit;
    }

    protected Vector2 Mouse => _cameraController.Camera.MouseToWorld2D(App.Input.MousePosition, App.Window.Width, App.Window.Height);
    protected Vector2di MouseI => Mouse.Map(mouse => new Vector2di((int)MathF.Round(mouse.X), (int)MathF.Round(mouse.Y)));

    protected abstract void ImGuiOnSubmit(ImGuiRenderer sender);

    private bool _rendering;
    
    protected void RenderPass(Action<QuadBatch> body)
    {
        if (!_rendering)
        {
            throw new InvalidOperationException();
        }

        _batch.Effects = QuadBatchEffects.Transformed(_cameraController.Camera.CameraMatrix);
        _batch.Clear();
        body(_batch);
        _batch.Submit(framebuffer: _postProcess.InputFramebuffer);
    }

    protected sealed override void Render(FrameInfo frameInfo)
    {
        _postProcess.ClearColor();
        _rendering = true;
        RenderStack();
        _rendering = false;
        _postProcess.Render();
    }

    protected abstract void RenderStack();

    protected override void OnAdded()
    {
        RegisterHandler<MouseEvent>(OnMouseEvent);
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

    private void UpdatePipelines()
    {
        _cameraController.Camera.AspectRatio = App.Window.Width / (float)App.Window.Height;

        _postProcess.ResizeInputs(App.Window.Size() * 2);
        _postProcess.SetOutput(App.Device.SwapchainFramebuffer);
        _batch.UpdatePipelines(outputDescription: _postProcess.InputFramebuffer.OutputDescription);
    }

    protected override void Resize(Size size)
    {
        base.Resize(size);

        UpdatePipelines();
    }

    private void UpdateCamera(FrameInfo frameInfo)
    {
        if (!_imGui.Captured)
        {
            if (_dragCamera)
            {
                var delta = (App.Input.MouseDelta / new Vector2(App.Window.Width, App.Window.Height)) * new Vector2(-1, 1) * _cameraController.Camera.Zoom * CamDragSpeed;
                _cameraController.FuturePosition2 += delta;
            }

            _cameraController.FutureZoom += App.Input.ScrollDelta * CamZoomSpeed * frameInfo.DeltaTime;
            _cameraController.FutureZoom = Math.Clamp(_cameraController.FutureZoom, MinZoom, MaxZoom);
        }

        _cameraController.Update(frameInfo.DeltaTime);
    }

    protected override void Update(FrameInfo frameInfo)
    {
        base.Update(frameInfo);
        UpdateCamera(frameInfo);
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