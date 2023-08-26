using GameFramework;
using GameFramework.ImGui;
using ImGuiNET;
using Microsoft.Extensions.DependencyInjection;
using Veldrid;
using Veldrid.Sdl2;

namespace Common;

public abstract class VisualizationApp : GameApplication
{
    public string Name { get; }

    private string ImGuiName => $"imgui_{Name}.ini";

    public VisualizationApp(bool relative = false)
    {
        Name = GetType().Name;
        Device.SyncToVerticalBlank = true;
        Window.Title = Name;
        ClearColor = RgbaFloat.Black;
    }

    protected override void RegisterServices(ServiceCollection services)
    {
        services.AddSingleton<VisualizationApp>(_ => this);
        services.AddSingleton<ImGuiLayer>();
        
        RegisterLayer(services);

        base.RegisterServices(services);
    }

    protected abstract void RegisterLayer(ServiceCollection services);

    protected override void Initialize()
    {
        Layers.ConstructLayer<ImGuiLayer>(imGui =>
        {
            var io = ImGui.GetIO();
            io.ConfigFlags |= ImGuiConfigFlags.DockingEnable;
            ImGuiStyles.Dark();
            ImGui.LoadIniSettingsFromDisk(ImGuiName);
        });

        Construct();
    }

    protected abstract void Construct();

    protected override void Destroy()
    {
        ImGui.SaveIniSettingsToDisk(ImGuiName);
        base.Destroy();
    }

    private bool _relativeMode;
    private Point _initialMouse;

    public bool InCameraMode => _relativeMode;

    protected override void Update(FrameInfo frameInfo)
    {
        if (Input.KeyReleased(Key.Escape))
        {
            _relativeMode = !_relativeMode;
            
            Sdl2Native.SDL_SetRelativeMouseMode(_relativeMode);

            if (_relativeMode)
            {
                _initialMouse = new Point((int)Input.MousePosition.X, (int)Input.MousePosition.Y);
            }
            else
            {
                Window.SetMousePosition(_initialMouse.X, _initialMouse.Y);
            }

            Layers.BackToFront.FirstOrDefault(x => x is ImGuiLayer)?.Also(imGuiLayer =>
            {
                imGuiLayer.IsEnabled = !_relativeMode;
            });
        }

        base.Update(frameInfo);
    }
}