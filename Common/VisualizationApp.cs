using GameFramework;
using GameFramework.ImGui;
using ImGuiNET;
using Microsoft.Extensions.DependencyInjection;
using Veldrid;

namespace Common;

public abstract class VisualizationApp : GameApplication
{
    public string Name { get; }

    private string ImGuiName => $"imgui_{Name}.ini";

    public VisualizationApp()
    {
        Name = GetType().Name;
        Device.SyncToVerticalBlank = true;
        Window.Title = Name;
        ClearColor = RgbaFloat.Black;
    }

    protected override void RegisterServices(ServiceCollection services)
    {
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
}