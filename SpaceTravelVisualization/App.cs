using Common;
using Microsoft.Extensions.DependencyInjection;

namespace SpaceTravelVisualization;

internal sealed class App : VisualizationApp
{
    public App() : base()
    {

    }

    protected override void RegisterLayer(ServiceCollection services)
    {
        services.AddSingleton<WorldLayer>();
    }

    protected override void Construct()
    {
        Layers.ConstructLayer<WorldLayer>();
    }
}