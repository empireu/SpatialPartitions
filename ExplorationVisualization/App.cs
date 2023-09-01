using Common;
using Microsoft.Extensions.DependencyInjection;

namespace ExplorationVisualization;

internal class App : VisualizationApp
{
    protected override void RegisterLayer(ServiceCollection services)
    {
        services.AddSingleton<WorldLayer>();
    }

    protected override void Construct()
    {
        Layers.ConstructLayer<WorldLayer>();
    }
}